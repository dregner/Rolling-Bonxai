#include "rolling_bonxai_ros/icp_frontend.hpp"

#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace RollingBonxai
{
namespace
{
constexpr double rad = 3.14159265358979323846 / 180.0;
IcpCloud::Ptr downsample(const IcpCloud& input, double voxel)
{
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(input.makeShared());
  grid.setLeafSize(voxel, voxel, voxel);
  auto output = pcl::make_shared<IcpCloud>();
  grid.filter(*output);
  return output;
}
double yaw(const Eigen::Isometry3d& T) { return std::atan2(T(1, 0), T(0, 0)); }
Eigen::Isometry3d planar(double x, double y, double angle)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, 0);
  return T;
}
}

void IcpConfig::validate() const
{
  if (voxels.empty() || voxels.size() != distances.size() || voxels.size() != iterations.size())
    throw std::invalid_argument("ICP scales must have matching, nonempty voxel/distance/iteration arrays");
  for (size_t i = 0; i < voxels.size(); ++i)
    if (!(voxels[i] > 0) || !(distances[i] > 0) || iterations[i] < 1)
      throw std::invalid_argument("ICP scales must be positive");
  if (!(min_range >= 0 && max_range > min_range && normal_radius > 0 && huber > 0 &&
        target_radius > 0 && max_target_points >= 100 && keyframe_translation > 0 &&
        keyframe_rotation > 0 && keyframe_interval > 0 && min_fitness > 0 && min_fitness <= 1 &&
        max_rmse > 0 && max_translation > 0 && max_rotation > 0 &&
        min_eigen_ratio >= 0 && min_eigen_ratio < 1))
    throw std::invalid_argument("Invalid ICP thresholds or point budget");
}
IcpFrontend::IcpFrontend(IcpConfig config) : config_(std::move(config)) { config_.validate(); }

IcpCloud::Ptr IcpFrontend::filter(const IcpCloud& scan, bool apply_range_window) const
{
  IcpCloud valid;
  for (const auto& p : scan) {
    const double r2 = p.getVector3fMap().squaredNorm();
    if (p.getVector3fMap().allFinite() && (!apply_range_window ||
        (r2 >= config_.min_range * config_.min_range && r2 < config_.max_range * config_.max_range)))
      valid.push_back(p);
  }
  return downsample(valid, config_.voxels.back());
}
bool IcpFrontend::isKeyframe(const Eigen::Isometry3d& prior, double stamp) const
{
  const auto d = last_prior_.inverse() * prior;
  return last_stamp_ < 0 || stamp - last_stamp_ >= config_.keyframe_interval ||
    d.translation().norm() >= config_.keyframe_translation ||
    Eigen::AngleAxisd(d.linear()).angle() >= config_.keyframe_rotation * rad;
}

IcpReport IcpFrontend::process(const IcpCloud& scan, const IcpCloud& target,
                              const Eigen::Isometry3d& prior, double stamp, bool target_ready)
{
  const auto start = std::chrono::steady_clock::now();
  IcpReport out;
  out.pose = correction_ * prior;
  auto finish = [&]() {
    out.milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    return out;
  };
  if (!isKeyframe(prior, stamp)) return finish();
  out.keyframe = true;
  if (scan.size() < 50) { out.reason = "too_few_scan_points"; return finish(); }
  if (!target_ready) { out.reason = "chunks_pending"; return finish(); }
  last_prior_ = prior;
  last_stamp_ = stamp;
  if (!config_.enabled || !initialized_) {
    out.accepted = out.integrate = true;
    out.reason = config_.enabled ? "bootstrap" : "prior_only";
    initialized_ = true;
    return finish();
  }
  // A sparse target after initialization is not sufficient evidence of new space.
  if (target.size() < 50) { out.reason = "insufficient_target"; return finish(); }

  // Solve a planar increment about the predicted sensor origin. Centering avoids
  // coupling yaw conditioning to distance from the world's origin.
  const Eigen::Isometry3d predicted = out.pose;
  const Eigen::Vector3d center = predicted.translation();
  IcpCloud source, centered_target;
  for (const auto& p : scan) {
    const Eigen::Vector3d v = predicted * p.getVector3fMap().cast<double>() - center;
    source.push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
  }
  for (const auto& p : target) {
    const Eigen::Vector3d v = p.getVector3fMap().cast<double>() - center;
    centered_target.push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
  }
  Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
  for (size_t scale = 0; scale < config_.voxels.size(); ++scale) {
    auto src = downsample(source, config_.voxels[scale]);
    auto tgt = downsample(centered_target, config_.voxels[scale]);
    if (src->size() < 50 || tgt->size() < 50) {
      out.reason = "too_few_scale_points"; return finish();
    }
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimation;
    estimation.setInputCloud(tgt);
    estimation.setRadiusSearch(std::max(config_.normal_radius, 3 * config_.voxels[scale]));
    pcl::PointCloud<pcl::Normal> normals;
    estimation.compute(normals);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(tgt);
    double x = delta.translation().x(), y = delta.translation().y(), angle = yaw(delta);
    // The extra evaluation computes gate metrics at the FINAL transform, rather
    // than using residuals from the iteration preceding the last update.
    for (int iteration = 0; iteration <= config_.iterations[scale]; ++iteration) {
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
      Eigen::Vector3d g = Eigen::Vector3d::Zero();
      double squared_error = 0;
      int count = 0;
      const auto T = planar(x, y, angle);
      std::vector<int> index(1);
      std::vector<float> distance(1);
      for (const auto& point : *src) {
        const Eigen::Vector3d p = T * point.getVector3fMap().cast<double>();
        pcl::PointXYZ query(p.x(), p.y(), p.z());
        if (!tree.nearestKSearch(query, 1, index, distance) ||
            distance[0] > config_.distances[scale] * config_.distances[scale]) continue;
        const Eigen::Vector3d n = normals[index[0]].getNormalVector3fMap().cast<double>();
        if (!n.allFinite() || n.squaredNorm() < 0.5) continue;
        const Eigen::Vector3d q = (*tgt)[index[0]].getVector3fMap().cast<double>();
        const double residual = n.dot(p - q);
        Eigen::Vector3d J(n.x(), n.y(), -n.x() * (p.y() - y) + n.y() * (p.x() - x));
        const double weight = std::min(1.0, config_.huber / std::max(std::abs(residual), 1e-12));
        H.noalias() += weight * J * J.transpose();
        g.noalias() += weight * J * residual;
        squared_error += residual * residual;
        ++count;
      }
      out.correspondences = count;
      out.fitness = double(count) / src->size();
      out.rmse = count ? std::sqrt(squared_error / count) : 0;
      if (count < 30) { out.reason = "too_few_correspondences"; return finish(); }
      // Express yaw as displacement at a one-metre lever arm.
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(H);
      if (eigen.info() != Eigen::Success || !H.allFinite()) {
        out.reason = "invalid_hessian"; return finish();
      }
      out.eigen_ratio = std::max(0.0, eigen.eigenvalues()[0]) /
        std::max(1e-12, eigen.eigenvalues()[2]);
      if (iteration == config_.iterations[scale]) break;
      Eigen::Vector3d step = -(H + 1e-9 * Eigen::Matrix3d::Identity()).ldlt().solve(g);
      if (!step.allFinite()) { out.reason = "invalid_solution"; return finish(); }
      x += step.x(); y += step.y(); angle += step.z();
      if (step.head<2>().norm() < 1e-5 && std::abs(step.z()) < 1e-5)
        iteration = config_.iterations[scale] - 1;
    }
    delta = planar(x, y, angle);
  }
  if (out.fitness < config_.min_fitness) out.reason = "low_overlap";
  else if (out.rmse > config_.max_rmse) out.reason = "high_rmse";
  else if (delta.translation().norm() > config_.max_translation) out.reason = "large_translation";
  else if (std::abs(yaw(delta)) > config_.max_rotation * rad) out.reason = "large_rotation";
  else if (out.eigen_ratio < config_.min_eigen_ratio) out.reason = "degenerate";
  else {
    Eigen::Isometry3d shift = Eigen::Isometry3d::Identity();
    shift.translation() = center;
    correction_ = shift * delta * shift.inverse() * correction_;
    out.pose = correction_ * prior;
    out.accepted = out.integrate = true;
    out.reason = "tracking";
  }
  return finish();
}
}  // namespace RollingBonxai
