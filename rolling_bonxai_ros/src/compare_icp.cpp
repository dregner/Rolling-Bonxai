// Deterministic bag replay without DDS drops or wall-clock playback effects.
#include "rolling_bonxai_ros/icp_frontend.hpp"
#include "rolling_bonxai_ros/sonar_clean_map.hpp"
#include "rolling_bonxai/rolling_occupancy_map.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <thread>
#include <sys/resource.h>

using namespace RollingBonxai;
struct QuietLogger : Logger {
  void log_debug(const std::string&) override {}
  void log_info(const std::string&) override {}
  void log_warn(const std::string& s) override { std::cerr << s << '\n'; }
  void log_error(const std::string& s) override { std::cerr << s << '\n'; }
  void log_fatal(const std::string& s) override { std::cerr << s << '\n'; }
};

int main(int argc, char** argv)
{
  if (argc != 4) {
    std::cerr << "usage: compare_icp INPUT.bin OUTPUT_DIRECTORY prior|icp|icp_clean\n";
    return 2;
  }
  const std::string mode(argv[3]);
  if (mode != "prior" && mode != "icp" && mode != "icp_clean") return 2;
  std::filesystem::path directory(argv[2]);
  if (std::filesystem::exists(directory)) {
    std::cerr << "Output directory already exists; use a fresh run directory\n";
    return 2;
  }
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) return 2;
  std::filesystem::create_directories(directory);
  RollingOccupancyMap::AllParameters params;
  params.map_params.resolution = 0.05;
  params.map_params.sensor_max_range = 10.0;
  params.map_params.occupancy_threshold = 0.5;
  params.map_params.occupancy_min_z = -100;
  params.map_params.occupancy_max_z = 100;
  params.map_params.sensor_hit = 0.7;
  params.map_params.sensor_miss = 0.4;
  params.map_params.sensor_min = 0.12;
  params.map_params.sensor_max = 0.97;
  params.chunk_size = 4.0;
  params.nb_radius = 3;
  params.asyncio.enable_io = true;
  params.full_save_folder_path = (directory / "chunks").string();
  RollingOccupancyMap map(params, std::make_shared<QuietLogger>());
  IcpConfig config;
  config.enabled = mode != "prior";
  const bool cleaning = mode == "icp_clean";
  SonarCleanConfig clean_config;
  SonarCoherenceFilter coherence(clean_config);
  std::unique_ptr<RollingSonarCleanMap> clean_map;
  if (cleaning) clean_map = std::make_unique<RollingSonarCleanMap>(clean_config, directory / "clean_chunks");
  IcpFrontend frontend(config);
  std::ofstream accepted_scans;
  if (cleaning) accepted_scans.open(directory / "accepted_keyframes.bin", std::ios::binary);
  std::ofstream csv(directory / "frames.csv");
  csv << "frame,stamp,held_out,keyframe,integrated,reason,fitness,rmse,eigen_ratio,correspondences,icp_ms,total_ms,map_bytes,active_chunks,target_points,dropped_points,x,y,z,yaw,heldout_mean_nn,heldout_overlap,raw_points,coherent_points,clean_points,clean_bytes,clean_chunks\n";
  int frame = 0;
  size_t max_map_bytes = 0;
  const auto start = std::chrono::steady_clock::now();
  while (true) {
    double stamp;
    input.read(reinterpret_cast<char*>(&stamp), sizeof(stamp));
    if (input.eof()) break;
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> matrix;
    uint32_t count;
    input.read(reinterpret_cast<char*>(matrix.data()), 16 * sizeof(double));
    input.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (!input || count > 1000000) return 3;
    IcpCloud raw;
    for (uint32_t i = 0; i < count; ++i) {
      float p[3];
      input.read(reinterpret_cast<char*>(p), sizeof(p));
      raw.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    }
    if (!input) return 3;
    const auto t0 = std::chrono::steady_clock::now();
    const Eigen::Isometry3d prior(matrix);
    CoherentScan coherent;
    if (cleaning) coherent = coherence.process(raw);
    auto scan = frontend.filter(cleaning ? coherent.points : raw, !cleaning);
    const auto predicted = frontend.correction() * prior;
    // Wait for map loads offline so disk latency never masquerades as no overlap.
    // The live node instead reports chunks_pending and bounds its incoming queue.
    if (cleaning) clean_map->prepare(predicted.translation());
    else map.prepareForPose(predicted.translation());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!map.getPendingChunks().empty()) {
      if (std::chrono::steady_clock::now() > deadline) return 4;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      map.prepareForPose(predicted.translation());
    }
    const bool held_out = frame % 5 == 4;
    std::vector<Eigen::Vector3d> local;
    // Prior-only mode also extracts targets for comparable evaluation and overhead.
    IcpCloud target;
    if (held_out || frontend.isKeyframe(prior, stamp)) {
      if (cleaning) clean_map->points(target, false, predicted.translation(), config.target_radius,
        held_out ? 0 : config.max_target_points);
      else {
        map.getLocalOccupiedPoints(predicted.translation(), config.target_radius,
          held_out ? std::numeric_limits<size_t>::max() : config.max_target_points, local);
        for (const auto& p : local) target.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
      }
    }
    IcpReport report;
    report.pose = predicted;
    double nn = -1, overlap = -1;
    if (held_out) {
      report.reason = "held_out";
      if (!target.empty() && !scan->empty()) {
        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(target.makeShared());
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        double sum = 0;
        size_t inliers = 0;
        for (const auto& p : *scan) {
          const Eigen::Vector3d v = predicted * p.getVector3fMap().cast<double>();
          tree.nearestKSearch(pcl::PointXYZ(v.x(), v.y(), v.z()), 1, indices, distances);
          const double d = std::sqrt(distances[0]);
          // Capped distance includes non-overlapping points, avoiding inlier-only bias.
          sum += std::min(d, 0.5);
          if (d <= 0.25) ++inliers;
        }
        nn = sum / scan->size(); overlap = double(inliers) / scan->size();
      }
    } else {
      report = frontend.process(*scan, target, prior, stamp);
      if (report.integrate && cleaning) {
        clean_map->update(coherent, report.pose);
        accepted_scans.write(reinterpret_cast<const char*>(&stamp), sizeof(stamp));
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> corrected(report.pose.matrix());
        accepted_scans.write(reinterpret_cast<const char*>(corrected.data()), 16 * sizeof(double));
        accepted_scans.write(reinterpret_cast<const char*>(&count), sizeof(count));
        for (const auto& p : raw) {
          const float point[3]{p.x, p.y, p.z};
          accepted_scans.write(reinterpret_cast<const char*>(point), sizeof(point));
        }
      }
      if (report.integrate && !cleaning) {
        map.prepareForPose(report.pose.translation());
        while (!map.getPendingChunks().empty()) {
          if (std::chrono::steady_clock::now() > deadline) return 4;
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          map.prepareForPose(report.pose.translation());
        }
        std::vector<Eigen::Vector3d> transformed;
        for (const auto& p : *scan)
          transformed.push_back(report.pose * p.getVector3fMap().cast<double>());
        const Eigen::Vector3d origin = report.pose.translation();
        const Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        map.insertPointCloud(transformed, origin, velocity, config.max_range);
      }
    }
    const auto bytes = cleaning ? clean_map->memoryBytes() : map.getMemoryUsage();
    IcpCloud clean_points;
    if (cleaning) clean_map->points(clean_points);
    max_map_bytes = std::max(max_map_bytes, bytes);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    csv << frame << ',' << std::fixed << std::setprecision(6) << stamp << ',' << held_out << ','
      << report.keyframe << ',' << report.integrate << ',' << report.reason << ',' << report.fitness
      << ',' << report.rmse << ',' << report.eigen_ratio << ',' << report.correspondences << ','
      << report.milliseconds << ',' << ms << ',' << bytes << ',' << (cleaning ? clean_map->activeChunks() : map.getActiveChunkCount()) << ','
      << target.size() << ',' << map.getDroppedPointCount() << ',' << report.pose.translation().x()
      << ',' << report.pose.translation().y() << ',' << report.pose.translation().z() << ','
      << std::atan2(report.pose(1, 0), report.pose(0, 0)) << ',' << nn << ',' << overlap << ',' << raw.size() << ',' << coherent.points.size() << ','
      << clean_points.size() << ',' << (cleaning ? clean_map->memoryBytes() : 0) << ','
      << (cleaning ? clean_map->activeChunks() : 0) << '\n';
    ++frame;
    if (frame % 100 == 0) std::cout << mode << ": " << frame << " frames\n" << std::flush;
  }
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  std::ofstream summary(directory / "runtime.json");
  summary << "{\"mode\":\"" << mode << "\",\"frames\":" << frame << ",\"seconds\":" << seconds
    << ",\"peak_rss_kib\":" << usage.ru_maxrss << ",\"peak_map_bytes\":" << max_map_bytes << "}\n";
  // Export only resident chunks; evicted chunks remain in the separate run's storage directory.
  std::vector<Eigen::Vector3d> points;
  if (cleaning) {
    IcpCloud surface;
    clean_map->points(surface, false);
    for (const auto& p : surface) points.push_back(p.getVector3fMap().cast<double>());
    clean_map->saveCleanPly(directory / "clean_map.ply");
  } else map.getOccupiedVoxels(points);
  std::ofstream ply(directory / "active_map.ply");
  ply << "ply\nformat ascii 1.0\nelement vertex " << points.size()
      << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
  for (const auto& p : points) ply << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
  std::cout << mode << ": finished " << frame << " frames in " << seconds << " s\n";
}
