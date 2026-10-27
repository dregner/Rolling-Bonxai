#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace RollingBonxai
{
using IcpCloud = pcl::PointCloud<pcl::PointXYZ>;

struct IcpConfig
{
  bool enabled{false};
  double min_range{0.3}, max_range{10.0};
  double keyframe_translation{0.10}, keyframe_rotation{5.0}, keyframe_interval{2.0};
  double target_radius{10.0}, normal_radius{0.30}, huber{0.05};
  std::vector<double> voxels{0.10, 0.05}, distances{0.80, 0.25};
  std::vector<int> iterations{40, 30};
  int max_target_points{60000};
  double min_fitness{0.60}, max_rmse{0.12};
  double max_translation{0.50}, max_rotation{10.0}, min_eigen_ratio{0.001};
  void validate() const;
};

struct IcpReport
{
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  bool keyframe{false}, accepted{false}, integrate{false};
  std::string reason{"not_keyframe"};
  double fitness{0}, rmse{0}, eigen_ratio{0}, milliseconds{0};
  int correspondences{0};
};

// ROS-independent registration. Owns only the correction and previous keyframe pose.
// The caller supplies an immutable, bounded target from the rolling map.
class IcpFrontend
{
public:
  explicit IcpFrontend(IcpConfig config);
  IcpCloud::Ptr filter(const IcpCloud& scan, bool apply_range_window = true) const;
  bool isKeyframe(const Eigen::Isometry3d& prior, double stamp) const;
  IcpReport process(const IcpCloud& scan, const IcpCloud& target,
                    const Eigen::Isometry3d& prior, double stamp, bool target_ready = true);
  const Eigen::Isometry3d& correction() const { return correction_; }
  const IcpConfig& config() const { return config_; }
private:
  IcpConfig config_;
  Eigen::Isometry3d correction_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_prior_{Eigen::Isometry3d::Identity()};
  double last_stamp_{-1};
  bool initialized_{false};
};
}  // namespace RollingBonxai
