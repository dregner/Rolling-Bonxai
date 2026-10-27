#pragma once

#include "rolling_bonxai_ros/icp_frontend.hpp"
#include "bonxai_core/bonxai.hpp"
#include <array>
#include <filesystem>
#include <map>
#include <memory>

namespace RollingBonxai
{
struct SonarCleanConfig
{
  bool enabled{false}, build_occupancy{false};
  int azimuth_beams{256}, elevation_beams{64};
  double azimuth_fov{90}, elevation_fov{40};
  double min_range{0.3}, max_range{10};
  Eigen::Vector3d projection_offset{Eigen::Vector3d::Zero()};
  int window{5}, min_neighbors{10}, min_component{60};
  double range_abs{0.10}, range_rel{0.02};
  double voxel{0.03}, miss_abs{0.08}, miss_rel{0.02};
  uint32_t min_hits{2};
  double min_ratio{0.7}, chunk_size{4.0}, hysteresis_ratio{0.2};
  int chunk_radius{3};
  void validate() const;
};

struct CoherentScan
{
  IcpCloud points;                       // Original point coordinates, before voxel sampling.
  std::vector<size_t> input_indices;     // Carries RGB through the same selection in the ROS node.
  std::vector<float> ranges;             // NaN means no usable observation, never free space.
};

class SonarCoherenceFilter
{
public:
  explicit SonarCoherenceFilter(SonarCleanConfig config);
  CoherentScan process(const IcpCloud& scan) const;
private:
  SonarCleanConfig config_;
};

// Surface-only evidence: no free-space cells and no stored scans/keyframes.
// Hits/misses are distinct keyframe votes, independent of point count per cell.
struct SonarEvidence
{
  double sum[3]{0, 0, 0};
  float color_sum[3]{0, 0, 0};
  uint32_t points{0}, hits{0}, misses{0};
};

class RollingSonarCleanMap
{
public:
  using Key = std::array<int32_t, 3>;
  using Grid = Bonxai::VoxelGrid<SonarEvidence>;
  RollingSonarCleanMap(SonarCleanConfig config, std::filesystem::path directory);
  // Single caller thread. Synchronous, atomic chunk saves; no unbounded I/O queue.
  void prepare(const Eigen::Vector3d& position);
  void update(const CoherentScan& scan, const Eigen::Isometry3d& pose,
              const std::vector<Eigen::Vector3f>& colors = {});
  void points(IcpCloud& output, bool confirmed = true,
              const Eigen::Vector3d& center = Eigen::Vector3d::Zero(),
              double radius = 0, size_t limit = 0,
              std::vector<Eigen::Vector3f>* colors = nullptr) const;
  void flush();
  // Stream all chunks to PLY one at a time, including evicted geometry.
  size_t saveCleanPly(const std::filesystem::path& path);
  size_t memoryBytes() const;
  size_t activeChunks() const { return chunks_.size(); }
  size_t activeCells() const;
  std::vector<std::pair<Eigen::Vector3d, bool>> chunkStates() const;
  size_t savedChunks() const { return saved_chunks_; }
  size_t loadedChunks() const { return loaded_chunks_; }
  const SonarCleanConfig& config() const { return config_; }
private:
  struct Chunk {
    std::unique_ptr<Grid> grid;
    bool dirty{false};
  };
  SonarCleanConfig config_;
  std::filesystem::path directory_;
  std::map<Key, Chunk> chunks_;
  Key reference_{0, 0, 0};
  bool prepared_{false};
  size_t saved_chunks_{0}, loaded_chunks_{0};
  Key chunkKey(const Eigen::Vector3d& p) const;
  bool inWindow(const Key& key) const;
  bool confirmed(const SonarEvidence& cell) const;
  std::filesystem::path chunkPath(const Key& key) const;
  std::unique_ptr<Grid> load(const std::filesystem::path& path) const;
  void save(const Key& key, Chunk& chunk);
};
}  // namespace RollingBonxai
