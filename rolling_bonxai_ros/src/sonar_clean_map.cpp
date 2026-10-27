#include "rolling_bonxai_ros/sonar_clean_map.hpp"
#include "bonxai_core/serialization.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>

namespace RollingBonxai
{
namespace
{
constexpr double degrees = 180.0 / 3.14159265358979323846;
bool beam(const Eigen::Vector3d& p, const SonarCleanConfig& cfg, int& row, int& col, double& range)
{
  range = p.norm();
  if (!p.allFinite() || range < cfg.min_range || range > cfg.max_range) return false;
  const double az = std::atan2(p.y(), p.x()) * degrees;
  const double el = std::asin(std::clamp(p.z() / range, -1.0, 1.0)) * degrees;
  // Float32 points at nominal FOV edges can exceed the angle by ~2e-6 degrees.
  // Preserve these boundary beams, as the reference's clamped projection does.
  if (std::abs(az) > cfg.azimuth_fov / 2 + 1e-5 || std::abs(el) > cfg.elevation_fov / 2 + 1e-5)
    return false;
  col = std::clamp(int(std::nearbyint((az + cfg.azimuth_fov / 2) / cfg.azimuth_fov *
    (cfg.azimuth_beams - 1))), 0, cfg.azimuth_beams - 1);
  row = std::clamp(int(std::nearbyint((el + cfg.elevation_fov / 2) / cfg.elevation_fov *
    (cfg.elevation_beams - 1))), 0, cfg.elevation_beams - 1);
  return true;
}
Eigen::Vector3d centroid(const SonarEvidence& cell)
{
  return Eigen::Vector3d(cell.sum[0], cell.sum[1], cell.sum[2]) / std::max(1u, cell.points);
}
}

void SonarCleanConfig::validate() const
{
  if (!(azimuth_beams > 1 && elevation_beams > 1 && azimuth_fov > 0 && azimuth_fov <= 180 &&
        elevation_fov > 0 && elevation_fov < 180 && min_range > 0 && max_range > min_range &&
        window >= 1 && window % 2 == 1 && window <= 31 && min_neighbors >= 0 &&
        min_neighbors < window * window && min_component >= 1 && range_abs >= 0 && range_rel >= 0 &&
        voxel > 0 && miss_abs >= 0 && miss_rel >= 0 && min_hits >= 1 &&
        min_ratio > 0 && min_ratio <= 1 && chunk_size > 0 && chunk_radius >= 1 &&
        hysteresis_ratio >= 0 && hysteresis_ratio < 0.5 && projection_offset.allFinite() &&
        chunk_size * (chunk_radius - hysteresis_ratio) >= max_range + projection_offset.norm() + voxel))
    throw std::invalid_argument("Invalid sonar clean-map parameters or chunk window smaller than sensor range");
}

SonarCoherenceFilter::SonarCoherenceFilter(SonarCleanConfig config) : config_(std::move(config))
{ config_.validate(); }

CoherentScan SonarCoherenceFilter::process(const IcpCloud& scan) const
{
  const auto& cfg = config_;
  const int width = cfg.azimuth_beams, height = cfg.elevation_beams;
  CoherentScan out;
  out.ranges.assign(width * height, std::numeric_limits<float>::quiet_NaN());
  std::vector<int> indices(width * height, -1);
  for (size_t i = 0; i < scan.size(); ++i) {
    const Eigen::Vector3d p = scan[i].getVector3fMap().cast<double>() - cfg.projection_offset;
    int row, col; double range;
    if (!beam(p, cfg, row, col, range)) continue;
    // The reference range image uses the last return when a beam has duplicates.
    out.ranges[row * width + col] = range;
    indices[row * width + col] = int(i);
  }
  std::vector<uint8_t> keep(width * height, 0), seen(width * height, 0);
  const int half = cfg.window / 2;
  for (int row = 0; row < height; ++row) for (int col = 0; col < width; ++col) {
    const int index = row * width + col;
    const float range = out.ranges[index];
    if (!std::isfinite(range)) continue;
    int count = 0;
    for (int dy = -half; dy <= half; ++dy) for (int dx = -half; dx <= half; ++dx) {
      if ((!dx && !dy) || row + dy < 0 || row + dy >= height || col + dx < 0 || col + dx >= width) continue;
      if (std::abs(out.ranges[(row + dy) * width + col + dx] - range) <= cfg.range_abs + cfg.range_rel * range)
        ++count;
    }
    keep[index] = count >= cfg.min_neighbors;
  }
  // Eight-connected components, matching scipy.ndimage.label(..., ones((3, 3))).
  for (int index = 0; index < width * height; ++index) {
    if (!keep[index] || seen[index]) continue;
    std::vector<int> component{index};
    seen[index] = 1;
    for (size_t i = 0; i < component.size(); ++i) {
      const int row = component[i] / width, col = component[i] % width;
      for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        if (row + dy < 0 || row + dy >= height || col + dx < 0 || col + dx >= width) continue;
        const int next = (row + dy) * width + col + dx;
        if (keep[next] && !seen[next]) { seen[next] = 1; component.push_back(next); }
      }
    }
    if (component.size() < size_t(cfg.min_component)) for (int i : component) keep[i] = 0;
  }
  for (int i = 0; i < width * height; ++i) {
    if (keep[i]) {
      out.points.push_back(scan[indices[i]]);
      out.input_indices.push_back(indices[i]);
    } else out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
  }
  return out;
}

RollingSonarCleanMap::RollingSonarCleanMap(SonarCleanConfig config, std::filesystem::path directory)
 : config_(std::move(config)), directory_(std::move(directory))
{
  config_.validate();
  std::filesystem::create_directories(directory_);
  // Fail on incompatible evidence instead of interpreting old cells with new thresholds/geometry.
  std::ostringstream manifest;
  manifest << std::setprecision(17) << "RollingSonarCleanMap-v1\n" << config_.voxel << ' '
    << config_.chunk_size << ' ' << sizeof(SonarEvidence) << '\n'
    << config_.azimuth_beams << ' ' << config_.elevation_beams << ' '
    << config_.azimuth_fov << ' ' << config_.elevation_fov << '\n'
    << config_.min_range << ' ' << config_.max_range << ' ' << config_.projection_offset.transpose() << '\n'
    << config_.window << ' ' << config_.min_neighbors << ' ' << config_.min_component << ' '
    << config_.range_abs << ' ' << config_.range_rel << ' ' << config_.miss_abs << ' ' << config_.miss_rel << '\n';
  const auto path = directory_ / "format.txt";
  if (std::filesystem::exists(path)) {
    std::ifstream input(path);
    std::string previous((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    if (previous != manifest.str()) throw std::runtime_error("Incompatible clean-map chunk directory");
  } else {
    std::ofstream output(path); output << manifest.str();
    if (!output) throw std::runtime_error("Cannot write clean-map manifest");
  }
}
RollingSonarCleanMap::Key RollingSonarCleanMap::chunkKey(const Eigen::Vector3d& p) const
{
  return {int32_t(std::round(p.x() / config_.chunk_size)),
          int32_t(std::round(p.y() / config_.chunk_size)), int32_t(std::round(p.z() / config_.chunk_size))};
}
bool RollingSonarCleanMap::inWindow(const Key& key) const
{
  for (int i = 0; i < 3; ++i) if (std::abs(int64_t(key[i]) - reference_[i]) > config_.chunk_radius) return false;
  return true;
}
bool RollingSonarCleanMap::confirmed(const SonarEvidence& cell) const
{
  return cell.hits >= config_.min_hits && double(cell.hits) /
    std::max(1.0, double(cell.hits) + cell.misses) >= config_.min_ratio;
}
std::filesystem::path RollingSonarCleanMap::chunkPath(const Key& key) const
{
  return directory_ / (std::to_string(key[0]) + "_" + std::to_string(key[1]) + "_" +
    std::to_string(key[2]) + ".clean");
}
std::unique_ptr<RollingSonarCleanMap::Grid> RollingSonarCleanMap::load(const std::filesystem::path& path) const
{
  std::ifstream stream(path, std::ios::binary);
  std::string header;
  if (!std::getline(stream, header)) throw std::runtime_error("Cannot read clean chunk: " + path.string());
  auto info = Bonxai::GetHeaderInfo(header);
  if (std::abs(info.resolution - config_.voxel) > 1e-7)
    throw std::runtime_error("Clean chunk resolution mismatch");
  auto grid = std::make_unique<Grid>(Bonxai::Deserialize<SonarEvidence>(stream, info));
  if (!stream) throw std::runtime_error("Truncated clean chunk: " + path.string());
  return grid;
}
void RollingSonarCleanMap::save(const Key& key, Chunk& chunk)
{
  if (!chunk.dirty) return;
  const auto destination = chunkPath(key);
  auto temporary = destination; temporary += ".tmp";
  {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    Bonxai::Serialize(stream, *chunk.grid);
    stream.flush();
    if (!stream) throw std::runtime_error("Cannot save clean chunk: " + destination.string());
  }
  std::filesystem::rename(temporary, destination);
  chunk.dirty = false;
  ++saved_chunks_;
}
void RollingSonarCleanMap::prepare(const Eigen::Vector3d& position)
{
  if (prepared_) {
    const Eigen::Vector3d reference_center(reference_[0] * config_.chunk_size,
      reference_[1] * config_.chunk_size, reference_[2] * config_.chunk_size);
    if ((position - reference_center).cwiseAbs().maxCoeff() <= config_.chunk_size *
        (0.5 + config_.hysteresis_ratio)) return;
  }
  const Key next = chunkKey(position);
  if (prepared_ && next == reference_) return;
  reference_ = next; prepared_ = true;
  for (auto it = chunks_.begin(); it != chunks_.end();) {
    if (!inWindow(it->first)) { save(it->first, it->second); it = chunks_.erase(it); }
    else ++it;
  }
  for (int x = -config_.chunk_radius; x <= config_.chunk_radius; ++x)
    for (int y = -config_.chunk_radius; y <= config_.chunk_radius; ++y)
      for (int z = -config_.chunk_radius; z <= config_.chunk_radius; ++z) {
        Key key{reference_[0] + x, reference_[1] + y, reference_[2] + z};
        if (chunks_.count(key)) continue;
        const auto path = chunkPath(key);
        if (std::filesystem::exists(path)) {
          chunks_.emplace(key, Chunk{load(path), false}); ++loaded_chunks_;
        }
      }
}
void RollingSonarCleanMap::update(const CoherentScan& scan, const Eigen::Isometry3d& pose,
                                 const std::vector<Eigen::Vector3f>& colors)
{
  if (scan.ranges.size() != size_t(config_.azimuth_beams * config_.elevation_beams) ||
      (!colors.empty() && colors.size() != scan.points.size()))
    throw std::invalid_argument("Invalid coherent scan layout or colors");
  prepare(pose.translation());
  const auto inverse = pose.inverse();
  // Missing/incoherent beams and surfaces behind the first return cast NO miss votes.
  for (auto& entry : chunks_) entry.second.grid->forEachCell(
    [&](SonarEvidence& cell, const Bonxai::CoordT& coord) {
      Eigen::Vector3d center((coord.x + 0.5) * config_.voxel,
                            (coord.y + 0.5) * config_.voxel, (coord.z + 0.5) * config_.voxel);
      int row, col; double range;
      if (!beam(inverse * center - config_.projection_offset, config_, row, col, range)) return;
      const float observed = scan.ranges[row * config_.azimuth_beams + col];
      if (std::isfinite(observed) && range < observed - (config_.miss_abs + config_.miss_rel * range)) {
        if (cell.misses == std::numeric_limits<uint32_t>::max()) throw std::overflow_error("Clean miss votes overflow");
        ++cell.misses; entry.second.dirty = true;
      }
    });
  // Aggregate a keyframe before touching cells: ten points in one voxel still give ONE hit.
  std::map<Key, SonarEvidence> aggregates;
  for (size_t i = 0; i < scan.points.size(); ++i) {
    const Eigen::Vector3d p = pose * scan.points[i].getVector3fMap().cast<double>();
    Key voxel{int32_t(std::floor(p.x() / config_.voxel)), int32_t(std::floor(p.y() / config_.voxel)),
              int32_t(std::floor(p.z() / config_.voxel))};
    auto& cell = aggregates[voxel];
    ++cell.points;
    for (int axis = 0; axis < 3; ++axis) {
      cell.sum[axis] += p[axis];
      cell.color_sum[axis] += colors.empty() ? 255.0f : colors[i][axis];
    }
  }
  for (const auto& entry : aggregates) {
    const auto& key = entry.first;
    const Eigen::Vector3d center((key[0] + 0.5) * config_.voxel,
      (key[1] + 0.5) * config_.voxel, (key[2] + 0.5) * config_.voxel);
    const Key chunk_key = chunkKey(center);
    if (!inWindow(chunk_key)) throw std::runtime_error("Clean evidence outside loaded chunk window");
    auto it = chunks_.find(chunk_key);
    // Small leaves avoid allocating 512 large evidence records around a thin surface.
    if (it == chunks_.end()) it = chunks_.emplace(chunk_key, Chunk{std::make_unique<Grid>(config_.voxel, 2, 2), false}).first;
    auto accessor = it->second.grid->createAccessor();
    auto* cell = accessor.value(Bonxai::CoordT{key[0], key[1], key[2]}, true);
    if (cell->hits == std::numeric_limits<uint32_t>::max() ||
        cell->points > std::numeric_limits<uint32_t>::max() - entry.second.points)
      throw std::overflow_error("Clean hit votes overflow");
    ++cell->hits; cell->points += entry.second.points;
    for (int axis = 0; axis < 3; ++axis) {
      cell->sum[axis] += entry.second.sum[axis]; cell->color_sum[axis] += entry.second.color_sum[axis];
    }
    it->second.dirty = true;
  }
}
void RollingSonarCleanMap::points(IcpCloud& output, bool clean, const Eigen::Vector3d& center,
                                double radius, size_t limit, std::vector<Eigen::Vector3f>* colors) const
{
  output.clear(); if (colors) colors->clear();
  uint64_t random = 42; size_t seen = 0;
  for (const auto& entry : chunks_) entry.second.grid->forEachCell(
    [&](const SonarEvidence& cell, const Bonxai::CoordT&) {
      if (!cell.points || (clean && !confirmed(cell))) return;
      const Eigen::Vector3d p = centroid(cell);
      if (radius > 0 && (p - center).squaredNorm() > radius * radius) return;
      ++seen;
      size_t index = output.size();
      if (limit && index >= limit) {
        random ^= random << 13; random ^= random >> 7; random ^= random << 17;
        index = random % seen; if (index >= limit) return;
      }
      const pcl::PointXYZ point(p.x(), p.y(), p.z());
      const Eigen::Vector3f color(cell.color_sum[0] / cell.points,
                                cell.color_sum[1] / cell.points, cell.color_sum[2] / cell.points);
      if (index == output.size()) { output.push_back(point); if (colors) colors->push_back(color); }
      else { output[index] = point; if (colors) (*colors)[index] = color; }
    });
}
void RollingSonarCleanMap::flush() { for (auto& entry : chunks_) save(entry.first, entry.second); }
size_t RollingSonarCleanMap::memoryBytes() const
{
  size_t bytes = 0;
  for (const auto& entry : chunks_) bytes += entry.second.grid->memUsage();
  return bytes;
}
size_t RollingSonarCleanMap::activeCells() const
{
  size_t cells = 0;
  for (const auto& entry : chunks_) cells += entry.second.grid->activeCellsCount();
  return cells;
}
std::vector<std::pair<Eigen::Vector3d, bool>> RollingSonarCleanMap::chunkStates() const
{
  std::vector<std::pair<Eigen::Vector3d, bool>> states;
  for (const auto& entry : chunks_) states.emplace_back(
    Eigen::Vector3d(entry.first[0] * config_.chunk_size, entry.first[1] * config_.chunk_size,
      entry.first[2] * config_.chunk_size), entry.second.dirty);
  return states;
}
size_t RollingSonarCleanMap::saveCleanPly(const std::filesystem::path& path)
{
  flush();
  size_t count = 0;
  for (const auto& file : std::filesystem::directory_iterator(directory_)) {
    if (file.path().extension() != ".clean") continue;
    auto grid = load(file.path());
    grid->forEachCell([&](const SonarEvidence& cell, const Bonxai::CoordT&) { if (confirmed(cell)) ++count; });
  }
  auto temporary = path; temporary += ".tmp";
  std::ofstream out(temporary);
  out << "ply\nformat ascii 1.0\nelement vertex " << count
      << "\nproperty float x\nproperty float y\nproperty float z\n"
      << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
  out << std::setprecision(9);
  for (const auto& file : std::filesystem::directory_iterator(directory_)) {
    if (file.path().extension() != ".clean") continue;
    auto grid = load(file.path());
    grid->forEachCell([&](const SonarEvidence& cell, const Bonxai::CoordT&) {
      if (!confirmed(cell)) return;
      const auto p = centroid(cell);
      out << p.x() << ' ' << p.y() << ' ' << p.z();
      for (int axis = 0; axis < 3; ++axis)
        out << ' ' << int(std::clamp(cell.color_sum[axis] / std::max(1u, cell.points), 0.0f, 255.0f));
      out << '\n';
    });
  }
  out.flush(); if (!out) throw std::runtime_error("Cannot export clean PLY");
  out.close(); std::filesystem::rename(temporary, path);
  return count;
}
}  // namespace RollingBonxai
