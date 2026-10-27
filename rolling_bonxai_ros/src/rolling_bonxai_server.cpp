#include "rolling_bonxai_ros/rolling_bonxai_server.hpp"
#include "rolling_bonxai_ros/body_tf_listener.hpp"

#include <tf2/exceptions.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>

namespace
{
inline double bytesToMiB(std::size_t bytes)
{
  return static_cast<double>(bytes) / (1024.0 * 1024.0);
}
}

namespace RollingBonxai
{

RollingBonxaiServer::RollingBonxaiServer(
  const rclcpp::NodeOptions& options)
: rclcpp::Node("rolling_bonxai_server", options)
{
  loadParameters();
  initLogger();
  initTF();
  initMap();
  initSubscriber();
  initService();
  initStatsPublishers();
  initChunkVisualization();

  RCLCPP_INFO(get_logger(), "RollingBonxaiServer started");
  if (clean_map_) {
    RCLCPP_INFO(get_logger(),
      "Clean cloud: /global_map_clean (frame=%s, min_hits=%u); chunks=%.2f m, radius=%d, window=%.2f m",
      reconstruction_frame_.c_str(), clean_config_.min_hits, clean_config_.chunk_size,
      clean_config_.chunk_radius, clean_config_.chunk_size * (2 * clean_config_.chunk_radius + 1));
  }
}

RollingBonxaiServer::~RollingBonxaiServer()
{
  if (clean_map_) {
    try { clean_map_->flush(); }
    catch (const std::exception& ex) { RCLCPP_ERROR(get_logger(), "Clean-map shutdown save failed: %s", ex.what()); }
  }
}

void RollingBonxaiServer::loadParameters()
{
  server_params_.frame_id = declare_parameter<std::string>("frame_id", "map");
  server_params_.base_frame_id = declare_parameter<std::string>("base_frame_id", "base_footprint");
  server_params_.topic_in = declare_parameter<std::string>("topic_in", "/points");

  server_params_.cleanup_interval_sec =
    declare_parameter<double>("server.cleanup_interval_sec", 300.0);

  server_params_.enable_stats =
    declare_parameter<bool>("stats.enable_stats", true);

  server_params_.quick_stats =
    declare_parameter<bool>("stats.quick_stats", true);

  server_params_.publish_occupied_voxels =
    declare_parameter<bool>("stats.publish_occupied_voxels", true);

  server_params_.stats_publish_rate =
    declare_parameter<double>("stats.publish_rate", 1.0);

  IcpConfig ic;
  ic.enabled = declare_parameter<bool>("icp.enabled", false);
  ic.min_range = declare_parameter<double>("icp.min_range", ic.min_range);
  ic.max_range = declare_parameter<double>("icp.max_range", ic.max_range);
  ic.keyframe_translation = declare_parameter<double>("icp.keyframe_translation", ic.keyframe_translation);
  ic.keyframe_rotation = declare_parameter<double>("icp.keyframe_rotation_deg", ic.keyframe_rotation);
  ic.keyframe_interval = declare_parameter<double>("icp.keyframe_interval", ic.keyframe_interval);
  ic.target_radius = declare_parameter<double>("icp.target_radius", ic.target_radius);
  ic.normal_radius = declare_parameter<double>("icp.normal_radius", ic.normal_radius);
  ic.huber = declare_parameter<double>("icp.huber", ic.huber);
  ic.voxels = declare_parameter<std::vector<double>>("icp.voxels", ic.voxels);
  ic.distances = declare_parameter<std::vector<double>>("icp.max_distances", ic.distances);
  auto iterations = declare_parameter<std::vector<int64_t>>("icp.iterations", {40, 30});
  ic.iterations.assign(iterations.begin(), iterations.end());
  ic.max_target_points = declare_parameter<int>("icp.max_target_points", ic.max_target_points);
  ic.min_fitness = declare_parameter<double>("icp.min_fitness", ic.min_fitness);
  ic.max_rmse = declare_parameter<double>("icp.max_rmse", ic.max_rmse);
  ic.max_translation = declare_parameter<double>("icp.max_translation", ic.max_translation);
  ic.max_rotation = declare_parameter<double>("icp.max_rotation_deg", ic.max_rotation);
  ic.min_eigen_ratio = declare_parameter<double>("icp.min_eigen_ratio", ic.min_eigen_ratio);
  pose_time_offset_ = declare_parameter<double>("icp.pose_time_offset", 0.0);
  max_tf_wait_ = declare_parameter<double>("icp.max_tf_wait", 3.0);
  reconstruction_frame_ = declare_parameter<std::string>("icp.reconstruction_frame", "reconstruction");
  ignore_static_body_tf_ = declare_parameter<bool>("icp.ignore_static_body_tf", false);
  if (max_tf_wait_ <= 0 || (ic.enabled && reconstruction_frame_ == server_params_.frame_id))
    throw std::invalid_argument("Invalid TF wait or reconstruction frame");
  icp_ = std::make_unique<IcpFrontend>(ic);

  auto& mp = rolling_params_.map_params;

  mp.resolution = declare_parameter<double>("occupancy.resolution", 0.05);
  mp.occupancy_min_z = declare_parameter<double>("occupancy.occupancy_min_z", -100.0);
  mp.occupancy_max_z = declare_parameter<double>("occupancy.occupancy_max_z", 100.0);
  mp.occupancy_threshold = declare_parameter<double>("occupancy.occupancy_threshold", 0.5);

  mp.sensor_max_range = declare_parameter<double>("sensor_model.max_range", 30.0);
  mp.sensor_hit = declare_parameter<double>("sensor_model.hit", 0.7);
  mp.sensor_miss = declare_parameter<double>("sensor_model.miss", 0.4);
  mp.sensor_min = declare_parameter<double>("sensor_model.min", 0.12);
  mp.sensor_max = declare_parameter<double>("sensor_model.max", 0.97);

  rolling_params_.chunk_size =
    declare_parameter<double>("rolling.coordinate_system.chunk_size", 5.0);

  rolling_params_.hysteresis_ratio =
    declare_parameter<double>("rolling.transition_manager.hysteresis_ratio", 0.20);

  rolling_params_.loading_policy_name =
    declare_parameter<std::string>("rolling.loading_policy.name", "neighbourhood");

  rolling_params_.nb_radius =
    declare_parameter<int>("rolling.loading_policy.neighborhood.radius", 2);

  rolling_params_.planar_z_min =
    declare_parameter<int>("rolling.loading_policy.planar.z_min", 1);

  rolling_params_.planar_z_max =
    declare_parameter<int>("rolling.loading_policy.planar.z_max", 3);

  rolling_params_.planar_use_relative =
    declare_parameter<bool>("rolling.loading_policy.planar.use_relative", false);

  rolling_params_.temporal_age_weightage =
    declare_parameter<double>("rolling.loading_policy.temporal.age_weightage", 0.3);

  rolling_params_.temporal_max_age_s =
    declare_parameter<int>("rolling.loading_policy.temporal.max_age_s", 300);

  rolling_params_.full_save_folder_path =
    declare_parameter<std::string>("rolling.storage.full_save_folder_path", "");

  rolling_params_.asyncio.enable_io = declare_parameter<bool>("rolling.asyncio.enable_io",true);
  
  rolling_params_.asyncio.num_load_threads =
    static_cast<size_t>(declare_parameter<int>("rolling.asyncio.num_load_threads", 3));

  rolling_params_.asyncio.num_save_threads =
    static_cast<size_t>(declare_parameter<int>("rolling.asyncio.num_save_threads", 1));

  auto& clean = clean_config_;
  clean.enabled = declare_parameter<bool>("clean.enabled", false);
  clean.build_occupancy = declare_parameter<bool>("clean.build_occupancy", false);
  clean.azimuth_beams = declare_parameter<int>("clean.azimuth_beams", clean.azimuth_beams);
  clean.elevation_beams = declare_parameter<int>("clean.elevation_beams", clean.elevation_beams);
  clean.azimuth_fov = declare_parameter<double>("clean.azimuth_fov", clean.azimuth_fov);
  clean.elevation_fov = declare_parameter<double>("clean.elevation_fov", clean.elevation_fov);
  clean.window = declare_parameter<int>("clean.window", clean.window);
  clean.min_neighbors = declare_parameter<int>("clean.min_neighbors", clean.min_neighbors);
  clean.min_component = declare_parameter<int>("clean.min_component", clean.min_component);
  clean.range_abs = declare_parameter<double>("clean.range_abs", clean.range_abs);
  clean.range_rel = declare_parameter<double>("clean.range_rel", clean.range_rel);
  clean.voxel = declare_parameter<double>("clean.voxel", clean.voxel);
  clean.miss_abs = declare_parameter<double>("clean.miss_abs", clean.miss_abs);
  clean.miss_rel = declare_parameter<double>("clean.miss_rel", clean.miss_rel);
  const int min_hits = declare_parameter<int>("clean.min_hits", clean.min_hits);
  if (min_hits < 1) throw std::invalid_argument("clean.min_hits must be positive");
  clean.min_hits = min_hits;
  clean.min_ratio = declare_parameter<double>("clean.min_ratio", clean.min_ratio);
  auto offset = declare_parameter<std::vector<double>>("clean.projection_offset", {0.0, 0.0, 0.0});
  if (offset.size() != 3) throw std::invalid_argument("clean.projection_offset requires three values");
  clean.projection_offset = Eigen::Vector3d(offset[0], offset[1], offset[2]);
  clean.min_range = ic.min_range; clean.max_range = ic.max_range;
  clean.chunk_size = rolling_params_.chunk_size; clean.chunk_radius = rolling_params_.nb_radius;
  clean.hysteresis_ratio = rolling_params_.hysteresis_ratio;
  clean_save_path_ = declare_parameter<std::string>("clean.save_path", "/tmp/rolling_clean_map.ply");
  clean_publish_max_points_ = declare_parameter<int>("clean.publish_max_points", 200000);
  clean_publish_rate_ = declare_parameter<double>("clean.publish_rate", 1.0);
  if (clean.enabled) {
    if (!ic.enabled || !rolling_params_.asyncio.enable_io || clean_publish_max_points_ < 1 ||
        !(clean_publish_rate_ > 0)) throw std::invalid_argument("Clean mapping requires ICP, persistent I/O, and positive publication limits");
    clean.validate();
    coherence_filter_ = std::make_unique<SonarCoherenceFilter>(clean);
  }

}

void RollingBonxaiServer::initLogger()
{
  logger_ = std::make_shared<RosLogger>(get_logger());
}

void RollingBonxaiServer::initTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  if (icp_->config().enabled && ignore_static_body_tf_)
    tf_listener_ = std::make_shared<BodyTfListener>(*tf_buffer_, this,
      server_params_.frame_id, server_params_.base_frame_id);
  else
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  correction_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void RollingBonxaiServer::initMap()
{
  if (clean_config_.enabled)
    clean_map_ = std::make_unique<RollingSonarCleanMap>(clean_config_,
      std::filesystem::path(rolling_params_.full_save_folder_path) / "sonar_evidence");
  occupancy_map_ =
    std::make_unique<RollingBonxai::RollingOccupancyMap>(rolling_params_, logger_);
}

void RollingBonxaiServer::initSubscriber()
{
  icp_diagnostics_pub_ = create_publisher<std_msgs::msg::String>("~/icp_diagnostics", 10);
  corrected_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/corrected_sensor_pose", 10);
  cloud_timer_ = create_wall_timer(std::chrono::milliseconds(20),
    std::bind(&RollingBonxaiServer::processCloudQueue, this));
  if (icp_->config().enabled)
    correction_timer_ = create_wall_timer(std::chrono::milliseconds(100),
      [this]() { publishCorrection(now()); });
  if (clean_map_) {
    clean_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/global_map_clean", rclcpp::QoS(1).transient_local());
    clean_publish_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / clean_publish_rate_),
      std::bind(&RollingBonxaiServer::publishCleanMap, this));
    save_clean_srv_ = create_service<std_srvs::srv::Trigger>("~/save_clean_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
          const auto count = clean_map_->saveCleanPly(clean_save_path_);
          response->success = true;
          response->message = "Saved " + std::to_string(count) + " clean points to " + clean_save_path_;
        } catch (const std::exception& ex) { response->success = false; response->message = ex.what(); }
      });
  }
  pointcloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
      server_params_.topic_in,
      rclcpp::SensorDataQoS(),
      std::bind(&RollingBonxaiServer::pointcloudCallback, this, std::placeholders::_1));
}

void RollingBonxaiServer::initService()
{
  clean_memory_srv_ =
    create_service<std_srvs::srv::Trigger>(
      "/rolling_bonxai/clean_memory",
      std::bind(
        &RollingBonxaiServer::cleanMemoryServiceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

void RollingBonxaiServer::initStatsPublishers()
{
  if (!server_params_.enable_stats) {
    return;
  }

  if (server_params_.publish_occupied_voxels) {
    voxel_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "/rolling_bonxai/occupied_voxels",
        rclcpp::QoS(1).transient_local());
  }

  chunk_stats_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::ChunkStats>("/rolling_bonxai/chunk_stats", 10);

  quick_occ_stats_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::QuickOccupancyStats>(
      "/rolling_bonxai/quick_occupancy_stats", 10);

  const auto period =
    std::chrono::duration<double>(1.0 / server_params_.stats_publish_rate);

  stats_timer_ =
    create_wall_timer(period, std::bind(&RollingBonxaiServer::publishStats, this));
}

void RollingBonxaiServer::initChunkVisualization()
{
  chunk_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("/rolling_bonxai/chunk_markers", 1);

  transition_state_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::RollingState>("/rolling_bonxai/transition_state", 10);

  chunk_vis_timer_ =
    create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RollingBonxaiServer::publishChunkVisualization, this));
}

void RollingBonxaiServer::cleanMemoryServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!occupancy_map_->isStatsStable()) {
    response->success = false;
    response->message = "Memory cleanup skipped: map not in stats-safe state";
    return;
  }

  occupancy_map_->releaseUnusedMemory();
  occupancy_map_->shrinkToFit();

  response->success = true;
  response->message = "RollingBonxai memory cleanup completed";
}


void RollingBonxaiServer::publishCleanMap()
{
  IcpCloud points;
  std::vector<Eigen::Vector3f> colors;
  clean_map_->points(points, true, Eigen::Vector3d::Zero(), 0, clean_publish_max_points_, &colors);
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    pcl::PointXYZRGB point;
    point.x = points[i].x; point.y = points[i].y; point.z = points[i].z;
    point.r = uint8_t(std::clamp(colors[i].x(), 0.0f, 255.0f));
    point.g = uint8_t(std::clamp(colors[i].y(), 0.0f, 255.0f));
    point.b = uint8_t(std::clamp(colors[i].z(), 0.0f, 255.0f));
    cloud.push_back(point);
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = reconstruction_frame_;
  message.header.stamp = now();
  clean_pub_->publish(message);  // Publish empty maps too, so stale geometry is cleared.
  std_msgs::msg::String stats;
  stats.data = "clean_cells=" + std::to_string(clean_map_->activeCells()) +
    " clean_chunks=" + std::to_string(clean_map_->activeChunks()) +
    " clean_bytes=" + std::to_string(clean_map_->memoryBytes()) +
    " published_points=" + std::to_string(points.size());
  icp_diagnostics_pub_->publish(stats);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "Clean map: %zu points, %zu cells, %zu resident chunks, %zu saved, %zu loaded; queued=%zu",
    points.size(), clean_map_->activeCells(), clean_map_->activeChunks(),
    clean_map_->savedChunks(), clean_map_->loadedChunks(), cloud_queue_.size());
}

void RollingBonxaiServer::publishStats()
{
  const auto now_time = now();

  if (voxel_pub_) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    occupancy_map_->getOccupiedVoxels(cloud.points);

    if (!cloud.empty()) {
      cloud.width = cloud.points.size();
      cloud.height = 1;
      cloud.is_dense = true;

      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header.frame_id = icp_->config().enabled ? reconstruction_frame_ : server_params_.frame_id;
      msg.header.stamp = now_time;
      voxel_pub_->publish(msg);
    }
  }

  rolling_bonxai_msgs::msg::ChunkStats cs;
  cs.header.stamp = now_time;
  cs.header.frame_id = icp_->config().enabled ? reconstruction_frame_ : server_params_.frame_id;
  cs.active_chunk_count = occupancy_map_->getActiveChunkCount();
  cs.dirty_chunk_count = occupancy_map_->getDirtyChunkCount();
  cs.clean_chunk_count = occupancy_map_->getCleanChunkCount();
  if (clean_map_ && !clean_config_.build_occupancy) {
    const auto states = clean_map_->chunkStates();
    cs.active_chunk_count = states.size();
    cs.dirty_chunk_count = std::count_if(states.begin(), states.end(), [](const auto& entry) { return entry.second; });
    cs.clean_chunk_count = states.size() - cs.dirty_chunk_count;
  }
  chunk_stats_pub_->publish(cs);

  const auto qs = occupancy_map_->getStats(false);
  rolling_bonxai_msgs::msg::QuickOccupancyStats qs_msg;
  qs_msg.header.stamp = now_time;
  qs_msg.header.frame_id = icp_->config().enabled ? reconstruction_frame_ : server_params_.frame_id;
  qs_msg.total_memory_mib = bytesToMiB(qs.total_memory_bytes + (clean_map_ ? clean_map_->memoryBytes() : 0));
  qs_msg.grid_memory_mib = bytesToMiB(qs.grid_memory_bytes + (clean_map_ ? clean_map_->memoryBytes() : 0));
  qs_msg.buffer_memory_mib = bytesToMiB(qs.buffer_memory_bytes);
  qs_msg.total_active_cells = qs.total_active_cells + (clean_map_ ? clean_map_->activeCells() : 0);
  qs_msg.total_insertions = qs.total_insertions;
  qs_msg.hit_points_added = qs.hit_points_added;
  qs_msg.miss_points_added = qs.miss_points_added;
  qs_msg.has_data = qs.has_data || (clean_map_ && clean_map_->activeCells() > 0);
  quick_occ_stats_pub_->publish(qs_msg);
}

void RollingBonxaiServer::publishChunkVisualization()
{
  const auto now_time = now();

  const auto states = clean_map_ && !clean_config_.build_occupancy ?
    clean_map_->chunkStates() : occupancy_map_->getChunkStates();
  const auto pending = occupancy_map_->getPendingChunks();

  visualization_msgs::msg::MarkerArray arr;
  // IDs are reassigned each snapshot; remove old IDs when the resident set shrinks.
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = icp_->config().enabled ? reconstruction_frame_ : server_params_.frame_id;
  clear.header.stamp = now_time;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);
  const double size = rolling_params_.chunk_size;
  int id = 0;

  for (const auto& [pos, dirty] : states) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = icp_->config().enabled ? reconstruction_frame_ : server_params_.frame_id;
    m.header.stamp = now_time;
    m.ns = "chunks";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = pos.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = size;
    m.scale.y = size;
    m.scale.z = size;

    if (dirty) {
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
    } else {
      m.color.r = 0.0f;
      m.color.g = 0.0f;
      m.color.b = 1.0f;
    }
    m.color.a = 0.15f;
    arr.markers.push_back(m);
  }

  for (const auto& pos : pending) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = icp_->config().enabled ? reconstruction_frame_ : server_params_.frame_id;
    m.header.stamp = now_time;
    m.ns = "chunks";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = pos.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = size;
    m.scale.y = size;
    m.scale.z = size;
    m.color.r = 0.5f;
    m.color.g = 0.5f;
    m.color.b = 0.5f;
    m.color.a = 0.15f;
    arr.markers.push_back(m);
  }

  chunk_marker_pub_->publish(arr);

  rolling_bonxai_msgs::msg::RollingState ts;
  ts.header.stamp = now_time;
  ts.header.frame_id = icp_->config().enabled ? reconstruction_frame_ : server_params_.frame_id;
  ts.state = clean_map_ && !clean_config_.build_occupancy ? "CLEAN_SURFACE_WINDOW" : occupancy_map_->getTransitionState();
  transition_state_pub_->publish(ts);
}

void RollingBonxaiServer::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (cloud_queue_.size() >= 60) {
    cloud_queue_.pop_front();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cloud queue overflow; dropped oldest scan");
  }
  cloud_queue_.push_back({msg, std::chrono::steady_clock::now()});
}

void RollingBonxaiServer::processCloudQueue()
{
  // Bounded work per timer callback; all map access uses the default mutually-exclusive group.
  if (cloud_queue_.empty()) return;
  const auto queued = cloud_queue_.front();
  const auto query = rclcpp::Time(queued.cloud->header.stamp) +
    rclcpp::Duration::from_seconds(pose_time_offset_);
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(server_params_.frame_id,
      queued.cloud->header.frame_id, query, rclcpp::Duration::from_seconds(0));
  } catch (const tf2::TransformException& ex) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - queued.received).count() > max_tf_wait_) {
      cloud_queue_.pop_front();
      RCLCPP_WARN(get_logger(), "Dropped cloud: %s -> %s at %.6f: %s",
        server_params_.frame_id.c_str(), queued.cloud->header.frame_id.c_str(), query.seconds(), ex.what());
    }
    return;
  }
  cloud_queue_.pop_front();
  processCloud(queued.cloud, tf);
}

void RollingBonxaiServer::publishCorrection(const rclcpp::Time& stamp)
{
  const auto& correction = icp_->correction();
  const Eigen::Quaterniond q(correction.linear());
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = reconstruction_frame_;
  message.child_frame_id = server_params_.frame_id;
  message.transform.translation.x = correction.translation().x();
  message.transform.translation.y = correction.translation().y();
  message.transform.translation.z = correction.translation().z();
  message.transform.rotation.x = q.x(); message.transform.rotation.y = q.y();
  message.transform.rotation.z = q.z(); message.transform.rotation.w = q.w();
  correction_broadcaster_->sendTransform(message);
}

void RollingBonxaiServer::processCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
  const geometry_msgs::msg::TransformStamped& tf)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  const bool has_color = std::any_of(msg->fields.begin(), msg->fields.end(),
    [](const auto& field) { return field.name == "rgb" || field.name == "rgba"; });
  if (has_color) pcl::fromROSMsg(*msg, *cloud);
  else {
    IcpCloud xyz;
    pcl::fromROSMsg(*msg, xyz);
    pcl::copyPointCloud(xyz, *cloud);
    for (auto& point : *cloud) point.r = point.g = point.b = 255;
  }
  if (cloud->empty()) return;

  Eigen::Vector3d origin(
    tf.transform.translation.x,
    tf.transform.translation.y,
    tf.transform.translation.z);

  Eigen::Vector3d linvel(0.0,0.0,0.0);

  Eigen::Quaterniond q(
    tf.transform.rotation.w,
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z);

  if (icp_->config().enabled) {
    Eigen::Isometry3d prior = Eigen::Isometry3d::Identity();
    prior.linear() = q.toRotationMatrix();
    prior.translation() = origin;
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    if (!icp_->isKeyframe(prior, stamp)) return;
    const auto predicted = icp_->correction() * prior;
    IcpCloud target, scan;
    CoherentScan coherent;
    std::vector<Eigen::Vector3f> coherent_colors;
    if (clean_map_) {
      clean_map_->prepare(predicted.translation());
      clean_map_->points(target, false, predicted.translation(), icp_->config().target_radius,
        icp_->config().max_target_points);
      pcl::copyPointCloud(*cloud, scan);
      coherent = coherence_filter_->process(scan);
    } else {
      occupancy_map_->prepareForPose(predicted.translation());
      std::vector<Eigen::Vector3d> local;
      occupancy_map_->getLocalOccupiedPoints(predicted.translation(), icp_->config().target_radius,
        icp_->config().max_target_points, local);
      for (const auto& p : local) target.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
    // Match the offline input policy while retaining RGB on the integrated cloud.
    auto valid = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (clean_map_) {
      for (size_t index : coherent.input_indices) {
        const auto& p = (*cloud)[index]; valid->push_back(p);
        coherent_colors.emplace_back(p.r, p.g, p.b);
      }
    } else for (const auto& point : *cloud) {
      const double range = point.getVector3fMap().norm();
      if (point.getVector3fMap().allFinite() && range >= icp_->config().min_range &&
          range < icp_->config().max_range) valid->push_back(point);
    }
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    const double voxel = icp_->config().voxels.back();
    grid.setLeafSize(voxel, voxel, voxel);
    grid.setInputCloud(valid);
    grid.filter(*cloud);
    pcl::copyPointCloud(*cloud, scan);
    auto result = icp_->process(scan, target, prior, stamp,
      clean_map_ || occupancy_map_->getPendingChunks().empty());
    std_msgs::msg::String diagnostic;
    std::ostringstream description;
    description << "reason=" << result.reason << " fitness=" << result.fitness
      << " rmse=" << result.rmse << " eigen_ratio=" << result.eigen_ratio
      << " correspondences=" << result.correspondences << " icp_ms=" << result.milliseconds;
    diagnostic.data = description.str();
    icp_diagnostics_pub_->publish(diagnostic);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "ICP %s: scan=%zu target=%zu integrate=%s fitness=%.3f rmse=%.3f",
      result.reason.c_str(), scan.size(), target.size(), result.integrate ? "true" : "false",
      result.fitness, result.rmse);
    publishCorrection(rclcpp::Time(msg->header.stamp));
    origin = result.pose.translation();
    q = Eigen::Quaterniond(result.pose.linear());
    geometry_msgs::msg::PoseStamped corrected;
    corrected.header.stamp = msg->header.stamp;
    corrected.header.frame_id = reconstruction_frame_;
    corrected.pose.position.x = origin.x(); corrected.pose.position.y = origin.y();
    corrected.pose.position.z = origin.z();
    // Corrected pose orientation includes vehicle attitude and sensor extrinsics.
    corrected.pose.orientation.x = q.x(); corrected.pose.orientation.y = q.y();
    corrected.pose.orientation.z = q.z(); corrected.pose.orientation.w = q.w();
    corrected_pose_pub_->publish(corrected);
    if (!result.integrate) return;
    if (clean_map_) {
      clean_map_->update(coherent, result.pose, coherent_colors);
      if (!clean_config_.build_occupancy) return;
    }
  }

  std::vector<pcl::PointXYZRGB> pts;
  pts.reserve(cloud->size());

  for (const auto& p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;

    Eigen::Vector3d ps(p.x, p.y, p.z);
    if (icp_->config().enabled && (ps.norm() < icp_->config().min_range ||
        ps.norm() >= icp_->config().max_range)) continue;
    Eigen::Vector3d pm = q * ps + origin;

    if (pm.z() < rolling_params_.map_params.occupancy_min_z ||
        pm.z() > rolling_params_.map_params.occupancy_max_z)
      continue;

    pcl::PointXYZRGB pt = p;
    pt.x = pm.x();
    pt.y = pm.y();
    pt.z = pm.z();
    pts.push_back(pt);
  }

  if (!pts.empty()) {
    occupancy_map_->prepareForPose(origin, linvel);
    if (icp_->config().enabled && !occupancy_map_->getPendingChunks().empty()) {
      std_msgs::msg::String diagnostic;
      diagnostic.data = "reason=chunks_pending_after_correction integrated=false";
      icp_diagnostics_pub_->publish(diagnostic);
      return;
    }
    occupancy_map_->insertPointCloud(
      pts, origin, linvel,
      rolling_params_.map_params.sensor_max_range);
  }
}

} // namespace RollingBonxai


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RollingBonxai::RollingBonxaiServer)
