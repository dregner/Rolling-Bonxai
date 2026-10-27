#pragma once

#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <string>

namespace RollingBonxai
{
// Some recorded bags contain both a placeholder static body pose and real
// dynamic poses. Never let that placeholder change the body's TF cache type.
class BodyTfListener : public tf2_ros::TransformListener
{
public:
  BodyTfListener(tf2::BufferCore& buffer, rclcpp::Node* node,
                 std::string world, std::string body)
  : tf2_ros::TransformListener(buffer, node, false),
    world_(std::move(world)), body_(std::move(body)), logger_(node->get_logger()) {}

  void subscription_callback(tf2_msgs::msg::TFMessage::ConstSharedPtr message,
                             bool is_static) override
  {
    if (!is_static) {
      tf2_ros::TransformListener::subscription_callback(message, false);
      return;
    }
    auto filtered = std::make_shared<tf2_msgs::msg::TFMessage>(*message);
    auto& transforms = filtered->transforms;
    transforms.erase(std::remove_if(transforms.begin(), transforms.end(),
      [this](const auto& tf) {
        return tf.header.frame_id == world_ && tf.child_frame_id == body_;
      }), transforms.end());
    if (transforms.size() != message->transforms.size()) {
      RCLCPP_WARN_ONCE(logger_, "Ignoring static %s -> %s; timestamped dynamic body poses are required",
        world_.c_str(), body_.c_str());
    }
    tf2_ros::TransformListener::subscription_callback(filtered, true);
  }

private:
  std::string world_, body_;
  rclcpp::Logger logger_;
};
}  // namespace RollingBonxai
