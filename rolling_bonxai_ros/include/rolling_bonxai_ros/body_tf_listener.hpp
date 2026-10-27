#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>
#include <string>
#include <utility>

namespace RollingBonxai
{
// Some recorded bags contain both a placeholder static body pose and real
// dynamic poses. Never let that placeholder change the body's TF cache type.
// Own the subscriptions instead of overriding TransformListener internals:
// its subscription callback is private and non-virtual in ROS 2 Humble.
class BodyTfListener
{
public:
  BodyTfListener(tf2::BufferCore& buffer, rclcpp::Node* node,
                 std::string world, std::string body)
  : buffer_(buffer), world_(std::move(world)), body_(std::move(body)), logger_(node->get_logger())
  {
    // Use the node's mutually-exclusive callback group, as cloud processing
    // performs nonblocking TF lookups. No private listener thread is needed.
    dynamic_sub_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf2_ros::DynamicListenerQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        subscription_callback(message, false);
      });
    static_sub_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf2_ros::StaticListenerQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        subscription_callback(message, true);
      });
  }

  void subscription_callback(tf2_msgs::msg::TFMessage::ConstSharedPtr message,
                             bool is_static)
  {
    for (const auto& transform : message->transforms) {
      if (is_static && transform.header.frame_id == world_ && transform.child_frame_id == body_) {
        RCLCPP_WARN_ONCE(logger_, "Ignoring static %s -> %s; timestamped dynamic body poses are required",
          world_.c_str(), body_.c_str());
        continue;
      }
      try {
        buffer_.setTransform(transform, "body_tf_listener", is_static);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(logger_, "Cannot insert TF %s -> %s: %s",
          transform.header.frame_id.c_str(), transform.child_frame_id.c_str(), ex.what());
      }
    }
  }

private:
  tf2::BufferCore& buffer_;
  std::string world_, body_;
  rclcpp::Logger logger_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_sub_, static_sub_;
};
}  // namespace RollingBonxai
