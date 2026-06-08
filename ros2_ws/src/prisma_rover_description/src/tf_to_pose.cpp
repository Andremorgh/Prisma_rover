#include <memory>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TransformPosePublisher : public rclcpp::Node
{
public:
  TransformPosePublisher()
  : Node("transform_pose_publisher")
  {
    // Declare and retrieve parameters for flexibility
    this->declare_parameter<std::string>("map_frame", "rover/map");
    this->declare_parameter<std::string>("base_frame", "rover/base_link");
    this->declare_parameter<std::string>("pose_topic", "rover_tf_pose");

    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("pose_topic", pose_topic_);

    RCLCPP_INFO(this->get_logger(), "Listening to transform: %s -> %s | Publishing: %s", 
                map_frame_.c_str(), base_frame_.c_str(), pose_topic_.c_str());

    // Create a publisher for PoseStamped
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 1);

    // Create the TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer: every 100 ms lookup transform and publish
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TransformPosePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Unable to lookup transform between %s and %s: %s (warnings throttled)", 
                       map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    // Convert transform to PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = transformStamped.transform.translation.x;
    pose_msg.pose.position.y = transformStamped.transform.translation.y;
    pose_msg.pose.position.z = transformStamped.transform.translation.z;
    pose_msg.pose.orientation = transformStamped.transform.rotation;

    // Publish the pose
    pose_pub_->publish(pose_msg);
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string pose_topic_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransformPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
