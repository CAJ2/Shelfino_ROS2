#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

class StaticRobotState : public rclcpp::Node
{
private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string node_namespace;

  double initial_x = 0.0;
  double initial_y = 0.0;
  double initial_yaw = 0.0;

public:
  StaticRobotState() : Node("static_robot_state")
  {
    this->node_namespace = this->get_namespace();

    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    this->publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&StaticRobotState::publish_tf, this));

    this->initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      this->node_namespace + "/initialpose", 10, std::bind(&StaticRobotState::initial_pose_cb, this, std::placeholders::_1));
  }

private:
  void publish_tf();
  void initial_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};

void StaticRobotState::publish_tf() {
  rclcpp::Time now = this->get_clock()->now();

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.transform.rotation.w = 1.0;

  // Zero transform from /map -> /shelfinoX/odom
  t.header.frame_id = "map";
  t.child_frame_id = node_namespace + "/odom";
  this->tf_broadcaster_->sendTransform(t);

  // Initial transform from /shelfinoX/odom -> /shelfinoX/base_link
  t.transform.translation.x = initial_x;
  t.transform.translation.y = initial_y;
  tf2::Quaternion rot;
  rot.setRPY(0.0, 0.0, initial_yaw);
  t.transform.rotation = tf2::toMsg(rot);
  t.header.frame_id = node_namespace + "/odom";
  t.child_frame_id = node_namespace + "/base_link";
  this->tf_broadcaster_->sendTransform(t);
}

void StaticRobotState::initial_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  this->initial_x = msg->pose.pose.position.x;
  this->initial_y = msg->pose.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  this->initial_yaw = q.getAngle();
  RCLCPP_INFO(this->get_logger(), "Initial pose set to x: %f, y: %f, yaw: %f", this->initial_x, this->initial_y, this->initial_yaw);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create the node
  auto node = std::make_shared<StaticRobotState>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
