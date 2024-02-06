#pragma once
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <random>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "planning_msgs/msg/graph_path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ExecutePath : public rclcpp::Node
{
public:
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    explicit ExecutePath()
    : Node("execute_path")
    {
        this->declare_parameter("execute_path_nodes", std::vector<std::string>());

        subscriber_paths = this->create_subscription<planning_msgs::msg::GraphPath>(
            "execute_path", 10, std::bind(&ExecutePath::graphPathCallback, this, _1));
        publisher_rviz_graph = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers/execute_graph", 10);
        publisher_rviz = this->create_publisher<nav_msgs::msg::Path>("markers/execute_path", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ExecutePath::runPath, this));
        publisher_action_plan = this->create_publisher<nav_msgs::msg::Path>("shelfino0/plan1", 10);
        action_follow_path = rclcpp_action::create_client<FollowPath>(this, "shelfino0/follow_path");
    }

private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_rviz_graph;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_rviz;
	rclcpp::Subscription<planning_msgs::msg::GraphPath>::SharedPtr subscriber_paths;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_action_plan;
    rclcpp_action::Client<FollowPath>::SharedPtr action_follow_path;

    bool running_path = false;
    planning_msgs::msg::GraphPath current_path;
    std::vector<nav_msgs::msg::Path> path_goals;
    int current_path_goal = 0;
    bool current_path_running = false;
    rclcpp::Time start_time;

    std::vector<planning_msgs::msg::GraphPath> graph_paths;
    std::vector<planning_msgs::msg::GraphPath> graph_paths_complete;

    void graphPathCallback(const planning_msgs::msg::GraphPath::SharedPtr msg);
    void runPath();
    void goalResponseCallback(const GoalHandleFollowPath::SharedPtr& msg);
    void feedbackCallback(const GoalHandleFollowPath::SharedPtr& msg, const std::shared_ptr<const FollowPath::Feedback> feedback);
    void resultCallback(const GoalHandleFollowPath::WrappedResult& result);

    visualization_msgs::msg::Marker add_point(float x, float y, int id);
    visualization_msgs::msg::Marker add_line(float x1, float y1, float x2, float y2, int id);
};
