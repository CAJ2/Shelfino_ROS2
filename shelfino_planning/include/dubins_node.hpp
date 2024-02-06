#include <unistd.h>
#include <iostream>
#include <fstream>
#include <random>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "utilities.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <map>
#include <set>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "planning_msgs/srv/gen_roadmap.hpp"
#include "planning_msgs/msg/roadmap_info.hpp"
#include "planning_msgs/msg/graph_path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "graph_node.hpp"
#include "graph.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

using namespace graph_search;

static const rmw_qos_profile_t rmw_qos_profile_custom2 =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

class DubinsPathGenerator : public rclcpp::Node
{
public:
    explicit DubinsPathGenerator()
    : Node("dubins_node", rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {
        RCLCPP_INFO(this->get_logger(), "\n\n -------- NODE ACTIVATED ---------------");

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom2);

        graph_path_subscription_ = this->create_subscription<planning_msgs::msg::GraphPath>(
                "/graph_path", qos, std::bind(&DubinsPathGenerator::graphPathCallback, this, _1));
        roadmap_subscription_ = this->create_subscription<planning_msgs::msg::RoadmapInfo>(
                "/roadmap", qos, std::bind(&DubinsPathGenerator::roadmapCallback, this, _1));

        marker_pub_ = this->create_publisher<nav_msgs::msg::Path>("markers/graph_search", qos);
        publisher_dubins_path_ = this->create_publisher<planning_msgs::msg::GraphPath>("execute_path", qos);
    }

private:

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr marker_pub_;
    rclcpp::Subscription<planning_msgs::msg::GraphPath>::SharedPtr graph_path_subscription_;
    rclcpp::Subscription<planning_msgs::msg::RoadmapInfo>::SharedPtr roadmap_subscription_;
    rclcpp::Publisher<planning_msgs::msg::GraphPath>::SharedPtr publisher_dubins_path_;

    std::vector<graph::Edge> obstacleEdges;
    planning_msgs::msg::RoadmapInfo roadmapInfo;
    std::vector<geometry_msgs::msg::Pose> dubinsPathPoints;

    void roadmapCallback(const planning_msgs::msg::RoadmapInfo::SharedPtr msg);
    void graphPathCallback(const planning_msgs::msg::GraphPath::SharedPtr msg);
    void visualizePath();
    void publishPath(planning_msgs::msg::GraphPath msg);
};
