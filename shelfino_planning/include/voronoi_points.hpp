
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <random>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "utilities.hpp"

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "planning_msgs/srv/gen_roadmap.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define JC_VORONOI_IMPLEMENTATION
#include "jc_voronoi.h"
#define JC_VORONOI_CLIP_IMPLEMENTATION
#include "jc_voronoi_clip.h"

using std::placeholders::_1;
using std::placeholders::_2;

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

class VoronoiPoints : public rclcpp::Node
{
  public:
    VoronoiPoints()
    : Node("voronoi_points")
    {
        this->node_namespace = this->get_namespace();

        this->roadmap_service = this->create_service<planning_msgs::srv::GenRoadmap>("voronoi_points",
          std::bind(&VoronoiPoints::generate, this, _1, _2));

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom2);
        borders_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
          "/map_borders",
          qos, 
          std::bind(&VoronoiPoints::listenBorders, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("markers/voronoi_edges", qos);

        // Create listener node of type PoseWithCovarianceStamped on /shelfino0/initialpose
        auto node = rclcpp::Node::make_shared("listen_initPose");
        auto sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/shelfino0/initialpose", 
        qos,
          [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: [%f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
            initialPose.pose.pose.position.x = msg->pose.pose.position.x;
            initialPose.pose.pose.position.y = msg->pose.pose.position.y;
          }
        );
    }

    int marker_id_ = 0;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  private:

    void voronoiEdgeGeneration(const std::vector<obstacle>& obstacles);

	  rclcpp::Service<planning_msgs::srv::GenRoadmap>::SharedPtr roadmap_service;

    std::string node_namespace;

    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr borders_sub_;

    void generate(const std::shared_ptr<planning_msgs::srv::GenRoadmap::Request> request,
		std::shared_ptr<planning_msgs::srv::GenRoadmap::Response> response);

    void listenBorders(const geometry_msgs::msg::Polygon::SharedPtr msg);

    geometry_msgs::msg::Polygon borders_;

    jcv_diagram diagram;

    geometry_msgs::msg::PoseWithCovarianceStamped initialPose;
};
