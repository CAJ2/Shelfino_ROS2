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

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "planning_msgs/srv/gen_roadmap.hpp"
#include "planning_msgs/msg/roadmap_info.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

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


class RoadmapHarness : public rclcpp::Node
{
public:
    explicit RoadmapHarness()
    : Node("roadmap_harness")
    {
        RCLCPP_INFO(this->get_logger(), "\n\n -------- NODE ACTIVATED ---------------");

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom2);
        gate_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "/gate_position", qos, std::bind(&RoadmapHarness::gate_callback, this, _1));
        obstacles_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
                "/obstacles", qos, std::bind(&RoadmapHarness::obstacles_callback, this, _1));
        victims_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
                "/victims", qos, std::bind(&RoadmapHarness::victims_callback, this, _1));
    
        publisher_rviz = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers/roadmap", qos);
        publisher_roadmap = this->create_publisher<planning_msgs::msg::RoadmapInfo>("/roadmap", qos);

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter("roadmap_services", std::vector<std::string>());
        this->declare_parameter("publish_roadmap_service", "");
    }

    void activate() {
        auto services = this->get_parameter("roadmap_services").as_string_array();
        std::string publish_service = this->get_parameter("publish_roadmap_service").as_string();

        auto request = std::make_shared<planning_msgs::srv::GenRoadmap::Request>();
        request->gate.obstacles = this->gates;
        request->obstacles.obstacles = this->obstacles;
        request->victims.obstacles = this->victims;

        this->srv_clients.erase(this->srv_clients.begin(), this->srv_clients.end());
        for (auto s : services) {
            auto client = this->create_client<planning_msgs::srv::GenRoadmap>(s);
            this->srv_clients.push_back(client);
            if (!client->wait_for_service(2s)) {
                RCLCPP_ERROR(this->get_logger(), "Service %s is not available, skipping", s.c_str());
            }

            auto start_time = std::chrono::system_clock::now();

            auto result_cb = [this, s, start_time](rclcpp::Client<planning_msgs::srv::GenRoadmap>::SharedFuture result) {
                auto service_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time);
                auto response = result.get();
                RCLCPP_INFO(this->get_logger(), "Service %s (took %ld ms) returned roadmap with %ld nodes",
                    s.c_str(), service_duration.count(), response->roadmap.nodes.size());

                geometry_msgs::msg::TransformStamped t;
                try {
                    t = tf_buffer_->lookupTransform("shelfino0/base_link", "map", tf2::TimePointZero);
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_INFO(this->get_logger(), "Could not transform map to shelfino0/base_link: %s", ex.what());
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "--------------------Hallooooooo");

                planning_msgs::msg::RoadmapInfo info;
                info.header.stamp = this->now();
                info.header.frame_id = "map";
                info.roadmap = response->roadmap;
                info.generator = s;
                info.roadmap_duration = service_duration.count();
                info.robot_pose.header.stamp = t.header.stamp;
                info.robot_pose.pose.pose.position.x = t.transform.translation.x;
                info.robot_pose.pose.pose.position.y = t.transform.translation.y;
                info.robot_pose.pose.pose.orientation = t.transform.rotation;
                info.gate.position.x = this->gates[0].x;
                info.gate.position.y = this->gates[0].y;
                info.obstacles.obstacles = this->obstacles;
                info.victims.obstacles = this->victims;
                this->publisher_roadmap->publish(info);

                visualization_msgs::msg::MarkerArray marks;
                for (size_t i = 0; i < response->roadmap.nodes.size(); i++) {
                    auto node = response->roadmap.nodes[i];
                    visualization_msgs::msg::Marker mark = add_point(node.x , node.y, s, i);
                    marks.markers.push_back(mark);
                }
                this->publisher_rviz->publish(marks);
            };

            auto result = client->async_send_request(request, result_cb);
        }
    }

private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_rviz;
    rclcpp::Publisher<planning_msgs::msg::RoadmapInfo>::SharedPtr publisher_roadmap;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_subscription_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscription_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victims_subscription_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::vector<std::shared_ptr<rclcpp::Client<planning_msgs::srv::GenRoadmap>>> srv_clients;

    std::vector<obstacles_msgs::msg::ObstacleMsg> gates;
    std::vector<obstacles_msgs::msg::ObstacleMsg> obstacles;
    std::vector<obstacles_msgs::msg::ObstacleMsg> victims;

    bool gate_ready = false;
    bool obstacles_ready = false;
    bool victims_ready = false;

    void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {

        //Printing out infos about obstacles
        RCLCPP_INFO(this->get_logger(), "\n\n ######### Obstacles###########\n\n");
        RCLCPP_INFO(this->get_logger(), "\nNumber of obstacles: %zu \n", msg->obstacles.size());
        for (const auto& obstacle : msg->obstacles) {
            if (obstacle.radius != 0.0){
                RCLCPP_INFO(this->get_logger(), "The object is a Cylinder with:");
                RCLCPP_INFO(this->get_logger(), "center in (%f,%f) and radius %f", obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius);
            }
            else{
                RCLCPP_INFO(this->get_logger(), "The object is a Box with vertices:");
                for (const auto& point : obstacle.polygon.points) {
                    RCLCPP_INFO(this->get_logger(), "Polygon Point - x: %f, y: %f, z: %f", point.x, point.y, point.z);
                }
            }
        }

        //Adding them to the list of obstacles
        for (auto obs : msg->obstacles) {
            if (obs.polygon.points.size() == 1) {
                obs.x = obs.polygon.points[0].x;
                obs.y = obs.polygon.points[0].y;
                obs.type = "CYLINDER";
                this->obstacles.push_back(obs);
            } else if (obs.polygon.points.size() == 4) {
                double max_x = 0.0, max_y = 0.0, min_x = 1000000.0, min_y = 1000000.0;
                for (auto point : obs.polygon.points) {
                    if (point.x > max_x) max_x = point.x;
                    if (point.x < min_x) min_x = point.x;
                    if (point.y > max_y) max_y = point.y;
                    if (point.y < min_y) min_y = point.y;
                }
                double xc = (max_x+min_x)/2.0;
                double yc = (max_y+min_y)/2.0;
                double dx = max_x-min_x;
                double dy = max_y-min_y;

                obs.x = xc;
                obs.y = yc;
                obs.dx = dx;
                obs.dy = dy;
                obs.type = "BOX";
                this->obstacles.push_back(obs);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Obstacle with %ld points not supported.", obs.polygon.points.size());
            }
        }

        this->obstacles_ready = true;
        this->activate_wrapper();

    }

    void victims_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "\n\n ######### Victims ###########\n\n");

        RCLCPP_INFO(this->get_logger(), "\nNumber of victims: %zu \n", msg->obstacles.size());
        for (const auto& obstacle : msg->obstacles) {

            RCLCPP_INFO(this->get_logger(), "Victim with:");
            RCLCPP_INFO(this->get_logger(), "center in (%f,%f) and value of %f", obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius);

        }

        //Adding them to the list of victims
        for (auto obs : msg->obstacles) {

            obs.x = obs.polygon.points[0].x;
            obs.y = obs.polygon.points[0].y;
            obs.type = "CYLINDER";
            this->victims.push_back(obs);

        }
        this->victims_ready = true;
        this->activate_wrapper();
    }

    void gate_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "\n\n ######### Gate ###########\n\n");


        RCLCPP_INFO(this->get_logger(), "Gate with:");
        RCLCPP_INFO(this->get_logger(), "center in (%f,%f)", msg->poses[0].position.x, msg->poses[0].position.x);

        //This is a loop but there should be only one
        for (auto pose : msg->poses) {
            obstacles_msgs::msg::ObstacleMsg obs;
            obs.x = pose.position.x;
            obs.y = pose.position.y;
            obs.dx = 1.0;
            obs.dy = 1.0;
            obs.type = "BOX";
            this->gates.push_back(obs);
        }

        this->gate_ready = true;
        this->activate_wrapper();

    }

    visualization_msgs::msg::Marker add_point(float x, float y, std::string service, int id) {
        // Publish markers, just for rviz
        std_msgs::msg::Header hh;
        hh.stamp = this->get_clock()->now();
        hh.frame_id = "map";

        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;

        visualization_msgs::msg::Marker mark;

        mark.header = hh;
        mark.ns = service;
        mark.id = id;
        mark.action = visualization_msgs::msg::Marker::ADD;
        mark.type = visualization_msgs::msg::Marker::CYLINDER;
        mark.pose = pose;
        mark.scale.x = 0.3;
        mark.scale.y = 0.3;
        mark.scale.z = 0.01;
        mark.color.a = 1.0;
        mark.color.r = 1.0;
        mark.color.g = 0.0;
        mark.color.b = 1.0;

        return mark;
    }

    void activate_wrapper() {
        if (this->gate_ready && this->obstacles_ready && this->victims_ready) {
            this->activate();
        } else {
            RCLCPP_INFO(this->get_logger(), "Waiting for infos to be ready");
        }

    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapHarness>());
  rclcpp::shutdown();
  return 0;
}
