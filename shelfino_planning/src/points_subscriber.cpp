
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <random>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
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

using std::placeholders::_1;

static const rmw_qos_profile_t rmw_qos_profile_custom =
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


class PointsSubscriber : public rclcpp::Node
{
  public:
    PointsSubscriber()
    : Node("points_subscriber")
    {
      RCLCPP_INFO(this->get_logger(), "\n\n -------- NODE ACTIVATED ---------------");
      
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
      gate_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      		"/gate_position", qos, std::bind(&PointsSubscriber::gate_callback, this, _1));
      obstacles_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      		"/obstacles", qos, std::bind(&PointsSubscriber::obstacles_callback, this, _1));
      victims_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      		"/victims", qos, std::bind(&PointsSubscriber::victims_callback, this, _1));
      //borders_subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      //		"/map_borders", qos, std::bind(&PointsSubscriber::borders_callback, this, _1));
      		
      publisherm_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers/possible_waypoints", qos);
      		

    }

  private:
  
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisherm_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_subscription_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscription_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victims_subscription_;
    //rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr borders_subscription_;
    
    //std::vector<obstacle> gates;
    //std::vector<obstacle> obstacles;

    bool gate_ready = false;
    bool obstacles_ready = false;
    bool victims_ready = false;
    
    void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
       RCLCPP_INFO(this->get_logger(), "\n\n ######### Obstacles###########\n\n");
  
      RCLCPP_INFO(this->get_logger(), "\nNumber of obstacles: %zu \n", msg->obstacles.size());
      for (const auto& obstacle : msg->obstacles) {
        // Accessing information about each obstacle
       
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
      this->victims_ready = true;
      this->activate_wrapper();
    }
      
    void gate_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "\n\n ######### Gate ###########\n\n");
  

      RCLCPP_INFO(this->get_logger(), "Gate with:");
      RCLCPP_INFO(this->get_logger(), "center in (%f,%f)", msg->poses[0].position.x, msg->poses[0].position.x);

      this->gate_ready = true;
      this->activate_wrapper();

    }
    
    /*
    void borders_callback(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "\n\n ######### Borders ###########\n\n");
  
      for (const auto& vert : msg->points) {
            RCLCPP_INFO(this->get_logger(), "vertece in (%f,%f)", vert.x, vert.y);
      }
      
      
      this->activate();
      
      
    }
    */
    
    visualization_msgs::msg::Marker add_point(float x, float y, int id){
    	// Publish markers
	std_msgs::msg::Header hh;
	hh.stamp = this->get_clock()->now();
	hh.frame_id = "map";

	geometry_msgs::msg::Pose pose;
	pose.position.x = x;
	pose.position.y = y;

    	
    	visualization_msgs::msg::Marker mark;
	
	mark.header = hh;
	mark.ns = "victims";
	mark.id = id;
	mark.action = visualization_msgs::msg::Marker::ADD;
	mark.type = visualization_msgs::msg::Marker::CYLINDER;
	mark.pose = pose;
	mark.scale.x = 0.5;
	mark.scale.y = 0.5;
	mark.scale.z = 0.01;
	mark.color.a = 1.0;
	mark.color.r = 1.0;
	mark.color.g = 0.0;
	mark.color.b = 1.0;
	
    	return mark;
    }
    
    void activate_wrapper(){
    	if (this->gate_ready && this->obstacles_ready && this->victims_ready){
    		this->activate();
    	}else{
    		RCLCPP_INFO(this->get_logger(), "Waiting for infos to be ready");
    	}
    
    }
    
    void activate(){


	visualization_msgs::msg::MarkerArray marks;
	
	
	visualization_msgs::msg::Marker mark1 = add_point(1, 1, 1);
	marks.markers.push_back(mark1);
	visualization_msgs::msg::Marker mark2 = add_point(2, 2, 2);
	marks.markers.push_back(mark2);
	
	
	this->publisherm_->publish(marks);

    	RCLCPP_INFO(this->get_logger(), "\n\n !!!!! Points spawned !!!!!!! \n\n");
    }
    
    

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointsSubscriber>());
  rclcpp::shutdown();
  return 0;
}
