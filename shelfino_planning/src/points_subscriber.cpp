
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <random>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "utilities.hpp"

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


class PointsSubscriber : public rclcpp::Node
{
  public:
    PointsSubscriber()
    : Node("points_subscriber")
    {
      RCLCPP_INFO(this->get_logger(), "\n\n -------- NODE ACTIVATED ---------------");
      
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom2);
      gate_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      		"/gate_position", qos, std::bind(&PointsSubscriber::gate_callback, this, _1));
      obstacles_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      		"/obstacles", qos, std::bind(&PointsSubscriber::obstacles_callback, this, _1));
      victims_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      		"/victims", qos, std::bind(&PointsSubscriber::victims_callback, this, _1));
      		
      publisher_rviz = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers/possible_waypoints", qos);
      publisher_available_waypoints = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("/available_waypoints", qos);
      		

    }

  private:
  
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_rviz;
    rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_available_waypoints;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_subscription_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscription_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victims_subscription_;

    
    std::vector<obstacle> gates;
    std::vector<obstacle> obstacles;
    std::vector<obstacle> victims;

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
		obstacle obs_tmp = {obs.radius, obs.polygon.points[0].x, obs.polygon.points[0].y, 0.0, 0.0, obstacle_type::CYLINDER};
		this->obstacles.push_back(obs_tmp);
	      }
	      else if (obs.polygon.points.size() == 4) {
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

		obstacle obs_tmp = {0.0, xc, yc, dx, dy, obstacle_type::BOX};
		this->obstacles.push_back(obs_tmp);
	      }
	      else {
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
	      
		obstacle obs_tmp = {obs.radius, obs.polygon.points[0].x, obs.polygon.points[0].y, 0.0, 0.0, obstacle_type::CYLINDER};
		this->victims.push_back(obs_tmp);
	      
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
	      obstacle gate = {0.0, pose.position.x, pose.position.y, 1.0, 1.0, obstacle_type::BOX};
	      this->gates.push_back(gate);
      }

      this->gate_ready = true;
      this->activate_wrapper();

    }
    

    visualization_msgs::msg::Marker add_point(float x, float y, int id){
    	// Publish markers, just for rviz
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
	mark.scale.x = 0.3;
	mark.scale.y = 0.3;
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

	int POINTS = 250; //usually not reached cause it is saturated by points
	int DX = 10;
	int DY = 10;
	float radius_of_distance = 0.5; //to regulate the density of the points
	std::string map_name = "hexagon";
	visualization_msgs::msg::MarkerArray marks;
	

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> x_dis(-DX, DX);
	std::uniform_real_distribution<> y_dis(-DY, DY);
	
	//Debugging, for seeing if all of them are caught
	for (int i=0; i<(int)obstacles.size(); i++){
		RCLCPP_INFO(this->get_logger(), "Obs n. %i", i);
	}
	
	for (int i=0; i<(int)gates.size(); i++){
		RCLCPP_INFO(this->get_logger(), "Gate n. %i", i);
	}
	
	for (int i=0; i<(int)victims.size(); i++){
		RCLCPP_INFO(this->get_logger(), "Victim n. %i", i);
	}

	
	std::vector<obstacle> possible_waypoints;
	
	//First add the victims we already have

	//!!The information about their value is contained in the radius of the "victims" vector,
	//not in the "possible_waypoints", which just contains their position and distance between them
	for (auto v : victims) {
		victim vict = victim(v.x, v.y);
		vict.radius = radius_of_distance;
		possible_waypoints.push_back(vict);
	}

	auto startTime = this->get_clock()->now();
	int trials = 0;
	for (int i=0; i<POINTS ; i++) {
		victim new_element = victim(0.0, 0.0);
		trials = 0;
		do {	
			trials++;
			new_element.x = x_dis(gen);
			new_element.y = y_dis(gen);
			new_element.radius = radius_of_distance;
			if (valid_position(map_name, DX, DY, new_element, {possible_waypoints, this->obstacles, this->gates})) {
				visualization_msgs::msg::Marker mark = add_point(new_element.x , new_element.y, i);
				marks.markers.push_back(mark); //for the visualization
				possible_waypoints.push_back(new_element); //for the internal list of waypoints
				break;

			}
		} while(!overTime(this->get_clock(), startTime, 10)); //Once it takes more than 10 seconds of trials it stops and exits
	  if (overTime(this->get_clock(), startTime, 10)) {
	    RCLCPP_ERROR(this->get_logger(), "Map already filled at node n. %i after trials %i" , i, trials);
	  }else{
	    RCLCPP_INFO(this->get_logger(), "Generated node n. %i after trials %i" , i, trials);
	  }
	}
	
	//publishing for rviz
	this->publisher_rviz->publish(marks);
	
	
	//Publish in the topic 
	
	obstacles_msgs::msg::ObstacleArrayMsg msg;
	uint counter = 0;
	for (auto o : possible_waypoints) {
	
		obstacles_msgs::msg::ObstacleMsg obs;
    		geometry_msgs::msg::Polygon pol;
		geometry_msgs::msg::Point32 point;
		
		if (counter<victims.size()){
			RCLCPP_INFO(this->get_logger(), "victim with (%f,%f) and value: %f" , o.x, o.y, victims[counter].radius);
			obs.radius = victims[counter].radius;
		}else{
			RCLCPP_INFO(this->get_logger(), "with (%f,%f)" , o.x, o.y);
			obs.radius = 0;
		}
		
		point.x = o.x;
		point.y = o.y;
		point.z = 0.0;
		pol.points.push_back(point);
		obs.polygon = pol; 
		//(all of this overkilling, but at least it is a list... didn't want to create something new)
	
		msg.obstacles.push_back(obs);
		counter++;
	}
	this->publisher_available_waypoints->publish(msg);
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
