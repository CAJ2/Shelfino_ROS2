
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

using std::placeholders::_1;
using std::placeholders::_2;

class RandomPoints : public rclcpp::Node
{
  public:
    RandomPoints()
    : Node("random_points")
    {
		this->roadmap_service = this->create_service<planning_msgs::srv::GenRoadmap>("random_points",
			std::bind(&RandomPoints::generate, this, _1, _2));
	}

  private:
	rclcpp::Service<planning_msgs::srv::GenRoadmap>::SharedPtr roadmap_service;

    void generate(const std::shared_ptr<planning_msgs::srv::GenRoadmap::Request> request,
		std::shared_ptr<planning_msgs::srv::GenRoadmap::Response> response) {

		int POINTS = 250; //usually not reached cause it is saturated by points
		int DX = 10;
		int DY = 10;
		float radius_of_distance = 0.5; //to regulate the density of the points
		std::string map_name = "hexagon";


		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<> x_dis(-DX, DX);
		std::uniform_real_distribution<> y_dis(-DY, DY);

		//Debugging, for seeing if all of them are caught
		for (int i=0; i<(int)request->obstacles.obstacles.size(); i++){
			RCLCPP_INFO(this->get_logger(), "Obs n. %i", i);
		}

		for (int i=0; i<(int)request->gate.obstacles.size(); i++){
			RCLCPP_INFO(this->get_logger(), "Gate n. %i", i);
		}

		for (int i=0; i<(int)request->victims.obstacles.size(); i++){
			RCLCPP_INFO(this->get_logger(), "Victim n. %i", i);
		}


		std::vector<obstacle> possible_waypoints;

		//First add the victims we already have

		//!!The information about their value is contained in the radius of the "victims" vector,
		//not in the "possible_waypoints", which just contains their position and distance between them
		for (auto v : request->victims.obstacles) {
			victim vict = victim(v.x, v.y);
			vict.radius = radius_of_distance;
			possible_waypoints.push_back(vict);
		}

		std::vector<obstacle> obstacles = msg_to_obstacles(request->obstacles);
		std::vector<obstacle> gates = msg_to_obstacles(request->gate);

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
				if (valid_position(map_name, DX, DY, new_element, {possible_waypoints, obstacles, gates})) {
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

		// Set the service response message
		for (auto o : possible_waypoints) {
			geometry_msgs::msg::Point point;

			point.x = o.x;
			point.y = o.y;
			point.z = 0.0;
			response->roadmap.nodes.push_back(point);
		}
		RCLCPP_INFO(this->get_logger(), "\n\n !!!!! Points spawned !!!!!!! \n\n");
    }

	std::vector<obstacle> msg_to_obstacles(obstacles_msgs::msg::ObstacleArrayMsg msg) {
		std::vector<obstacle> obs;
		for (auto m : msg.obstacles) {
			obstacle_type ty;
			if (m.type == "CYLINDER") {
				ty = CYLINDER;
			} else if (m.type == "BOX") {
				ty = BOX;
			}
			obstacle o = {m.radius, m.x, m.y, m.dx, m.dy, ty};
			obs.push_back(o);
		}
		return obs;
	}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomPoints>());
  rclcpp::shutdown();
  return 0;
}
