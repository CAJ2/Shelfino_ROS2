
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
#include "planning_msgs/action/gen_roadmap.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class RandomPoints : public rclcpp::Node
{
  public:
	using GenRoadmap = planning_msgs::action::GenRoadmap;
	using GoalHandleGenRoadmap = rclcpp_action::ServerGoalHandle<GenRoadmap>;

    RandomPoints()
    : Node("random_points")
    {
		this->roadmap_service = this->create_service<planning_msgs::srv::GenRoadmap>("random_points",
			std::bind(&RandomPoints::generate, this, _1, _2));

		this->action_server_ = rclcpp_action::create_server<GenRoadmap>(
			this,
			"random_points",
			std::bind(&RandomPoints::handle_goal, this, _1, _2),
			std::bind(&RandomPoints::handle_cancel, this, _1),
			std::bind(&RandomPoints::handle_accepted, this, _1));
	}

  private:
	rclcpp::Service<planning_msgs::srv::GenRoadmap>::SharedPtr roadmap_service;

	rclcpp_action::Server<planning_msgs::action::GenRoadmap>::SharedPtr action_server_;

	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID & uuid,
		std::shared_ptr<const GenRoadmap::Goal> goal)
	{
		RCLCPP_INFO(this->get_logger(), "Received goal request with %ld obstacles", goal->obstacles.obstacles.size());
		(void)uuid;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleGenRoadmap> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
		(void)goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted(const std::shared_ptr<GoalHandleGenRoadmap> goal_handle)
	{
		using namespace std::placeholders;
		// this needs to return quickly to avoid blocking the executor, so spin up a new thread
		std::thread{std::bind(&RandomPoints::execute, this, _1), goal_handle}.detach();
	}

	void execute(const std::shared_ptr<GoalHandleGenRoadmap> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Executing goal");
		rclcpp::Rate loop_rate(1);
		const auto goal = goal_handle->get_goal();
		auto feedback = std::make_shared<GenRoadmap::Feedback>();
		auto result = std::make_shared<GenRoadmap::Result>();

		// Check if there is a cancel request
		if (goal_handle->is_canceling()) {
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Goal canceled");
			return;
		}
		// Publish feedback
		goal_handle->publish_feedback(feedback);
		RCLCPP_INFO(this->get_logger(), "Publish feedback");

		loop_rate.sleep();

		// Check if goal is done
		if (rclcpp::ok()) {
			goal_handle->succeed(result);
			RCLCPP_INFO(this->get_logger(), "Goal succeeded");
		}
	}


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
