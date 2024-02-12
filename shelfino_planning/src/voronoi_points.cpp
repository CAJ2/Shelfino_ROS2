
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <random>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "utilities.hpp"
#include "voronoi_points.hpp"

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
#include "planning_msgs/msg/point2_d.hpp"

void VoronoiPoints::voronoiEdgeGeneration(const std::vector<obstacle> &obstacles)
{
    jcv_point_* points;

    int numOfPoints = 6; //number of boarder points

    for(auto& obstacle : obstacles)
    {
        if(obstacle.type == obstacle_type::CYLINDER)
        {
            numOfPoints++;
        } 
        else if(obstacle.type == obstacle_type::BOX) 
        {
            numOfPoints += 4;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Obstacle type not supported");
        }
    }

    points = (jcv_point_*)malloc(sizeof(jcv_point_) * numOfPoints);

    float maxX, maxY, minX, minY;
    maxX = maxY = -1000000;
    minX = minY = 1000000;
    uint16_t i = 0;
    for(auto point : borders_.points)
    {
        if(point.x > maxX)
            maxX = point.x;
        if(point.x < minX)
            minX = point.x;
        if(point.y > maxY)
            maxY = point.y;
        if(point.y < minY)
            minY = point.y;
        points[i].x = point.x;
        points[i].y = point.y;
        i++;
        RCLCPP_INFO(this->get_logger(), "Border point %f, %f", point.x, point.y);
    }


    jcv_rect_ rect;
    jcv_point_ min = {minX, minY};
    jcv_point_ max = {maxX, maxY}; 
    rect.min = min;
    rect.max = max;
    for(int j = 0; j < i; j++)
    {
        RCLCPP_INFO(this->get_logger(), "Point %f, %f", points[j].x, points[j].y);
    }
    for(auto& obstacle : obstacles)
    {
        if(obstacle.type == obstacle_type::CYLINDER)
        {
            points[i].x = obstacle.x;
            points[i].y = obstacle.y;
            i++;
        }
        else if(obstacle.type == obstacle_type::BOX)
        {
            points[i].x = obstacle.x - obstacle.dx/2;
            points[i].y = obstacle.y - obstacle.dy/2;
            i++;
            points[i].x = obstacle.x + obstacle.dx/2;
            points[i].y = obstacle.y - obstacle.dy/2;
            i++;
            points[i].x = obstacle.x + obstacle.dx/2;
            points[i].y = obstacle.y + obstacle.dy/2;
            i++;
            points[i].x = obstacle.x - obstacle.dx/2;
            points[i].y = obstacle.y + obstacle.dy/2;
            i++;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Obstacle type not supported");
        }
    }

    int numOfRelaxations = 5;

    memset(&diagram, 0, sizeof(jcv_diagram_));

    for (int i = 0; i < numOfRelaxations; ++i)
    {
        jcv_diagram_generate(numOfPoints, (const jcv_point*)points, &rect, nullptr, &diagram);
        relax_points(&diagram, points);
    }
}

void VoronoiPoints::generate(const std::shared_ptr<planning_msgs::srv::GenRoadmap::Request> request,
                             std::shared_ptr<planning_msgs::srv::GenRoadmap::Response> response)
{
    std::string map_name = "hexagon";

    double DX = 9.0; // made a bit smaller than 10 so it is not too close to the edges
	double DY = 9.0;

    std::vector<obstacle> possible_waypoints;

    possible_waypoints.push_back(victim(initialPose.pose.pose.position.x, initialPose.pose.pose.position.y, 0.05));

    // First add the victims we already have

    //!!The information about their value is contained in the radius of the "victims" vector,
    // not in the "possible_waypoints", which just contains their position and distance between them
    for (auto v : request->victims.obstacles)
    {
        victim vict = victim(v.x, v.y);
        vict.radius = 0.05;
        possible_waypoints.push_back(vict);
    }

    std::vector<obstacle> obstacles = msg_to_obstacles(request->obstacles);
    std::vector<obstacle> gates = msg_to_obstacles(request->gate);
    possible_waypoints.push_back(victim(gates[0].x, gates[0].y, 0.05));

    voronoiEdgeGeneration(obstacles);

    const jcv_edge_* edge = jcv_diagram_get_edges(&diagram);

    while (edge)
    {
        RCLCPP_INFO(this->get_logger(), "Edge %f, %f", edge->pos[0].x, edge->pos[0].y);
        victim new_element{0, 0};

        new_element.x = edge->pos[0].x;
        new_element.y = edge->pos[0].y;
        new_element.radius = 0.05;

        if (valid_position(map_name, DX, DY, new_element, {possible_waypoints, obstacles, gates})) 
        {
            possible_waypoints.push_back(new_element); //for the internal list of waypoints
        }

        edge = jcv_diagram_get_next_edge(edge);
    }
    planning_msgs::msg::Roadmap roadmap = createGraphEdges(possible_waypoints, obstacles);
		
	for(auto edge : roadmap.edges){
		response->roadmap.edges.push_back(edge);
	}
	for(auto node : roadmap.nodes){
		response->roadmap.nodes.push_back(node);
	}

    RCLCPP_INFO(this->get_logger(), "\n\n !!!!! Points spawned !!!!!!! \n\n");
}

void VoronoiPoints::listenBorders(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "------------------Received borders");
    borders_ = *msg;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiPoints>());
  rclcpp::shutdown();
  return 0;
}
