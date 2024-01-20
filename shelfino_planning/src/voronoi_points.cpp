
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

    // Debugging, for seeing if all of them are caught
    for (int i = 0; i < (int)request->obstacles.obstacles.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Obs n. %i", i);
    }

    for (int i = 0; i < (int)request->gate.obstacles.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Gate n. %i", i);
    }

    for (int i = 0; i < (int)request->victims.obstacles.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Victim n. %i", i);
    }

    std::vector<obstacle> possible_waypoints;
    std::vector<double> pointsForDelauney;

    // First add the victims we already have

    //!!The information about their value is contained in the radius of the "victims" vector,
    // not in the "possible_waypoints", which just contains their position and distance between them
    for (auto v : request->victims.obstacles)
    {
        victim vict = victim(v.x, v.y);
        vict.radius = 0.25;
        possible_waypoints.push_back(vict);
        pointsForDelauney.push_back(vict.x);
        pointsForDelauney.push_back(vict.y);
    }

    std::vector<obstacle> obstacles = msg_to_obstacles(request->obstacles);
    std::vector<obstacle> gates = msg_to_obstacles(request->gate);
    possible_waypoints.push_back(victim(gates[0].x, gates[0].y, 0.25));
    pointsForDelauney.push_back(gates[0].x);    
    pointsForDelauney.push_back(gates[0].y);

    voronoiEdgeGeneration(obstacles);

    const jcv_edge_* edge = jcv_diagram_get_edges(&diagram);

    // Publish markers, just for rviz
    std_msgs::msg::Header hh;
    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";
    visualization_msgs::msg::Marker line_list;
    line_list.header = hh;
    line_list.ns = "voronoi_edges";
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.id = marker_id_++; // Make sure marker_id_ is initialized to 0
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.scale.x = 0.05; // Width of the lines

    // Line color and alpha
    line_list.color.r = 1.0;
    line_list.color.g = 0.0;
    line_list.color.b = 0.0;
    line_list.color.a = 1.0;

    while (edge)
    {
        RCLCPP_INFO(this->get_logger(), "Edge %f, %f", edge->pos[0].x, edge->pos[0].y);
        victim new_element{0, 0};

        new_element.x = edge->pos[0].x;
        new_element.y = edge->pos[0].y;
        new_element.radius = 0.25;

        if (valid_position(map_name, 10, 10, new_element, {{}, obstacles, gates})) 
        {
            possible_waypoints.push_back(new_element); //for the internal list of waypoints
            pointsForDelauney.push_back(new_element.x);
            pointsForDelauney.push_back(new_element.y);

            victim new_element2{0, 0};
            new_element2.x = edge->pos[1].x;
            new_element2.y = edge->pos[1].y;
            new_element2.radius = 0.1;

            geometry_msgs::msg::Point p1, p2;
            p1.x = edge->pos[0].x;
            p1.y = edge->pos[0].y;
            p1.z = 0; // Set Z to 0 or appropriate value
            p2.x = edge->pos[1].x;
            p2.y = edge->pos[1].y;
            p2.z = 0; // Set Z to 0 or appropriate value
        }

        edge = jcv_diagram_get_next_edge(edge);
    }

    // --------- Use Delaunator to generate edges ----------
    delaunator::Delaunator delaunator(pointsForDelauney);
    const auto &triangles = delaunator.triangles;

    for(size_t i = 0; i < triangles.size(); i += 3)
    {
		for (size_t j = 0; j < 3; ++j)
        {
            int id_starting = triangles[i + j];
		    int id_ending = triangles[i + (j + 1) % 3];
			float x1 = possible_waypoints[id_starting].x;
			float y1 = possible_waypoints[id_starting].y;
			float x2 = possible_waypoints[id_ending].x;
			float y2 = possible_waypoints[id_ending].y;
            if(!line_overlap(x1, y1, x2, y2, obstacles))
            {
                geometry_msgs::msg::Point p1, p2;
                p1.x = x1;
                p1.y = y1;
                p1.z = 0; // Set Z to 0 or appropriate value
                p2.x = x2;
                p2.y = y2;
                p2.z = 0; // Set Z to 0 or appropriate value
                line_list.points.push_back(p1);
                line_list.points.push_back(p2);
            }
        }
    }

    marker_pub_->publish(line_list);

    // Set the service response message
    for (auto waypoint : possible_waypoints)
    {
        geometry_msgs::msg::Point point;

        point.x = waypoint.x;
        point.y = waypoint.y;
        point.z = 0.0;
        response->roadmap.nodes.push_back(point);
        RCLCPP_INFO(this->get_logger(), "Generated node n. %f, %f", waypoint.x, waypoint.y);
    }
    RCLCPP_INFO(this->get_logger(), "\n\n !!!!! Points spawned !!!!!!! \n\n");
}

void VoronoiPoints::listenBorders(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------Received borders");
    borders_ = *msg;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiPoints>());
  rclcpp::shutdown();
  return 0;
}
