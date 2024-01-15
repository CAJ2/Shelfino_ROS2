
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

#define JC_VORONOI_IMPLEMENTATION
#include "jc_voronoi.h"
#define JC_VORONOI_CLIP_IMPLEMENTATION
#include "jc_voronoi_clip.h"

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

    // First add the victims we already have

    //!!The information about their value is contained in the radius of the "victims" vector,
    // not in the "possible_waypoints", which just contains their position and distance between them
    for (auto v : request->victims.obstacles)
    {
        victim vict = victim(v.x, v.y);
        vict.radius = 0.25;
        possible_waypoints.push_back(vict);
    }

    std::vector<obstacle> obstacles = msg_to_obstacles(request->obstacles);
    std::vector<obstacle> gates = msg_to_obstacles(request->gate);

    auto startTime = this->get_clock()->now();
    int trials = 0;


    //jcv_point_ * clipPoints = (jcv_point_*)malloc(sizeof(jcv_point_) * 6);

    // uint8_t i = 0;
    // for(auto point : borders_.points)
    // {
    //     if(point.x > maxX)
    //         maxX = point.x;
    //     if(point.x < minX)
    //         minX = point.x;
    //     if(point.y > maxY)
    //         maxY = point.y;
    //     if(point.y < minY)
    //         minY = point.y;
    //     clipPoints[i].x = point.x;
    //     clipPoints[i].y = point.y;
    //     i++;
    // }

    //jcv_clipping_polygon_ clipPoly;
    //jcv_clipper_* clipper;
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
    

    // if(clipPoints)
    // {
    //     clipPoly.num_points = 6;
    //     clipPoly.points = clipPoints;
    //     jcv_clipper polygonclipper;
    //     polygonclipper.test_fn = jcv_clip_polygon_test_point;
    //     polygonclipper.clip_fn = jcv_clip_polygon_clip_edge;
    //     polygonclipper.fill_fn = jcv_clip_polygon_fill_gaps;
    //     polygonclipper.ctx = &clipPoly;

    //     clipper = &polygonclipper;
    // }
    // else
    // {
    //     clipPoly.num_points = 0;
    //     clipPoly.points = 0;
    // }


    //int numOfRelaxations = 0;

    jcv_diagram_ diagram;
    memset(&diagram, 0, sizeof(jcv_diagram_));

    // for (int i = 0; i < numOfRelaxations; ++i)
    // {
    //     jcv_diagram_generate(numOfPoints, (const jcv_point*)points, &rect, clipper, &diagram);
    //     relax_points(&diagram, points);
    // }

    jcv_diagram_generate(numOfPoints, (const jcv_point*)points, &rect, nullptr, &diagram);
    const jcv_edge_* edge = jcv_diagram_get_edges(&diagram);


    while (edge)
    {
        victim new_element{0, 0};

        new_element.x = edge->pos[0].x;
        new_element.y = edge->pos[0].y;
        new_element.radius = 0.;
        possible_waypoints.push_back(new_element);
        // if (valid_position(map_name, 10, 10, new_element, {possible_waypoints, obstacles, gates}))
        // {
        //     possible_waypoints.push_back(new_element); //for the internal list of waypoints
        // }
        edge = jcv_diagram_get_next_edge(edge);
    }
            RCLCPP_ERROR(this->get_logger(), "---------------%ld", possible_waypoints.size());

    if (overTime(this->get_clock(), startTime, 10))
    {
        RCLCPP_ERROR(this->get_logger(), "Map already filled at node n. %i after trials %i", i, trials);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Generated node n. %i after trials %i", i, trials);
    }

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
