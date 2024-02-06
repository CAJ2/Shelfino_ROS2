#include <unistd.h>
#include <iostream>
#include <fstream>
#include <random>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "utilities.hpp"
#include "execute_path.hpp"

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
#include "planning_msgs/msg/roadmap_info.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

void ExecutePath::graphPathCallback(const planning_msgs::msg::GraphPath::SharedPtr msg)
{
    graph_paths.push_back(*msg);
    RCLCPP_INFO(this->get_logger(), "Received graph path from node %s", msg->path_planner.c_str());
}

void ExecutePath::runPath()
{
    if(graph_paths.size() > 0)
    {
        if(!running_path)
        {
            current_path = graph_paths[0];
            graph_paths.erase(graph_paths.begin());
            running_path = true;
            RCLCPP_INFO(this->get_logger(), "Running path from node %s", current_path.path_planner.c_str());

            int path_split = 100;
            for (int i = 0; i < current_path.graph_path.poses.size(); i += path_split)
            {
                RCLCPP_INFO(this->get_logger(), "Path node %d: x:%f, y:%f", i, current_path.graph_path.poses[i].pose.position.x, current_path.graph_path.poses[i].pose.position.y);
                nav_msgs::msg::Path path;
                path.header = current_path.graph_path.header;
                for (int j = i; j < i + path_split && j < current_path.graph_path.poses.size(); j++)
                {
                    path.poses.push_back(current_path.graph_path.poses[j]);
                }
                this->path_goals.push_back(path);
            }
            RCLCPP_INFO(this->get_logger(), "Path split into %d goals", this->path_goals.size());

            if (!this->action_follow_path->wait_for_action_server(10s)) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                running_path = false;
                return;
            }
            this->start_time = this->now();
        }
    }

    if(running_path)
    {
        if (!this->current_path_running) {
            if (this->current_path_goal >= this->path_goals.size())
            {
                this->running_path = false;
                this->current_path_goal = 0;
                this->path_goals.clear();
                RCLCPP_INFO(this->get_logger(), "Finished running path from node %s", current_path.path_planner.c_str());
                return;
            }
            this->publisher_action_plan->publish(this->path_goals[this->current_path_goal]);
            RCLCPP_INFO(this->get_logger(), "Published path from node %s", current_path.path_planner.c_str());

            // Debug print the current path_goals
            auto first_path_goal = this->path_goals[this->current_path_goal].poses.front();
            RCLCPP_INFO(this->get_logger(), "Path goal start: %f, %f", first_path_goal.pose.position.x, first_path_goal.pose.position.y);
            auto last_path_goal = this->path_goals[this->current_path_goal].poses.back();
            RCLCPP_INFO(this->get_logger(), "Path goal end: %f, %f", last_path_goal.pose.position.x, last_path_goal.pose.position.y);

            this->path_goals[this->current_path_goal].header.stamp = this->get_clock()->now();
            this->path_goals[this->current_path_goal].header.frame_id = "map";
            auto goal_msg = FollowPath::Goal();
            goal_msg.path = this->path_goals[this->current_path_goal];
            goal_msg.controller_id = "FollowPath";
            goal_msg.goal_checker_id = "goal_checker";

            auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&ExecutePath::goalResponseCallback, this, _1);
            send_goal_options.feedback_callback = std::bind(&ExecutePath::feedbackCallback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&ExecutePath::resultCallback, this, _1);
            this->action_follow_path->async_send_goal(goal_msg, send_goal_options);
            RCLCPP_INFO(this->get_logger(), "Sent action goal for node %s", current_path.path_planner.c_str());
            this->current_path_running = true;
            this->current_path_goal++;
        }

        if(current_path.roadmap.nodes.size() > 0)
        {
            visualization_msgs::msg::MarkerArray marks;

            //Here for the nodes
            for (size_t i = 0; i < current_path.roadmap.nodes.size(); i++) {
                auto node = current_path.roadmap.nodes[i];
                visualization_msgs::msg::Marker mark = add_point(node.x , node.y, i);
                marks.markers.push_back(mark);
            }

            //Here for the edges
            int id = current_path.roadmap.nodes.size();
            for (size_t i = 0; i < current_path.roadmap.edges.size(); i++) {
                auto nodes = current_path.roadmap.edges[i].node_ids;

                for(uint node_id : nodes){
                    // Edge is intended that the index of the edge in the list is the id
                    // of the starting node, and the list contains the connected nodes ids
                    float x1 = current_path.roadmap.nodes[i].x;
                    float y1 = current_path.roadmap.nodes[i].y;
                    float x2 = current_path.roadmap.nodes[node_id].x;
                    float y2 = current_path.roadmap.nodes[node_id].y;
                    visualization_msgs::msg::Marker mark = add_line( x1, y1, x2, y2, id);
                    marks.markers.push_back(mark);
                    id++;
                }

            }

            this->publisher_rviz_graph->publish(marks);
        }

        this->publisher_rviz->publish(current_path.graph_path);
    }
}

void ExecutePath::goalResponseCallback(const GoalHandleFollowPath::SharedPtr& msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        running_path = false;
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void ExecutePath::feedbackCallback(const GoalHandleFollowPath::SharedPtr& msg, const std::shared_ptr<const FollowPath::Feedback> feedback)
{
    std::stringstream ss;
    ss << "Feedback received with " << feedback->distance_to_goal << " distance to goal";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void ExecutePath::resultCallback(const GoalHandleFollowPath::WrappedResult& result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal was successfully executed. Waiting for next path");
            RCLCPP_INFO(this->get_logger(), "Path goals remaining: %d", this->path_goals.size() - this->current_path_goal);
            this->current_path_running = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            running_path = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            running_path = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            running_path = false;
            break;
    }
}


visualization_msgs::msg::Marker ExecutePath::add_point(float x, float y, int id) {
    // Publish markers, just for rviz
    std_msgs::msg::Header hh;
    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;

    visualization_msgs::msg::Marker mark;

    mark.header = hh;
    mark.ns = "execute_path";
    mark.id = id;
    mark.action = visualization_msgs::msg::Marker::ADD;
    mark.type = visualization_msgs::msg::Marker::CYLINDER;
    mark.pose = pose;
    mark.scale.x = 0.3;
    mark.scale.y = 0.3;
    mark.scale.z = 0.01;
    mark.color.a = 1.0;
    mark.color.r = 0.3;
    mark.color.g = 1.0;
    mark.color.b = 0.5;

    return mark;
}

visualization_msgs::msg::Marker ExecutePath::add_line(float x1, float y1, float x2, float y2, int id) {
    // Publish markers, just for rviz
    std_msgs::msg::Header hh;
    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    visualization_msgs::msg::Marker mark;

    mark.header = hh;
    mark.ns = "execute_path";
    mark.id = id;
    mark.action = visualization_msgs::msg::Marker::ADD;
    mark.type = visualization_msgs::msg::Marker::LINE_STRIP;

    geometry_msgs::msg::Point start_point;
    start_point.x = x1;
    start_point.y = y1;
    start_point.z = 0.0;

    geometry_msgs::msg::Point end_point;
    end_point.x = x2;
    end_point.y = y2;
    end_point.z = 0.0;

    mark.points.push_back(start_point);
    mark.points.push_back(end_point);

    mark.scale.x = 0.03; //line width

    mark.color.a = 0.5;
    mark.color.r = 0.0;
    mark.color.g = 0.0;
    mark.color.b = 1.0;
    return mark;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExecutePath>());
    rclcpp::shutdown();
    return 0;
}
