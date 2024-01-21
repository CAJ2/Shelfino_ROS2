#include "graph_search.hpp"


visualization_msgs::msg::Marker GraphSearch::add_line(float x1, float y1, float x2, float y2, std::string service, int id) {
        // Publish markers, just for rviz
        std_msgs::msg::Header hh;
        hh.stamp = this->get_clock()->now();
        hh.frame_id = "map";

        visualization_msgs::msg::Marker mark;

        mark.header = hh;
        mark.ns = service;
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

void GraphSearch::roadmapCallback(const planning_msgs::msg::RoadmapInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Roadmap received");

    planning_msgs::msg::RoadmapInfo roadmapInfo = *msg;

    int startNodeID;
    int goalNodeID;

    for(int i = 0; i < (int)roadmapInfo.roadmap.edges.size(); i++)
    {
        const auto& nodes = roadmapInfo.roadmap.edges[i].node_ids;
        graph_search::Node node(i);
        node.position.x = roadmapInfo.roadmap.nodes[i].x;
        node.position.y = roadmapInfo.roadmap.nodes[i].y;
        for(auto& n : nodes)
        {
            node.neighbors.push_back(n);
        }

        allNodes.insert(std::pair<int, graph_search::Node>(node.nodeID, node));

        if(node.position.x == roadmapInfo.gate.position.x && node.position.y == roadmapInfo.gate.position.y)
        {
            goalNodeID = node.nodeID;
        }
        if(node.position.x == 0.0 && node.position.y == 0.0)
        {
            startNodeID = node.nodeID;
        }
    }

    AStarSearch(startNodeID, goalNodeID);

    return;
}

void GraphSearch::AStarSearch(int startNodeID, int goalNodeID)
{
    RCLCPP_INFO(this->get_logger(), "Starting A* search");

    path.clear();

    graph_search::Node* startNode = &allNodes[startNodeID];
    graph_search::Node* goalNode = &allNodes[goalNodeID];

    startNode -> gCost = 0;
    startNode -> computeHeuristic(*goalNode);
    openSet.push(*startNode);

    while(!openSet.empty())
    {
        graph_search::Node currentNode = openSet.top();
        openSet.pop();

        if(currentNode == *goalNode)
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            return;
        }

        closedSet.insert(std::pair<int, graph_search::Node>(currentNode.nodeID, currentNode));

        for(auto& neighborID : currentNode.neighbors)
        {
            graph_search::Node* neighbor = &allNodes[neighborID];

            if(closedSet.find(neighborID) != closedSet.end())
            {
                continue;
            }

            double tentativeGCost = currentNode.gCost + graph_search::distance(currentNode.position, neighbor->position);

            if(tentativeGCost < neighbor->gCost)
            {
                neighbor->parent = &currentNode;
                neighbor->gCost = tentativeGCost;
                neighbor->computeHeuristic(*goalNode);

                if(std::find(openSetTracker.begin(), openSetTracker.end(), neighbor) == openSetTracker.end())
                {
                    openSet.push(*neighbor);
                    openSetTracker.insert(neighborID);
                }
            }
        }
    }   
}

void GraphSearch::reconstructPath(graph_search::Node* current)
{
    while(current != nullptr)
    {
        path.push_back(*current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
}