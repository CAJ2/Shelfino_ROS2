#include "graph_search.hpp"

void GraphSearch::roadmapCallback(const planning_msgs::msg::RoadmapInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "/*/*/***/*-----------------Roadmap received-----------------");

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

        allNodes.push_back(node);

        if(node.position.x == roadmapInfo.gate.position.x && node.position.y == roadmapInfo.gate.position.y)
        {
            goalNodeID = node.nodeID;
        }
        if(node.position.x == 0.0 && node.position.y == 0.0)
        {
            startNodeID = node.nodeID;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Start node: %d", startNodeID);
    RCLCPP_INFO(this->get_logger(), "Goal node: %d", goalNodeID);
    RCLCPP_INFO(this->get_logger(), "Start position: %f, %f", allNodes[startNodeID].position.x, allNodes[startNodeID].position.y);
    RCLCPP_INFO(this->get_logger(), "Goal position: %f, %f", allNodes[goalNodeID].position.x, allNodes[goalNodeID].position.y);

    AStarSearch(startNodeID, goalNodeID);

    visualizePath();

    publishGraphPath(roadmapInfo);

    return;
}

void GraphSearch::AStarSearch(int startNodeID, int goalNodeID)
{
    RCLCPP_INFO(this->get_logger(), "Starting A* search");

    graph_search::Node startNode = allNodes[startNodeID];
    graph_search::Node goalNode = allNodes[goalNodeID];

    for(auto& id : startNode.neighbors)
    {
        RCLCPP_INFO(this->get_logger(), "Start node neighbor: %d", id);
    }

    startNode.gCost = 0;
    startNode.computeHeuristic(goalNode, {0});
    startNode.gCost = 0.0;
    openSet.insert(startNode);

    while(!openSet.empty())
    {
        graph_search::Node currentNode = *openSet.begin();
        openSet.erase(currentNode);

        if(currentNode == goalNode)
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            reconstructPath(currentNode, startNode);
            break;
        }

        closedSet.push_back(currentNode);

        for(auto& neighborID : currentNode.neighbors)
        {
            graph_search::Node neighbor = allNodes[neighborID];

            if((std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()))
            {
                continue;
            }

            neighbor.computeHeuristic(goalNode, currentNode);
            auto it = openSet.find(neighbor);

            bool isInOpenSet = false;
            for(auto& node : openSet)
            {
                if(node.nodeID == neighbor.nodeID)
                {
                    isInOpenSet = true;
                    break;
                }
            }

            if(!isInOpenSet)
            {
                neighbor.parentID = currentNode.nodeID;
                openSet.insert(neighbor);
                allNodes[neighbor.nodeID].parentID = neighbor.parentID;
            } else if(neighbor.gCost < it->gCost)
            {
                openSet.erase(it);
                neighbor.parentID = currentNode.nodeID;
                openSet.insert(neighbor);
                allNodes[neighbor.nodeID].parentID = neighbor.parentID;
            }
        }
    }
    return;
}

void GraphSearch::reconstructPath(const graph_search::Node& goal, const graph_search::Node& start)
{
    path.clear();

    graph_search::Node node = goal;

    int i = 0;
    while(node.nodeID != start.nodeID && i < 15)
    {
        path.push_back(node);
        node = allNodes[node.parentID];
        i++;
    }

    path.push_back(start);

    std::reverse(path.begin(), path.end());
}

void GraphSearch::visualizePath()
{
    RCLCPP_INFO(this->get_logger(), "Visualizing path");

    // Publish markers, just for rviz
    std_msgs::msg::Header hh;
    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    visualization_msgs::msg::Marker line_list;

    line_list.header = hh;
    line_list.ns = "graph_search";
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.id = 0; // Make sure marker_id_ is initialized to 0
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.scale.x = 0.2; // Width of the lines

    // Line color and alpha
    line_list.color.r = 1.0;
    line_list.color.g = 0.0;
    line_list.color.b = 0.0;
    line_list.color.a = 1.0;

    for(int i = 0; i < (int)path.size() - 1; i++)
    {
        RCLCPP_INFO(this->get_logger(), "//Path node: %d, x: %f, y: %f", path[i].nodeID, path[i].position.x, path[i].position.y);
        geometry_msgs::msg::Point p1, p2;

        p1.x = path[i].position.x;
        p1.y = path[i].position.y;
        p1.z = 0.0;
        p2.x = path[i+1].position.x;
        p2.y = path[i+1].position.y;
        p2.z = 0.0;

        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
    }

    marker_pub_->publish(line_list);

    RCLCPP_INFO(this->get_logger(), "Path published");
}

void GraphSearch::publishGraphPath(const planning_msgs::msg::RoadmapInfo roadmapInfo)
{
    RCLCPP_INFO(this->get_logger(), "Publishing graph path");

    planning_msgs::msg::GraphPath graphPath;
    graphPath.header = roadmapInfo.header;
    graphPath.roadmap = roadmapInfo.roadmap;
    graphPath.generator = roadmapInfo.generator;
    graphPath.roadmap_duration = roadmapInfo.roadmap_duration;
    graphPath.path_planner = "graph_search";
    graphPath.path_planning_duration = 0;
    graphPath.robot_pose = roadmapInfo.robot_pose;
    graphPath.gate = roadmapInfo.gate;
    graphPath.obstacles = roadmapInfo.obstacles;
    graphPath.victims = roadmapInfo.victims;

    for(auto& node : path)
    {
        planning_msgs::msg::Point2D point;
        point.x = node.position.x;
        point.y = node.position.y;
        graphPath.graph_path_points.push_back(point);
    }

    publisher_graph_path_->publish(graphPath);

    RCLCPP_INFO(this->get_logger(), "Graph path published");

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphSearch>());
  rclcpp::shutdown();
  return 0;
}