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

        allNodesBackup.push_back(node);

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
    RCLCPP_INFO(this->get_logger(), "Start position: %f, %f", allNodesBackup[startNodeID].position.x, allNodesBackup[startNodeID].position.y);
    RCLCPP_INFO(this->get_logger(), "Goal position: %f, %f", allNodesBackup[goalNodeID].position.x, allNodesBackup[goalNodeID].position.y);


    SalesManSearch();
    
    //printing the path
    for(auto node : salesman_path)
    {
        RCLCPP_INFO(this->get_logger(), "--- FINAL ids of path: %d", node.nodeID);
    }
    
    
    
    visualizePath();

    publishGraphPath(roadmapInfo);

    return;
}

void GraphSearch::SalesManSearch(){
    double minDistance = std::numeric_limits<double>::infinity();
    std::vector<int> minPath;
    std::vector<int> victimsID = {1,2,3,4,5};
    std::sort(victimsID.begin(), victimsID.end());  //then we will need to populate using the victims list inside the info

    do {
        // Include the start and goal nodes in the path
        std::vector<int> currentPath = {0};
        currentPath.insert(currentPath.end(), victimsID.begin(), victimsID.end());
        currentPath.push_back(6);

        double currentDistance = 0.0;

        // Compute the total distance of the path
        for (size_t i = 0; i < currentPath.size() - 1; ++i) {
            currentDistance += AStarSearch(currentPath[i], currentPath[i + 1]);
        }

        // Check if the current path is the shortest
        if (currentDistance < minDistance) {
            minDistance = currentDistance;
            minPath = currentPath;
        }

    } while (std::next_permutation(victimsID.begin(), victimsID.end()));

    // Update the salesman_path with the shortest path found
    RCLCPP_INFO(this->get_logger(), "-->> LENGTH of total path: %F", minDistance);
    salesman_path.clear(); //not needed?
    
    double total_distance = 0.0;
    for (int i=0; i<minPath.size()-1; i++) {
        
        total_distance += AStarSearch(minPath[i], minPath[i + 1]);
    	salesman_path.insert( salesman_path.end(), path.begin(), path.end() );
    	if (i!=minPath.size()-2)
    		salesman_path.pop_back(); //remove the last one
        
    }
    RCLCPP_INFO(this->get_logger(), "-->> LENGTH of total path second: %F", total_distance);
   

}

double GraphSearch::AStarSearch(int startNodeID, int goalNodeID)
{

    path.clear();
    openSet.clear(); 
    closedSet.clear(); 
    allNodes.clear();
    //there is a better way 
    for (int i=0; i<allNodesBackup.size(); i++)  
        allNodes.push_back(allNodesBackup[i]);  
    
    //RCLCPP_INFO(this->get_logger(), "Starting A* search");

    graph_search::Node startNode = allNodes[startNodeID];
    graph_search::Node goalNode = allNodes[goalNodeID];
/*
    for(auto& id : startNode.neighbors)
    {
        RCLCPP_INFO(this->get_logger(), "Start node neighbor: %d", id);
    }
*/
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
            //RCLCPP_INFO(this->get_logger(), "Goal reached");
            return reconstructPath(currentNode, startNode);
            
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
    return 0.0;
}

double GraphSearch::reconstructPath(const graph_search::Node& goal, const graph_search::Node& start)
{
    graph_search::Node node = goal;

    double track_distance=0.0;
    while(node.nodeID != start.nodeID) 
    {
        track_distance += graph_search::distance(node.position, allNodes[node.parentID].position);
        path.push_back(node);
        node = allNodes[node.parentID];
    }

    path.push_back(start);

    std::reverse(path.begin(), path.end());
    
    return track_distance;
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

    for(auto& node : salesman_path)
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
