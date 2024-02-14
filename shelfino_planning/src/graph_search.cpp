#include "graph_search.hpp"
#include <algorithm>

void GraphSearch::roadmapCallback(const planning_msgs::msg::RoadmapInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "/*/*/***/*-----------------Roadmap received-----------------");
    auto start_time = std::chrono::system_clock::now();

    planning_msgs::msg::RoadmapInfo roadmapInfo = *msg;
    int startNodeID;
    int goalNodeID;
    std::vector<int> victimsID;

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



        if(node.position.x == roadmapInfo.gate.position.x && node.position.y == roadmapInfo.gate.position.y)
        {
            goalNodeID = node.nodeID;
        }
        if(node.position.x == 0.0 && node.position.y == 0.0)
        {
            startNodeID = node.nodeID;
        }
        for(auto victim : roadmapInfo.victims.obstacles){
            if (node.position.x == victim.polygon.points[0].x && node.position.y == victim.polygon.points[0].y){
            		//RCLCPP_INFO(this->get_logger(), "-> node posx %f and vict pos x: %f", node.position.x, victim.polygon.points[0].x);
        		victimsID.push_back(node.nodeID);
        		node.value = victim.radius;
        	}
        }

        allNodesBackup.push_back(node);

    }


    for(int v : victimsID)
    {
        RCLCPP_INFO(this->get_logger(), "-> VICTIM ID: %d with value: %.2f", v, allNodesBackup[v].value);
    }

    RCLCPP_INFO(this->get_logger(), "Start node id: %d", startNodeID);
    RCLCPP_INFO(this->get_logger(), "Goal node id: %d", goalNodeID);
    RCLCPP_INFO(this->get_logger(), "Start position: %f, %f", allNodesBackup[startNodeID].position.x, allNodesBackup[startNodeID].position.y);
    RCLCPP_INFO(this->get_logger(), "Goal position: %f, %f", allNodesBackup[goalNodeID].position.x, allNodesBackup[goalNodeID].position.y);


    //Toggle here the algorithm chosen
    SalesManBruteSearch(startNodeID, goalNodeID, victimsID, 60.0);
    //SalesManHeuristicSearch(startNodeID, goalNodeID, victimsID, 60.0);

    //printing the path
    for(auto node : salesman_path)
    {
        RCLCPP_INFO(this->get_logger(), "--- FINAL ids of path: %d", node.nodeID);
    }



    visualizePath();

    auto service_duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start_time);
    publishGraphPath(roadmapInfo, service_duration.count());

    return;
}


void GraphSearch::generateCombinations(std::vector<int>& combination, int offset, int k,
                          std::vector<int>& victimsID, double DISCOUNT_FACTOR,
                          double& minDistance, std::vector<int>& minPath,
                          int startNodeID, int goalNodeID, double maxDistance) {
    if (k == 0) {
        do {
		// Try the current combination
		std::vector<int> currentPath = {startNodeID};
		currentPath.insert(currentPath.end(), combination.begin(), combination.end());
		currentPath.push_back(goalNodeID);

		double currentDistance = 0.0;       //this includes the bonus
		double currentDistanceLenght = 0.0; //this only the distance

		// Compute the total distance of the path
		for (size_t j = 0; j < currentPath.size() - 1; ++j) {
		    // Compute the A* distance between consecutive nodes
		    double distance = AStarSearch(currentPath[j], currentPath[j + 1]);
		    currentDistanceLenght+=distance;

		    // Take into consideration the value of the victim
		    distance -= allNodesBackup[currentPath[j + 1]].value / DISCOUNT_FACTOR;

		    // Accumulate the distance
		    currentDistance += distance;
		}

		// Check if the current path is the shortest
		if (currentDistance < minDistance && currentDistanceLenght < maxDistance) {
		    minDistance = currentDistance;
		    minPath = currentPath;
		}
	} while (std::next_permutation(combination.begin(), combination.end()));
        return;
    }

    for (uint i = offset; i <= victimsID.size() - k; ++i) {
        combination.push_back(victimsID[i]);
        generateCombinations(combination, i + 1, k - 1, victimsID, DISCOUNT_FACTOR, minDistance, minPath, startNodeID, goalNodeID, maxDistance);
        combination.pop_back();
    }
}

void GraphSearch::SalesManBruteSearch(int startNodeID, int goalNodeID, std::vector<int> victimsID, double maxDistance) {
    double DISCOUNT_FACTOR = 43.0;
    double minDistance = std::numeric_limits<double>::infinity();
    std::vector<int> minPath;

    // Sort the victims in ascending order based on their values
    std::sort(victimsID.begin(), victimsID.end(), [&](int a, int b) {
        return allNodesBackup[a].value < allNodesBackup[b].value;
    });

    for (uint k = 0; k <= victimsID.size(); ++k) {
        std::vector<int> combination;
        generateCombinations(combination, 0, k, victimsID, DISCOUNT_FACTOR, minDistance, minPath, startNodeID, goalNodeID, maxDistance);
    }

    // Update the salesman_path with the shortest path found
    RCLCPP_INFO(this->get_logger(), "-->> LENGTH of total path given bonus by value: %F", minDistance);

    double total_distance = 0.0;
    for (uint i = 0; i < minPath.size() - 1; i++) {
        total_distance += AStarSearch(minPath[i], minPath[i + 1]);
        salesman_path.insert(salesman_path.end(), path.begin(), path.end());
        if (i != minPath.size() - 2)
            salesman_path.pop_back(); // Remove the last one
    }
    RCLCPP_INFO(this->get_logger(), "-->> LENGTH of total path (without bonus): %F", total_distance);
}




double GraphSearch::AStarSearch(int startNodeID, int goalNodeID)
{

    path.clear();
    openSet.clear();
    closedSet.clear();
    allNodes.clear();
    //there is a better way
    for (uint i=0; i<allNodesBackup.size(); i++)
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




void GraphSearch::SalesManHeuristicSearch(int startNodeID, int goalNodeID, std::vector<int> victimsID, double maxTotalDistance) {
    double DISCOUNT_FACTOR = 20.0;

    // Create a vector to store the ordered path
    std::vector<int> orderedPath;

    // Sort the victims based on their values
    std::sort(victimsID.begin(), victimsID.end(), [&](int a, int b) {
        return allNodesBackup[a].value > allNodesBackup[b].value;
    });
    orderedPath.push_back(startNodeID);

    // Start from the initial node
    int currentNodeID = startNodeID;
    double currentDistance = 0.0;

    while (!victimsID.empty()) {
        double minDistance = std::numeric_limits<double>::infinity();
        int nextNodeID = -1;

        // Find the nearest neighbor
        for (int neighborID : victimsID) {
            double distance = AStarSearch(currentNodeID, neighborID);
            double distanceToGate = AStarSearch(currentNodeID, goalNodeID);
            double totalCost = distance - allNodesBackup[neighborID].value / DISCOUNT_FACTOR;

            currentDistance+= distance;
	    if ((currentDistance + distanceToGate) >= maxTotalDistance) break;

            if (totalCost < minDistance ) {
                minDistance = totalCost;
                nextNodeID = neighborID;
            }
        }

        // If no suitable victim found, break the loop
        if (nextNodeID == -1) {
            break;
        }

        // Add the nearest neighbor to the path
        orderedPath.push_back(nextNodeID);

        // Update the current node and total distance for the next iteration
        currentNodeID = nextNodeID;
        currentDistance += AStarSearch(currentNodeID, orderedPath.back());

        // Remove the chosen victim from the list
        victimsID.erase(std::remove(victimsID.begin(), victimsID.end(), nextNodeID), victimsID.end());
    }

    // Complete the path by adding the goal node
    orderedPath.push_back(goalNodeID);
    RCLCPP_INFO(this->get_logger(), "DIST -----> (tot): %f", currentDistance);

    // Visualize or use the orderedPath as needed
    RCLCPP_INFO(this->get_logger(), "Ordered Path: ");
    for (int nodeID : orderedPath) {
        RCLCPP_INFO(this->get_logger(), "Node ID: %d, Value: %.2f", nodeID, allNodesBackup[nodeID].value);
    }

    // Reconstruct the path for visualization or further use
    double total_distance = 0.0;
    for (uint i = 0; i < orderedPath.size() - 1; i++) {
        total_distance += AStarSearch(orderedPath[i], orderedPath[i + 1]);
        salesman_path.insert(salesman_path.end(), path.begin(), path.end());
        if (i != orderedPath.size() - 2)
            salesman_path.pop_back(); // Remove the last one
    }
    RCLCPP_INFO(this->get_logger(), "-->> LENGTH of total path (without bonus): %F", total_distance);
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

void GraphSearch::publishGraphPath(const planning_msgs::msg::RoadmapInfo roadmapInfo, int duration)
{
    RCLCPP_INFO(this->get_logger(), "Publishing graph path");

    planning_msgs::msg::GraphPath graphPath;
    graphPath.header = roadmapInfo.header;
    graphPath.roadmap = roadmapInfo.roadmap;
    graphPath.generator = roadmapInfo.generator;
    graphPath.roadmap_duration = roadmapInfo.roadmap_duration;
    graphPath.path_planner = "graph_search";
    graphPath.path_planning_duration = duration;
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
