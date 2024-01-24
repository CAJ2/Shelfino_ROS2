#include "dubins_node.hpp"


void DubinsPathGenerator::graphPathCallback(const planning_msgs::msg::GraphPath::SharedPtr msg)
{

    planning_msgs::msg::GraphPath graphPath = *msg;

    graphEdges.clear();
    for(int i = 0; i < (int)graphPath.graph_path_points.size() - 1; i++)
    {
        graph::Point point1{graphPath.graph_path_points[i].x, graphPath.graph_path_points[i].y};
        graph::Point point2{graphPath.graph_path_points[i+1].x, graphPath.graph_path_points[i+1].y};
       
        graph::Edge edge{point1, point2};
        graphEdges.push_back(edge);
    }
    RCLCPP_INFO(this->get_logger(), "/*/*/***/*-----------------Graph path received-----------------");
    for(auto& edge : graphEdges)
    {
        RCLCPP_INFO(this->get_logger(), "Edge: %f, %f, %f, %f", edge.p1.x, edge.p1.y, edge.p2.x, edge.p2.y);
    }
    return;
}

void DubinsPathGenerator::visualizePath()
{
}

void DubinsPathGenerator::publishPath()
{
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DubinsPathGenerator>());
  rclcpp::shutdown();
  return 0;
}