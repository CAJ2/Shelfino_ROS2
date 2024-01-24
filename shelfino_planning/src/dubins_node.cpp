#include "dubins_node.hpp"
#include "dubins.hpp"

geometry_msgs::msg::Polygon approximateCircle(float x, float y, float radius, int num_points) {
    std::vector<geometry_msgs::msg::Point32> points;
    geometry_msgs::msg::Polygon polygon;
    for (int i = 0; i < num_points; i++) {
        float angle = i * 2 * M_PI / num_points;
        geometry_msgs::msg::Point32 p;
        p.x = x + radius * cos(angle);
        p.y = y + radius * sin(angle);
        points.push_back(p);
    }
    polygon.set__points(points);
    return polygon;
}

void DubinsPathGenerator::roadmapCallback(const planning_msgs::msg::RoadmapInfo::SharedPtr msg)
{
    planning_msgs::msg::RoadmapInfo roadmapInfo = *msg;

    for(const auto& obs : roadmapInfo.obstacles.obstacles)
    {
        if("CYLINDER" == obs.type)
        {
            geometry_msgs::msg::Polygon polygon;
            polygon = approximateCircle(obs.x, obs.y, obs.radius, 20);    
            for(int i; i < polygon.points.size() - 1; i++)
            {
                graph::Point point1{obs.polygon.points[i].x, obs.polygon.points[i].y};
                graph::Point point2{obs.polygon.points[i+1].x, obs.polygon.points[i+1].y};
                graph::Edge edge{point1, point2};
                obstacleEdges.push_back(edge);
            }
        } else if("BOX" == obs.type)
        {
            double x1 = obs.x - obs.dx/2;
            double y1 = obs.y - obs.dy/2;
            double x2 = obs.x + obs.dx/2;
            double y2 = obs.y - obs.dy/2;
            double x3 = obs.x + obs.dx/2;
            double y3 = obs.y + obs.dy/2;
            double x4 = obs.x - obs.dx/2;
            double y4 = obs.y + obs.dy/2;
            graph::Point point1{x1, y1};
            graph::Point point2{x2, y2};
            graph::Point point3{x3, y3};
            graph::Point point4{x4, y4};
            graph::Edge edge1{point1, point2};
            graph::Edge edge2{point2, point3};
            graph::Edge edge3{point3, point4};
            graph::Edge edge4{point4, point1};
            obstacleEdges.push_back(edge1);
            obstacleEdges.push_back(edge2);
            obstacleEdges.push_back(edge3);
            obstacleEdges.push_back(edge4);
        }
    }

    RCLCPP_INFO(this->get_logger(), "/*/*/***/*-----------------Obstacles received-----------------");
}

void DubinsPathGenerator::graphPathCallback(const planning_msgs::msg::GraphPath::SharedPtr msg)
{

    planning_msgs::msg::GraphPath graphPath = *msg;

    dubins::DubinsPoint** dubinsPoints = new dubins::DubinsPoint*[graphPath.graph_path_points.size()];

    for(int i = 0; i < (int)graphPath.graph_path_points.size() - 1; i++)
    {
        graph::Point point1{graphPath.graph_path_points[i].x, graphPath.graph_path_points[i].y};
        graph::Point point2{graphPath.graph_path_points[i+1].x, graphPath.graph_path_points[i+1].y};
       
        dubinsPoints[i] = new dubins::DubinsPoint{graphPath.graph_path_points[i].x, graphPath.graph_path_points[i].y, 0.0};
    }

    dubinsPoints[graphPath.graph_path_points.size() - 1] = 
        new dubins::DubinsPoint{graphPath.graph_path_points[graphPath.graph_path_points.size() - 1].x, 
                                graphPath.graph_path_points[graphPath.graph_path_points.size() - 1].y, 
                                0.0};

    RCLCPP_INFO(this->get_logger(), "/*/*/***/*-----------------Graph path received-----------------");

    dubins::Dubins multipointDubins(2.0, 0.01);

    dubins::DubinsCurve** dubinsCurves = multipointDubins.multipointShortestPath(dubinsPoints, graphPath.graph_path_points.size(), obstacleEdges);

    RCLCPP_INFO(this->get_logger(), "/*/*/***/*-----------------Dubins path generated-----------------");

    for(int i = 0; i < graphPath.graph_path_points.size(); i++)
    {
        delete dubinsPoints[i];
    }
    delete[] dubinsPoints;

    for(int i = 0; i < graphPath.graph_path_points.size() - 1; i++)
    {
        delete dubinsCurves[i];
    }
    delete[] dubinsCurves;

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