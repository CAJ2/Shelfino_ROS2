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
            polygon = approximateCircle(obs.x, obs.y, obs.radius + collisionThreshold, polygonPoints);
            for(int i; i < polygon.points.size() - 1; i++)
            {
                planning_msgs::msg::Point2D point1;
                point1.x = obs.polygon.points[i].x; 
                point1.y = obs.polygon.points[i].y;

                planning_msgs::msg::Point2D point2;
                point1.x = obs.polygon.points[i+1].x; 
                point1.y = obs.polygon.points[i+1].y;
                dubins::Edge edge{point1, point2};
                obstacleEdges.push_back(edge);
            }
        } else if("BOX" == obs.type)
        {
            double x1 = obs.x - obs.dx/2 - collisionThreshold/2.0;
            double y1 = obs.y - obs.dy/2 - collisionThreshold/2.0;
            double x2 = obs.x + obs.dx/2 + collisionThreshold/2.0;
            double y2 = obs.y - obs.dy/2 - collisionThreshold/2.0;
            double x3 = obs.x + obs.dx/2 + collisionThreshold/2.0;
            double y3 = obs.y + obs.dy/2 + collisionThreshold/2.0;
            double x4 = obs.x - obs.dx/2 - collisionThreshold/2.0;
            double y4 = obs.y + obs.dy/2 + collisionThreshold/2.0;
            planning_msgs::msg::Point2D point1;
            point1.x = x1;
            point1.y = y1;
            planning_msgs::msg::Point2D point2;
            point2.x = x2;
            point2.y = y2;
            planning_msgs::msg::Point2D point3;
            point3.x = x3;
            point3.y = y3;
            planning_msgs::msg::Point2D point4;
            point4.x = x4;
            point4.y = y4;
            dubins::Edge edge1{point1, point2};
            dubins::Edge edge2{point2, point3};
            dubins::Edge edge3{point3, point4};
            dubins::Edge edge4{point4, point1};
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
        int deviderNumber = 10;
        dubins::DubinsArc* arc = dubinsCurves[i]->a1;

        float deviderLength = arc->L / (float)deviderNumber;
        for(float s = 0.0; s < arc->L; s += deviderLength)
        {
            geometry_msgs::msg::Pose pose;
            dubins::DubinsLine line(s, arc->x0, arc->y0, arc->th0, arc->k);
            pose.position.x = line.x;
            pose.position.y = line.y;
            pose.position.z = line.th; // yes, to the z the orientation is assigned
            dubinsPathPoints.push_back(pose);
        }

        arc = dubinsCurves[i]->a2;
        for(float s = 0.0; s < arc->L; s += deviderLength)
        {
            geometry_msgs::msg::Pose pose;
            dubins::DubinsLine line(s, arc->x0, arc->y0, arc->th0, arc->k);
            pose.position.x = line.x;
            pose.position.y = line.y;
            pose.position.z = line.th; // yes, to the z the orientation is assigned
            dubinsPathPoints.push_back(pose);
        }

        arc = dubinsCurves[i]->a3;
        for(float s = 0.0; s < arc->L; s += deviderLength)
        {
            geometry_msgs::msg::Pose pose;
            dubins::DubinsLine line(s, arc->x0, arc->y0, arc->th0, arc->k);
            pose.position.x = line.x;
            pose.position.y = line.y;
            pose.position.z = line.th; // yes, to the z the orientation is assigned
            dubinsPathPoints.push_back(pose);
        }
    }

    for(int i = 0; i < graphPath.graph_path_points.size() - 1; i++)
    {
        delete dubinsCurves[i];
    }
    delete[] dubinsCurves;

    visualizePath();

    publishPath(graphPath);

    return;
}

void DubinsPathGenerator::visualizePath()
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    auto now = this->now();
    path.header.stamp = now;
    for (auto p : dubinsPathPoints)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now;
        pose.pose = p;
        path.poses.push_back(pose);
    }
    this->marker_pub_->publish(path);
}

void DubinsPathGenerator::publishPath(planning_msgs::msg::GraphPath msg)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    auto now = this->now();
    path.header.stamp = now;
    for (auto p : dubinsPathPoints)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now;
        pose.pose = p;
        path.poses.push_back(pose);
    }
    msg.graph_path = path;

    this->publisher_dubins_path_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DubinsPathGenerator>());
  rclcpp::shutdown();
  return 0;
}