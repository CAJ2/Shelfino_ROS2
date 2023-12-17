#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

geometry_msgs::msg::Polygon create_hexagon(double dx){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        double f = 0.866;  // fixed number for apothem of hexagon calculation
        point.x = -dx/2;
        point.y = dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = -dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -dx/2;
        point.y = -dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -dx;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}


geometry_msgs::msg::Polygon create_rectangle(double dx, double dy){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        point.x = -dx/2;
        point.y = -dy/2;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -dx/2;
        point.y = dy/2;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = dy/2;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = -dy/2;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}

visualization_msgs::msg::Marker create_hexagon_marker(geometry_msgs::msg::Polygon pol){
        visualization_msgs::msg::Marker mark;
        geometry_msgs::msg::Point point;
        std::vector<geometry_msgs::msg::Point> points_temp;
        for (auto p : pol.points){
          point.x = p.x;
          point.y = p.y;
          point.z = 0;
          points_temp.push_back(point);
        }

        mark.ns = "borders";
        mark.id = 0;
        mark.type = visualization_msgs::msg::Marker::LINE_STRIP;
        mark.action = visualization_msgs::msg::Marker::ADD;
        mark.scale.x = 5.3;
        mark.color.a = 1.0;
        mark.color.r = 0.0;
        mark.color.g = 1.0;
        mark.color.b = 0.0;
        mark.points = points_temp;
        return mark;
}


visualization_msgs::msg::Marker create_rectangle_marker(geometry_msgs::msg::Polygon pol){
        visualization_msgs::msg::Marker mark;
        geometry_msgs::msg::Point point;
        std::vector<geometry_msgs::msg::Point> points_temp;
        for (auto p : pol.points){
          point.x = p.x;
          point.y = p.y;
          point.z = 0;
          points_temp.push_back(point);
        }

        mark.ns = "borders";
        mark.id = 0;
        mark.type = visualization_msgs::msg::Marker::LINE_STRIP;
        mark.action = visualization_msgs::msg::Marker::ADD;
        mark.scale.x = 0.3;
        mark.color.a = 1.0;
        mark.color.r = 0.0;
        mark.color.g = 1.0;
        mark.color.b = 0.0;
        mark.points = points_temp;
        return mark;
}

static const rmw_qos_profile_t rmw_qos_profile_custom =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

class BordersPublisher : public rclcpp::Node
{
private:
  std::string share_dir;

public:
  BordersPublisher()
  : Node("send_borders")
  {
    this->share_dir = ament_index_cpp::get_package_share_directory("map_pkg");

    this->declare_parameter<std::string>("map", "hexagon");
    this->declare_parameter<double>("dx", 5.0);
    this->declare_parameter<double>("dy", 5.0);
    this->declare_parameter<bool>("use_gui", true);
    std::string map_name = this->get_parameter("map").as_string();  // hexagon, rectangle
    double dx = this->get_parameter("dx").as_double();
    double dy = this->get_parameter("dy").as_double();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
    publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("/map_borders", qos);

    std_msgs::msg::Header hh;

    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    geometry_msgs::msg::Polygon pol;

    geometry_msgs::msg::PolygonStamped pol_stamped;

    pol_stamped.header = hh;

    if(map_name=="hexagon")         pol = create_hexagon(dx);
    else if(map_name=="rectangle")  pol = create_rectangle(dx,dy);

    pol_stamped.polygon = pol;

    pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/borders", qos);

    publisher_->publish(pol);
    pub_->publish(pol_stamped);

    visualization_msgs::msg::Marker mark;

    if(map_name=="hexagon")         mark = create_hexagon_marker(pol);
    else if(map_name=="rectangle")  mark = create_rectangle_marker(pol);

    mark.header = hh;
    mark.header.frame_id = "";
    pubm_ = this->create_publisher<visualization_msgs::msg::Marker>("/markers/borders", qos);
    pubm_->publish(mark);

    usleep(1000000);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubm_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BordersPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}