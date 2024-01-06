#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "gazebo_msgs/srv/spawn_entity.hpp"

#include "map_pkg/spawn_model.hpp"

#include "gazebo_msgs/srv/spawn_entity.hpp"

#include "map_pkg/spawn_model.hpp"
#include "map_pkg/utilities.hpp"

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

class BordersPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
  std::string gz_models;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;

  // Data obtained from parameters
  struct Data {
    std::string map_name;
    double dx;
    double dy;
  } data ;

public:
  explicit BordersPublisher(bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode("send_borders",
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  {
    this->gz_models = ament_index_cpp::get_package_share_directory("shelfino_gazebo");
    this->configure();
    this->activate();
    this->deactivate();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state)
  {
    this->declare_parameter<std::string>("map", "hexagon");
    this->declare_parameter<double>("dx", 5.0);
    this->declare_parameter<double>("dy", 5.0);
    this->declare_parameter<bool>("use_gui", true);
    this->data.map_name = this->get_parameter("map").as_string();  // hexagon, rectangle
    this->data.dx = this->get_parameter("dx").as_double();
    this->data.dy = this->get_parameter("dy").as_double();

    RCLCPP_INFO(this->get_logger(), "Map name: %s", this->data.map_name.c_str());
    RCLCPP_INFO(this->get_logger(), "dx: %f", this->data.dx);
    RCLCPP_INFO(this->get_logger(), "dy: %f", this->data.dy);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

    this->publisher_  = this->create_publisher<geometry_msgs::msg::Polygon>("/map_borders", qos);
    this->pub_        = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/borders", qos);
    this->spawner_    = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    this->pubm_       = this->create_publisher<visualization_msgs::msg::Marker>("/markers/borders", qos);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    std_msgs::msg::Header hh;

    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    geometry_msgs::msg::Polygon pol;
    std::string xml_string;

    if(this->data.map_name=="hexagon"){
      this->map_hexagon(xml_string, pol);
    }
    else if(this->data.map_name=="rectangle"){
      this->map_rectangle(xml_string, pol);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Map name %s not recognized", this->data.map_name.c_str());
    }

    geometry_msgs::msg::PolygonStamped pol_stamped;
    pol_stamped.header = hh;
    pol_stamped.polygon = pol;

    // Publish borders
    this->publisher_->publish(pol);
    this->pub_->publish(pol_stamped);

    visualization_msgs::msg::Marker mark;

    if(this->data.map_name=="hexagon")         mark = create_hexagon_marker(pol);
    else if(this->data.map_name=="rectangle")  mark = create_rectangle_marker(pol);

    mark.header = hh;
    mark.header.frame_id = "";
    this->pubm_->publish(mark);

    // Spawn model in gazebo
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;
    spawn_model(this->get_node_base_interface(), this->spawner_, xml_string, pose, "border", true);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating node.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubm_;
  void map_hexagon(std::string & xml_string, geometry_msgs::msg::Polygon & pol);
  void map_rectangle(std::string & xml_string, geometry_msgs::msg::Polygon & pol);
};

void
BordersPublisher::map_hexagon(std::string & xml_string, geometry_msgs::msg::Polygon & pol)
{
  pol = create_hexagon(this->data.dx);
  // Read XML file to string
  std::ifstream xml_file(this->gz_models + "/worlds/hexagon_world/model.sdf");
  if (!xml_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file %s",
      (this->gz_models + "/worlds/hexagon_world/model.sdf").c_str()
    );
    exit(1);
  }
  xml_string.assign(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>()
  );

  float original_size = 12.00;     // 13.20
  std::string size_string = "<scale>1 1 1</scale>";
  std::string size_replace_string = "<scale>" +
    std::to_string(this->data.dx/original_size) + " " +
    std::to_string(this->data.dx/original_size) + " 1</scale>";

  size_t pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
}

void
BordersPublisher::map_rectangle(std::string & xml_string, geometry_msgs::msg::Polygon & pol)
{
  pol = create_rectangle(this->data.dx, this->data.dy);
  // Read XML file to string
  std::ifstream xml_file(this->gz_models + "/worlds/rectangle_world/model.sdf");
  if (!xml_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file %s",
      (this->gz_models + "/worlds/rectangle_world/model.sdf").c_str()
    );
    exit(1);
  }
  xml_string.assign(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>()
  );

  float wid = 0.15;
  std::string size_string = "dx";
  std::string size_replace_string = std::to_string(this->data.dx);
  size_t pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
  size_string = "dy";
  size_replace_string = std::to_string(this->data.dy);
  pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
  size_string = "width";
  size_replace_string = std::to_string(wid);
  pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
  size_string = "L1";
  size_replace_string = std::to_string((this->data.dy+wid)/2);
  pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
  size_string = "L2";
  size_replace_string = std::to_string((this->data.dx+wid)/2);
  pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BordersPublisher>();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}