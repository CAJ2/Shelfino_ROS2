cmake_minimum_required(VERSION 3.8)
project(planning_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/RoadmapEdge.msg"
  "msg/Roadmap.msg"
  "msg/RoadmapInfo.msg"
  "msg/GraphPath.msg"
  "srv/GenRoadmap.srv"
  "action/GenRoadmap.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs obstacles_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
