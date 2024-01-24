cmake_minimum_required(VERSION 3.8)
project(shelfino_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-Wno-error)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(planning_msgs REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# list of nodes
set(NODES
  static_robot_state
  roadmap_harness
  random_points
  voronoi_points
  graph_search
  dubins_node
)

add_library(${PROJECT_NAME}_lib SHARED
  src/utilities.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  ${PROJECT_NAME}_lib
  "geometry_msgs"
  "rclcpp"
  "obstacles_msgs"
  "tf2"
  "visualization_msgs"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "planning_msgs"
  "rclcpp_action"
)

# build nodes
foreach(NODE ${NODES})
  add_executable(${NODE} src/${NODE}.cpp src/utilities.cpp src/utils.cpp src/graph.cpp src/dubins.cpp)

  target_include_directories(${NODE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_compile_features(${NODE} PUBLIC c_std_99 cxx_std_17)

  # target_link_libraries(${NODE} ${PROJECT_NAME}_lib)

  ament_target_dependencies(
    ${NODE}
    "rclcpp"
    "std_msgs"
    "nav_msgs"
    "obstacles_msgs"
    "tf2"
    "tf2_ros"
    "lifecycle_msgs"
    "rclcpp_lifecycle"
    "rclcpp"
    "geometry_msgs"
    "tf2_geometry_msgs"
    "visualization_msgs"
    "planning_msgs"
    "rclcpp_action"
  )

  install(TARGETS ${NODE}
    DESTINATION lib/${PROJECT_NAME})
endforeach()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
