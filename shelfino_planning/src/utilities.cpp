#include "utilities.hpp"
#include "planning_msgs/srv/gen_roadmap.hpp"

#include <vector>

#include "delaunator.hpp"


h2d::CPolyline obs_to_cpoly (const obstacle& obs){
  h2d::CPolyline poly;
  if (obs.type == obstacle_type::BOX){
    poly = h2d::CPolyline(std::vector<h2d::Point2d> {
      {obs.x - obs.dx/2.0, obs.y + obs.dy/2.0},
      {obs.x - obs.dx/2.0, obs.y - obs.dy/2.0},
      {obs.x + obs.dx/2.0, obs.y - obs.dy/2.0},
      {obs.x + obs.dx/2.0, obs.y + obs.dy/2.0}
    });
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Obstacle type %d not supported by this function.", obs.type);
  }
  return poly;
}

h2d::Circle obs_to_circle (const obstacle& obs){
  h2d::Circle circle;
  if (obs.type == obstacle_type::CYLINDER){
    circle = h2d::Circle(h2d::Point2d(obs.x, obs.y), obs.radius);
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Obstacle type %d not supported by this function.", obs.type);
  }
  return circle;
}

/**
 * @brief Function that checks that two obstacles do not overlap
 * 
 * @param obs1 Obstacle 1
 * @param obs2 Obstacle 2
 * @return true If the obstacles overlap
 * @return false If the obstacles do not overlap
 */
bool overlaps(obstacle obs1, obstacle obs2) {
  bool ret = false;
  if (obs1.type == obstacle_type::CYLINDER && obs2.type == obstacle_type::CYLINDER) {
    h2d::Circle obs1_circle(obs_to_circle(obs1));
    h2d::Circle obs2_circle(obs_to_circle(obs2));

    ret = ( 
            obs1_circle == obs2_circle ||
            obs1_circle.intersects(obs2_circle).size() > 0 || 
            obs1_circle.isInside(obs2_circle) || 
            obs2_circle.isInside(obs1_circle)
    );
  } 
  else if (obs1.type == obstacle_type::BOX && obs2.type == obstacle_type::BOX) {
    h2d::CPolyline obs1_rect(obs_to_cpoly(obs1));
    h2d::CPolyline obs2_rect(obs_to_cpoly(obs2));

    ret = (
            obs1_rect == obs2_rect ||
            obs1_rect.intersects(obs2_rect).size() > 0 || 
            obs1_rect.isInside(obs2_rect) || 
            obs2_rect.isInside(obs1_rect)
    );
  } 
  else {
    if (obs1.type == obstacle_type::BOX) {
      obstacle temp = obs1;
      obs1 = obs2;
      obs2 = temp;
    }

    h2d::Circle obs1_circle(obs_to_circle(obs1));
    h2d::CPolyline obs2_rect(obs_to_cpoly(obs2));

    ret = (
            obs1_circle.intersects(obs2_rect).size() > 0 || 
            obs1_circle.isInside(obs2_rect) || 
            obs2_rect.isInside(obs1_circle)
    );
  }
  return ret;
}

/**
 * @brief Function that checks if an obstacles overlaps with any of the obstacles in a vector
 * 
 * @param obs1 The obstacle to check
 * @param obstacles The vector of obstacles to check against
 * @return true If the obstacle overlaps with any of the obstacles in the vector
 * @return false If the obstacle does not overlap with any of the obstacles in the vector
 */
bool overlaps(obstacle obs1, std::vector<obstacle> obstacles){
  for (auto obs2 : obstacles){
    if (overlaps(obs1, obs2)){
      return true;
    }
  }
  return false;
}


/**
 * @brief Function that checks if a line segment overlaps with an obstacle
 * 
 * @param x1, y1 The starting point of the line segment
 * @param x2, y2 The ending point of the line segment
 * @param obs The obstacle to check against
 * @return true If the line segment overlaps with the obstacle
 * @return false If the line segment does not overlap with the obstacle
 */
bool overlaps(double x1, double y1, double x2, double y2, const obstacle& obs) {
 
  
  if (obs.type == obstacle_type::CYLINDER){
   	h2d::Segment line_segment(h2d::Point2d(x1, y1), h2d::Point2d(x2, y2));
  	h2d::Circle obs_geom(obs_to_circle(obs));
  	return obs_geom.intersects(line_segment).size() > 0;
  	}
  else if (obs.type == obstacle_type::BOX){
   	h2d::Segment line_segment(h2d::Point2d(x1, y1), h2d::Point2d(x2, y2));
  	h2d::CPolyline obs_geom(obs_to_cpoly(obs));
  	return obs_geom.intersects(line_segment).size() > 0;
  	}
  else return false;


}

/**
 * @brief Function that checks if a line crosses any obstacle in a vector
 * 
 * @param x1, y1 The starting point of the line segment
 * @param x2, y2 The ending point of the line segment
 * @param obstacles The vector of obstacles to check against
 * @return true If the line crosses any obstacle in the vector
 * @return false If the line does not cross any obstacle in the vector
 */
bool line_overlap(double x1, double y1, double x2, double y2, const std::vector<obstacle>& obstacles) {
  for (const auto& obs : obstacles) {
    if (overlaps(x1, y1, x2, y2, obs)) {
      return true;
    }
  }
  return false;
}



/**
 * @brief It checks if the obstacle is inside the map
 * 
 * @param obs The obstacle to check
 * @param map The map to use
 * @param dx The x dimension of the map
 * @param dy The y dimension of the map
 * @return true If the obstacle is inside the map
 * @return false If the obstacle is not inside the map
 */
bool is_inside_map(obstacle obs, std::string map, double dx, double dy){
  bool inside = false;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
    "Checking obstacle %f, %f, %f, %f, %f, in map %s dx: %f  dy: %f", 
    obs.x, obs.y, obs.radius, obs.dx, obs.dy, map.c_str(), dx, dy
  );

  h2d::CPolyline map_poly;
  if (map == "rectangle"){
    obstacle tmp = {0.0, 0.0, 0.0, dx, dy, obstacle_type::BOX};
    map_poly = obs_to_cpoly(tmp);
  }
  else if (map == "hexagon"){
    std::vector<h2d::Point2d> vertexes;
    vertexes.push_back(h2d::Point2d(    -dx,             0.0));
    vertexes.push_back(h2d::Point2d(-dx/2.0, -dx*sqrt(3)/2.0));
    vertexes.push_back(h2d::Point2d( dx/2.0, -dx*sqrt(3)/2.0));
    vertexes.push_back(h2d::Point2d(     dx,             0.0));
    vertexes.push_back(h2d::Point2d( dx/2.0,  dx*sqrt(3)/2.0));
    vertexes.push_back(h2d::Point2d(-dx/2.0,  dx*sqrt(3)/2.0));
    map_poly = h2d::CPolyline(vertexes);
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Map %s not recognized.", map.c_str());
    inside = false;
  }

  if (obs.type == obstacle_type::CYLINDER){
    h2d::Circle obs_circle(obs_to_circle(obs));
    if (obs_circle.isInside(map_poly) && obs_circle.intersects(map_poly).size() == 0) {
      inside = true;
    }
  }
  else if (obs.type == obstacle_type::BOX){
    h2d::CPolyline obs_rect = obs_to_cpoly(obs);
    if (obs_rect.isInside(map_poly) && obs_rect.intersects(map_poly).size() == 0) {
      inside = true;
    }
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Obstacle type %d not recognized.", obs.type);
    inside = false;
  }  

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Result: %s", (inside ? "inside" : "NOT inside"));
  return inside;
}


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

/**
 * @brief Checks if the obstacle overlaps with any other obstacle in the vector
 * 
 * @param obs The obstacle to check
 * @param obstacles The vector of obstacles
 * @param map The type of the map (rectangle or hexagon)
 * @param dx x dimension of the map
 * @param dy y dimension of the map
 * @return true If the obstacle does not overlap and is inside the map
 * @return false If the obstacle overlaps or is outside the map
 */
bool valid_position(
  std::string map, double dx, double dy,
  const obstacle & obs, std::vector<std::vector<obstacle>> others 
)
{
  bool res = is_inside_map(obs, map, dx, dy);
  for (auto other : others) {
    res &= !overlaps(obs, other);
  }
  return res;
}

std::vector<obstacle> msg_to_obstacles(obstacles_msgs::msg::ObstacleArrayMsg msg) {
		std::vector<obstacle> obs;
		for (auto m : msg.obstacles) {
			obstacle_type ty;
			if (m.type == "CYLINDER") {
				ty = CYLINDER;
			} else if (m.type == "BOX") {
				ty = BOX;
			}
			obstacle o = {m.radius, m.x, m.y, m.dx, m.dy, ty};
			obs.push_back(o);
		}
		return obs;
	}

    planning_msgs::msg::Roadmap createGraphEdges(const std::vector<obstacle> &possible_waypoints, const std::vector<obstacle> &obstacles)
    {
        planning_msgs::msg::Roadmap roadmap;

        std::vector<double> coords;

        for (const auto& waypoint : possible_waypoints)
        {
          geometry_msgs::msg::Point point;

          point.x = waypoint.x;
          point.y = waypoint.y;
          point.z = 0.0;
          roadmap.nodes.push_back(point);
          coords.push_back(waypoint.x);
          coords.push_back(waypoint.y);
        }

        delaunator::Delaunator delaunator(coords);
        const auto &triangles = delaunator.triangles;
        
        // Creating empty first
        for (size_t i = 0; i < roadmap.nodes.size(); i++)
        {
          planning_msgs::msg::RoadmapEdge empty_edge;
          roadmap.edges.push_back(empty_edge);
        }

        for (size_t i = 0; i < triangles.size(); i += 3)
	      {
            for (size_t j = 0; j < 3; ++j)
            {
              //check if it crosses an obstacle
              int id_starting = triangles[i + j];
              int id_ending = triangles[i + (j + 1) % 3];
              float x1 = roadmap.nodes[id_starting].x;
              float y1 = roadmap.nodes[id_starting].y;
              float x2 = roadmap.nodes[id_ending].x;
              float y2 = roadmap.nodes[id_ending].y;
              
              if (!line_overlap(x1, y1, x2, y2, {obstacles})) //|| (x1 == 0.0 && y1 == 0.0) || (x2 == 0.0 && y2 == 0.0))
              {
                  // Check if the edge doesn't already exist
                  if (std::find(roadmap.edges[id_starting].node_ids.begin(),
                        roadmap.edges[id_starting].node_ids.end(), id_ending) ==
                      roadmap.edges[id_starting].node_ids.end())
                  {
                      // Add connection from starting node to ending node
                      roadmap.edges[id_starting].node_ids.push_back(id_ending);

                      // Add connection from ending node to starting node
                      roadmap.edges[id_ending].node_ids.push_back(id_starting);
                  }
              }
            }

		    }

        return roadmap;
    }
