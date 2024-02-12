#ifndef UTILS
#define UTILS

#define _USE_MATH_DEFINES
#include <vector>
#include <cstddef>

#include "planning_msgs/msg/point2_d.hpp"

namespace dubins
{
  struct DubinsPoint
  {
    double x, y, th;

    DubinsPoint(double x = -1, double y = -1, double th = -1) : x(x), y(y), th(th) {}
  };

  class Edge
    {
    public:
        planning_msgs::msg::Point2D p1;

        planning_msgs::msg::Point2D p2;

        Edge(planning_msgs::msg::Point2D point1, planning_msgs::msg::Point2D point2): 
             p1(point1) , p2(point2) {}
    };

  double sinc(double t);

  double mod2pi(double angle);

  double rangeSymm(double angle);

  double crossProduct(DubinsPoint a, DubinsPoint b);

  double dot2D(DubinsPoint a, DubinsPoint b);

  int getOrientation(planning_msgs::msg::Point2D p, planning_msgs::msg::Point2D q, planning_msgs::msg::Point2D r);
}

#endif