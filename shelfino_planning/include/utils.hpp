#ifndef UTILS
#define UTILS

#define _USE_MATH_DEFINES
#include <vector>
#include <cstddef>

namespace dubins
{
  /**
   * @brief A point structure, that can be used as:
   * - a simple 2D point (x,y)
   * - a point representing a position (x,y,theta)
   * 
   */
  struct DubinsPoint
  {
    double x, y, th;

    DubinsPoint(double x = -1, double y = -1, double th = -1) : x(x), y(y), th(th) {}
  };

  /**
   * @brief Implementation of function sinc(t)
   * 
   * @param t TODO
   * @return double 1 for t==0, and sin(t)/t otherwise
   */
  double sinc(double t);

  /**
   * @brief Normalize an angle (in range [0,2*pi))
   * 
   * @param ang Angle to normalize
   * @return double Normalized angle
   */
  double mod2pi(double angle);

  /**
   * @brief Normalize an angular difference (range (-pi, pi])
   * 
   * @param ang Angular difference to normalize
   * @return double Normalized angular difference
   */
  double rangeSymm(double angle);

  /**
   * @brief Calculates the cross product between two Points
   * 
   * @param a First input point
   * @param b Second input point
   * @return double Cross product between provided points
   */
  double crossProduct(DubinsPoint a, DubinsPoint b);

  /**
   * @brief Calculates the dot product between two Points
   * 
   * @param a First input point
   * @param b Second input point
   * @return double Dot product between provided points
   */
  double dot2D(DubinsPoint a, DubinsPoint b);

}

#endif