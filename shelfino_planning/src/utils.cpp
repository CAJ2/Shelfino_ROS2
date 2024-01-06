#include "utils.hpp"

#include <cmath>

namespace dubins
{

    /**
     * @brief Implementation of function sinc(t)
     * 
     * @param t TODO
     * @return double 1 for t==0, and sin(t)/t otherwise
     */
    double sinc(double t)
    {
        double s;
        if (abs(t) < 0.002)
        {
            s = 1 - pow(t, 2) / 6 * (1 - pow(t, 2) / 20);
        }
        else
        {
            s = sin(t) / t;
        }
        return s;
    }

    /**
     * @brief Normalize an angle (in range [0,2*pi))
     * 
     * @param ang Angle to normalize
     * @return double Normalized angle
     */
    double mod2pi(double ang)
    {
        double out = ang;
        while (out < 0)
        {
            out = out + 2 * M_PI;
        }
        while (out >= 2 * M_PI)
        {
            out = out - 2 * M_PI;
        }
        return out;
    }

    /**
     * @brief Normalize an angular difference (range (-pi, pi])
     * 
     * @param ang Angular difference to normalize
     * @return double Normalized angular difference
     */
    double rangeSymm(double ang)
    {
        double out = ang;
        while (out <= -M_PI)
        {
            out = out + 2 * M_PI;
        }
        while (out > M_PI)
        {
            out = out - 2 * M_PI;
        }
        return out;
    }

    /**
     * @brief Calculates the cross product between two Points
     * 
     * @param a First input point
     * @param b Second input point
     * @return double Cross product between provided points
     */
    double crossProduct(DubinsPoint a, DubinsPoint b)
    {
        return a.x * b.y - a.y * b.x;
    }

    /**
     * @brief Calculates the dot product between two Points
     * 
     * @param a First input point
     * @param b Second input point
     * @return double Dot product between provided points
     */
    double dot2D(DubinsPoint a, DubinsPoint b)
    {
        return a.x * b.x + a.y * b.y;
    }
}