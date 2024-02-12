#include "utils.hpp"

#include <cmath>

#include "planning_msgs/msg/point2_d.hpp"


namespace dubins
{
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


    // Normalize an angle (in range [0,2*pi))
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

    // Normalize an angular difference (range (-pi, pi])
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

    double crossProduct(DubinsPoint a, DubinsPoint b)
    {
        return a.x * b.y - a.y * b.x;
    }

    double dot2D(DubinsPoint a, DubinsPoint b)
    {
        return a.x * b.x + a.y * b.y;
    }

    int getOrientation(planning_msgs::msg::Point2D p, planning_msgs::msg::Point2D q, planning_msgs::msg::Point2D r)
    {
        double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

        if (val == 0)
            return 0; // collinear
        return (val < 0) ? 1 : -1; // clock or counterclock wise
    }
}