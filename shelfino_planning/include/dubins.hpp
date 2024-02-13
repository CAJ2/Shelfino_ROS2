#ifndef DUBINSCURVES
#define DUBINSCURVES

#include "utils.hpp"
#include <vector>
#include <iostream>
#include <cmath>

namespace dubins
{
   struct DubinsLine
   {
      double x, y, th;

      DubinsLine(double s, double x0, double y0, double th0, double k)
      {
         double tmp = (s * sinc(k * s / 2.0) * sin(th0 + ((k * s) / 2)));
         double xtmp = y0 + tmp;
         x = x0 + (s * sinc(k * s / 2.0) * cos(th0 + ((k * s) / 2)));
         y = y0 + (s * sinc(k * s / 2.0) * sin(th0 + ((k * s) / 2)));
         th = mod2pi(th0 + (k * s));
      }
   };

   // Structure representing an arc of a Dubins curve (straight or circular)
   struct DubinsArc
   {
      double x0, y0, th0, k, L;
      DubinsLine *dubins_line;

      DubinsArc(double i_x0, double i_y0, double i_th0, double i_k, double i_L)
      {
         dubins_line = new DubinsLine(i_L, i_x0, i_y0, i_th0, i_k);
         x0 = i_x0;
         y0 = i_y0;
         th0 = i_th0;
         k = i_k;
         L = i_L;
      }

      ~DubinsArc()
      {
         delete dubins_line;
      }
   };

   struct DubinsCurve
   {
      DubinsArc *a1;
      DubinsArc *a2;
      DubinsArc *a3;
      double L;

      DubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2)
      {
         a1 = new DubinsArc(x0, y0, th0, k0, s1);
         a2 = new DubinsArc(a1->dubins_line->x, a1->dubins_line->y, a1->dubins_line->th, k1, s2);
         a3 = new DubinsArc(a2->dubins_line->x, a2->dubins_line->y, a2->dubins_line->th, k2, s3);

         L = a1->L + a2->L + a3->L;
      }

      ~DubinsCurve()
      {
         delete a1;
         delete a2;
         delete a3;
      }
   };

   struct ParametersResult
   {
      double scaled_th0, scaled_thf, scaled_k_max, lambda;

      ParametersResult(double scaled_th0, double scaled_thf, double scaled_k_max, double lambda) : scaled_th0(scaled_th0), scaled_thf(scaled_thf), scaled_k_max(scaled_k_max), lambda(lambda) {}
   };

   struct CurveSegmentsResult
   {
      bool ok;
      double s1, s2, s3;

      CurveSegmentsResult(bool ok, double s1, double s2, double s3) : ok(ok), s1(s1), s2(s2), s3(s3) {}
   };

   class Dubins
   {
   private:
      /**
       * @brief The total possible curves to consider when solving a dubins problem
       * 
       */
      const int TOTAL_POSSIBLE_CURVES = 10;

      /**
       * @brief The types of curves to consider when solving a dubins problem
       * 
       */
      enum possible_curves
      {
         LSL,
         RSR,
         LSR,
         RSL,
         RLR,
         LRL,
         LS,
         RS,
         SL,
         SR
      };

      /**
       * @brief The curvature signs, where each element corresponds to one of the possible_curves
       * 
       */
      const int ksigns[6][3] = {
          {1, 0, 1},
          {-1, 0, -1},
          {1, 0, -1},
          {-1, 0, 1},
          {-1, 1, -1},
          {1, -1, 1},
      };

      // All the possible angles handled by the solution of the multipoint dubins problem
      const double multipointAngles[8] = {0, M_PI / 4, M_PI / 2, 3.0 / 4 * M_PI, M_PI, 5.0 / 4 * M_PI, 3.0 / 2 * M_PI, 7.0 / 4 * M_PI};

      // Bound on maximum path curvature
      double k_max = 1;

      double discritizer_size = 0.005; //[m]

      bool checkValidity(CurveSegmentsResult *curve_segments_result, double k0, double k1, double k2, double th0, double thf);

      ParametersResult *scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf);

      CurveSegmentsResult *scaleFromStandard(double lambda, CurveSegmentsResult *curve_segments);

      CurveSegmentsResult *useLSL(double scaled_th0, double scaled_thf, double scaled_k_max);
      CurveSegmentsResult *useRSR(double scaled_th0, double scaled_thf, double scaled_k_max);
      CurveSegmentsResult *useLSR(double scaled_th0, double scaled_thf, double scaled_k_max);
      CurveSegmentsResult *useRSL(double scaled_th0, double scaled_thf, double scaled_k_max);
      CurveSegmentsResult *useRLR(double scaled_th0, double scaled_thf, double scaled_k_max);
      CurveSegmentsResult *useLRL(double scaled_th0, double scaled_thf, double scaled_k_max);

      double *multipointShortestPathAngles(DubinsPoint **points, unsigned int numberOfPoints, std::vector<dubins::Edge> &edges);

   public:
      Dubins() = default;

      explicit Dubins(double k_max, double discritizer_size);

      DubinsCurve *findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf);

      DubinsCurve *findShortestPathCollisionDetection(double x0, double y0, double th0, double xf, double yf, double thf, std::vector<dubins::Edge> &edges);

      DubinsCurve **multipointShortestPath(DubinsPoint **points, unsigned int numberOfPoints, std::vector<dubins::Edge> &edges);

      bool intersCircleLine(DubinsPoint circleCenter, double r, DubinsPoint point1, DubinsPoint point2, std::vector<DubinsPoint> &pts, std::vector<double> &t);

      bool intersArcLine(DubinsArc *arc, DubinsPoint point1, DubinsPoint point2, std::vector<DubinsPoint> &pts, std::vector<double> &t);

      bool intersLineLine(DubinsPoint p1, DubinsPoint p2, DubinsPoint p3, DubinsPoint p4, std::vector<DubinsPoint> &pts, std::vector<double> &ts);
   };

}

#endif