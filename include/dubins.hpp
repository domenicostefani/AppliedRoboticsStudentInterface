/*
  CHECK FOR TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  Library is not complete
*/

#pragma once

#include <vector>

namespace dubins{

  double mod2pi(double ang);

  class Position{
  public:
    double s;
    double x;
    double y;
    double th;
    double k;
    Position(double s, double x, double y, double th, double k);
  };

  /*!
  * Class representing one of the three arcs of every path of Dubin's maneuvers
  */
  class Arc {
  public:
    double x0, y0, th0;   // Initial position (x pos,y pos,yaw angle)
    double k;             // Arc curvature
    double L;             // Arc length
    double xf, yf, thf;   // Final position (x pos,y pos,yaw angle
    
    Arc() {};
    
    /*!
    * Set all class fields values at once
    * @param[in] x0        x position value of the start of the arc
    * @param[in] y0        y position value of the start of the arc
    * @param[in] th0       orientation angle of the start of the arc
    * @param[in] k         curvature of the arc
    * @param[in] L         arc length
    */
    
    void set(double x0, double y0, double th0, double k, double L);
    std::vector<Position> discretizeArc(double delta, double& remainingDelta, double& last_s, bool add_endpoint);
  };

  /*!
  * Class representing a Dubin's curve or maneuver, composed by three arcs
  */
  class Curve {
  public:
    Arc a1, a2, a3; // arcs
    double L;       // path length
    /*!
    * Class constructor
    * @param[in] x0        x position value of the start of the maneuver
    * @param[in] y0        y position value of the start of the maneuver
    * @param[in] th0       orientation angle of the start of the maneuver
    * @param[in] s1        length of the first arc                              // !TODO: Check if s are lengths or curvilinear abscissas
    * @param[in] s2        length of the second arc
    * @param[in] s3        length of the third arc
    * @param[in] k0        curvature of the first arc                           // !TODO: Check why this starts from zero and the s from 1
    * @param[in] k1        curvature of the second arc
    * @param[in] k2        curvature of the third arc
    */
    Curve(double x0, double y0, double th0, double s1, double s2, double s3,
          double k0, double k1, double k2);

    std::vector<Position> discretizeCurve(double delta, double& remainingDelta, double& last_s, bool add_endpoint);
    std::vector<Position> discretizeSingleCurve(double delta);
  };

  /*!
  * Compute the shortest path as a Dubin's maneuver.
  * @param[in]  x0        x position value of the start of the maneuver
  * @param[in]  y0        y position value of the start of the maneuver
  * @param[in]  th0       orientation angle of the start of the maneuver
  * @param[in]  xf        x position value of the start of the maneuver
  * @param[in]  yf        y position value of the start of the maneuver
  * @param[in]  thf       orientation angle of the start of the maneuver
  * @param[in]  Kmax      maximum curvature allowed
  * @param[out] pidx      index of the best maneuver
  * @return               the shortest path maneuver or curve
  */
  Curve
  dubins_shortest_path(double x0, double y0, double th0, double xf,
                       double yf, double thf, double Kmax, int& pidx);
}
