/** \file dubins.hpp
 * @brief Dubins Path Computation library.
*/
#pragma once

#include <vector>

//! Dubins curves computation methods
namespace dubins{

  double mod2pi(double ang);

  /** Path sample
   * Path sample position
  */
  class Position{
  public:
    double s;   ///< Abscissa
    double x;   ///< X coordinate of the point
    double y;   ///< Y coordinate of the point
    double th;  ///< Angle
    double k;   ///< Curvature

    /**
     * Set position values.
    */
    Position(double s, double x, double y, double th, double k);
  };

  /** Class representing one of the three arcs of every path of Dubin's maneuvers.
  */
  class Arc {
  public:
    double x0,    ///< Initial arc point, x coordinate
           y0,    ///< Initial arc point, y coordinate
           th0;   ///< Initial arc point, angle
    double k;     ///< Arc curvature
    double L;     ///< Arc length
    double xf,    ///< Final arc point, x coordinate
           yf,    ///< Final arc point, y coordinate
           thf;   ///< Final arc point, angle

    /**
     * Default constructor,
    */
    Arc() :
        x0(0), y0(0), th0 (0),
        k(0), L(0),
        xf(0), yf(0), thf(0)
    {};

    /** Set all class fields values at once.
    * @param[in] x0        x position value of the start of the arc
    * @param[in] y0        y position value of the start of the arc
    * @param[in] th0       orientation angle of the start of the arc
    * @param[in] k         curvature of the arc
    * @param[in] L         arc length
    */
    void set(double x0, double y0, double th0, double k, double L);

    /** Discretize a Dubins Arc.
     * Discretize a Dubins arc, sampling positions from the arc with fixed
     * distance delta.
     *
     * @param[in] delta sampling step size
     * @param[inout] remainingDelta remaining value of delta at the end of the arc, carry over
     * @param[inout] last_s last curvilinear abscissa value
     * @param[in] add_endpoint flag that states whether the last point must be added or not
     * @return vector of path samples (positions)
    */
    std::vector<Position> discretizeArc(double delta, double& remainingDelta, double& last_s, bool add_endpoint);
  };

  /** Class representing a Dubin's curve or maneuver, composed by three arcs.
  */
  class Curve {
  public:
    Arc a1, a2, a3; // arcs
    double L;       // path length

    /** Class constructor 1.
    */
    Curve();
    /** Class constructor 2.
    * @param[in] x0        x position value of the start of the maneuver
    * @param[in] y0        y position value of the start of the maneuver
    * @param[in] th0       orientation angle of the start of the maneuver
    * @param[in] s1        length of the first arc
    * @param[in] s2        length of the second arc
    * @param[in] s3        length of the third arc
    * @param[in] k0        curvature of the first arc
    * @param[in] k1        curvature of the second arc
    * @param[in] k2        curvature of the third arc
    */
    Curve(double x0, double y0, double th0, double s1, double s2, double s3,
          double k0, double k1, double k2);

    /** Discretize a Dubins Curve.
     * Discretize a Dubins curve, sampling positions from the curve with fixed
     * distance delta.
     *
     * @param[in] delta sampling step size
     * @param[inout] remainingDelta remaining value of delta at the end of the curve, carry over
     * @param[inout] last_s last curvilinear abscissa value
     * @param[in] add_endpoint flag that states whether the last curve point must be added or not
     * @return vector of path samples (positions)
    */
    std::vector<Position> discretizeCurve(double delta, double& remainingDelta, double& last_s, bool add_endpoint);

    /** Discretize a SINGLE Dubins Curve.
     * Discretize a single Dubins curve, sampling positions from the curve with
     * fixed distance delta.
     * The fact that it's a single curve means that there is no need to carry
     * over the delta remainder value or the last abscissa.
     * NOTE: Using this for a multicurve/multipoint causes inconsistency of
     * sample spacing at the end of each curve.
     *
     * @param[in] delta sampling step size
     * @return vector of path samples (positions)
    */
    std::vector<Position> discretizeSingleCurve(double delta);
  };

  /** Compute Dubins curve
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
