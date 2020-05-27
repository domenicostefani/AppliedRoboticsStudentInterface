#include "dubins.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits>
#include <vector>
#include <assert.h>

#include <iostream>

namespace dubins{

/*!
* Pointer to maneuver funciton
*/
typedef bool(*maneuver)(double, double, double, double&, double&, double&);

void
circline(double s, double x0, double y0, double th0, double k, double& x,
         double& y, double& th);

double mod2pi(double ang);

double sinc(double t);

///
/// Class Methods
///

/*!
* Set all class fields values at once
*/

/*
void Arc::set(double x0, double y0, double th0, double k, double L) {
  this->x0 = x0;
  this->y0 = y0;
  this->th0 = th0;
  this->k = k;
  this->L = L;
  circline(L, x0, y0, th0, k, xf, yf, thf);
}
*/

void Arc::set(double x0, double y0, double th0, double k, double L) {
    this->x0 = x0;
    this->y0 = y0;
    this->th0 = th0;
    this->k = k;
    this->L = L;
    this->xf = this->x0 + this->L * sinc(this->k *  this->L / 2.0) * cos(th0 + k * this->L / 2);
    this->yf = this->y0 + this->L * sinc(this->k *  this->L / 2.0) * sin(th0 + k * this->L / 2);
    this->thf = mod2pi(this->th0 + this->k * this->L);
}

/*!
* Curve constructor 1
*/

Curve::Curve() {
    a1.set(0.0,0.0,0.0,0.0,0.0);
    a2.set(a1.xf, a1.yf, a1.thf, 0.0, 0.0);
    a3.set(a2.xf, a2.yf, a2.thf, 0.0, 0.0);
    L = a1.L + a2.L + a3.L;
}

/*!
* Curve constructor 2
*/

Curve::Curve(double x0, double y0, double th0, double s1, double s2,
           double s3, double k0, double k1, double k2) {
    a1.set(x0, y0, th0, k0, s1);
    a2.set(a1.xf, a1.yf, a1.thf, k1, s2);
    a3.set(a2.xf, a2.yf, a2.thf, k2, s3);
    L = a1.L + a2.L + a3.L;
}

///
/// Auxiliary functions
///

/*
* Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t
* otherwise
*/
double sinc(double t) {
    double s = (double) 0.0;
    if (fabs(t) < 0.002) { // for small values of t use Taylor series approx.
        s = 1 - pow(t, 2.0) / 6.0 * (1 - pow(t, 2.0) / 20.0);
    } else {
        s = sin(t) / t;
    }
    return s;
}

/*
* Normalize an angle (in range [0,2*pi))
*/
double mod2pi(double ang) {
    double out = ang;
    while (out < 0.0)
        out = out + 2.0 * M_PI;
    while (out >= 2.0 * M_PI)
        out = out - 2.0 * M_PI;
    return out;
}

/*
* Normalize an angular difference (range (-pi, pi])
*/
double rangeSymm(double ang) {
    double out = ang;
    while (out <= -M_PI)
        out = out + 2.0 * M_PI;
    while (out > M_PI)
        out = out - 2.0 * M_PI;
    return out;
}

///
/// Validity check functions
///

/*
* Check validity of a solution by evaluating explicitly the 3 equations
* defining a Dubins problem (in standard form)
*/
bool check(double s1, double k0, double s2, double k1, double s3, double k2,
           double th0, double thf) {
    double x0 = -1.0;
    double y0 = 0.0;
    double xf = 1.0;
    double yf = 0.0;

    double eq1 = x0 + s1 * sinc((1 / 2.) * k0 * s1) * cos(th0 + (1 / 2.) * k0 * s1) +
        s2 * sinc((1 / 2.) * k1 * s2) * cos(th0 + k0 * s1 + (1 / 2.) * k1 * s2) +
        s3 * sinc((1 / 2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - xf;
    double eq2 = y0 + s1 * sinc((1 / 2.) * k0 * s1) * sin(th0 + (1 / 2.) * k0 * s1) +
        s2 * sinc((1 / 2.) * k1 * s2) * sin(th0 + k0 * s1 + (1 / 2.) * k1 * s2) +
        s3 * sinc((1 / 2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - yf;
    double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

    bool Lpos = (s1 > 0.) || (s2 > 0.) || (s3 > 0.);

    double thres = 1.e-10;  // threshold for validity check

    #ifdef DEBUG_VERBOSE
        printf("eq1: %f\n",eq1);
        printf("eq2: %f\n",eq2);
        printf("eq3: %f\n",eq3);
        printf("check: %f\n",sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3));
        printf("Thres: %f\n", thres);
        printf("Lpos: %s\n",Lpos?"T":"F");
    #endif

    return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < thres) && Lpos;
}

///
/// Transform functions
///

/*
* Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
*/
void
scaleToStandard(double x0, double y0, double th0, double xf, double yf,
                double thf, double Kmax, double& sc_th0, double& sc_thf,
                double& sc_Kmax, double& lambda) {
    // Find transform parameters
    double dx = xf - x0;
    double dy = yf - y0;
    double phi = atan2(dy, dx);
    lambda = hypot(dx, dy) / 2.;

    // scale and normalize angles and curvature
    sc_th0 = mod2pi(th0 - phi);
    sc_thf = mod2pi(thf - phi);
    sc_Kmax = Kmax * lambda;
}

/*
* Scale the solution to the standard problem back to the original problem
*/
void
scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3,
                    double& s1, double& s2, double& s3) {
    s1 = sc_s1 * lambda;
    s2 = sc_s2 * lambda;
    s3 = sc_s3 * lambda;
}

///
/// Manouver computation methods  (One for each possible solution)
///

/*
* LSL  (Left-Straight-Left soluton)
*/
bool LSL(double sc_th0, double sc_thf, double sc_Kmax,double& sc_s1,
         double& sc_s2, double& sc_s3) {

    bool ok;

    double invK = 1. / sc_Kmax;
    double C = cos(sc_thf) - cos(sc_th0);
    double S = 2. * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(C, S);
    sc_s1 = invK * mod2pi(temp1 - sc_th0);
    double temp2 = 2. + 4. * pow(sc_Kmax, 2) - 2. * cos(sc_th0 - sc_thf) + 4. *
                   sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if (temp2 < 0.) {
        ok = false;
        sc_s1 = 0.;
        sc_s2 = 0.;
        sc_s3 = 0.;
    }
    else {
        sc_s2 = invK * sqrt(temp2);
        sc_s3 = invK * mod2pi(sc_thf - temp1);
        ok = true;
    }
    return ok;
}

/*
* RSR  (Right-Straight-Right soluton)
*/
bool RSR(double sc_th0, double sc_thf, double sc_Kmax,
         double& sc_s1, double& sc_s2, double& sc_s3) {
    bool ok;

    double invK = 1. / sc_Kmax;
    double C = cos(sc_th0) - cos(sc_thf);
    double S = 2. * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(C, S);
    sc_s1 = invK * mod2pi(sc_th0 - temp1);
    double temp2 = 2. + 4. * pow(sc_Kmax, 2) - 2. * cos(sc_th0 - sc_thf) - 4. *
                   sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if (temp2 < 0) {
        ok = false;
        sc_s1 = 0.;
        sc_s2 = 0.;
        sc_s3 = 0.;
    }
    else {
        sc_s2 = invK * sqrt(temp2);
        sc_s3 = invK * mod2pi(temp1 - sc_thf);
        ok = true;
    }
    return ok;
}

/*
* LSR  (Left-Straight-Right soluton)
*/
bool LSR(double sc_th0, double sc_thf, double sc_Kmax,
         double& sc_s1, double& sc_s2, double& sc_s3) {
    bool ok;
    double invK = 1. / sc_Kmax;
    double C = cos(sc_th0) + cos(sc_thf);
    double S = 2. * sc_Kmax + sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(-C, S);
    double temp3 = 4. * pow(sc_Kmax, 2) - 2. + 2. * cos(sc_th0 - sc_thf) + 4. *
                   sc_Kmax * (sin(sc_th0) + sin(sc_thf));
    if (temp3 < 0.) {
        ok = false; sc_s1 = 0.; sc_s2 = 0.; sc_s3 = 0.;
    }
    else {
        sc_s2 = invK * sqrt(temp3);
        double temp2 = -atan2(-2., sc_s2 * sc_Kmax);
        sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
        sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
    }
    return ok;
}

/*
* RSL  (Right-Straight-Left soluton)
*/
bool RSL(double sc_th0, double sc_thf, double sc_Kmax,
         double& sc_s1, double& sc_s2, double& sc_s3) {
    bool ok;
    double invK = 1. / sc_Kmax;
    double C = cos(sc_th0) + cos(sc_thf);
    double S = 2. * sc_Kmax - sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(C, S);
    double temp3 = 4. * pow(sc_Kmax, 2) - 2. + 2. * cos(sc_th0 - sc_thf) - 4. *
                   sc_Kmax * (sin(sc_th0) + sin(sc_thf));
    if (temp3 < 0) {
        ok = false;
        sc_s1 = 0.;
        sc_s2 = 0.;
        sc_s3 = 0.;
    }
    else {
        sc_s2 = invK * sqrt(temp3);
        double temp2 = atan2(2., sc_s2 * sc_Kmax);
        sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
        sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
    }
    return ok;
}

/*
* RLR  (Right-Left-Right soluton)
*/
bool RLR(double sc_th0, double sc_thf, double sc_Kmax,
         double& sc_s1, double& sc_s2, double& sc_s3) {
    bool ok;
    double invK = 1. / sc_Kmax;
    double C = cos(sc_th0) - cos(sc_thf);
    double S = 2. * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6. - 4. * pow(sc_Kmax, 2) + 2. *
                   cos(sc_th0 - sc_thf) + 4. * sc_Kmax *
                   (sin(sc_th0) - sin(sc_thf)));
    if (fabs(temp2) > 1) {
        ok = false;
        sc_s1 = 0.;
        sc_s2 = 0.;
        sc_s3 = 0.;
    }
    else {
        sc_s2 = invK * mod2pi(2. * M_PI - acos(temp2));
        sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
        sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
        ok = true;
    }
    return ok;
}

/*
* LRL  (Left-Right-Left soluton)
*/
bool LRL(double sc_th0, double sc_thf, double sc_Kmax,
         double& sc_s1, double& sc_s2, double& sc_s3) {
    bool ok;
    double invK = 1. / sc_Kmax;
    double C = cos(sc_thf) - cos(sc_th0);
    double S = 2. * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6. - 4. * pow(sc_Kmax, 2) + 2. *
                    cos(sc_th0 - sc_thf) - 4. * sc_Kmax *
                    (sin(sc_th0) - sin(sc_thf)));
    if (fabs(temp2) > 1.) {
        ok = false;
        sc_s1 = 0.;
        sc_s2 = 0.;
        sc_s3 = 0.;
    }
    else {
        sc_s2 = invK * mod2pi(2. * M_PI - acos(temp2));
        sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
        sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));
        ok = true;
    }
    return ok;
}

/*
* Solve the Dubins problem for the given input parameters.
* Return the type and the parameters of the optimal curve
*/
Curve
dubins_shortest_path(double x0, double y0, double th0, double xf, double yf,
                     double thf, double Kmax, int& pidx) {
    // Compute params of standard scaled problem
    double sc_th0, sc_thf, sc_Kmax, lambda;
    scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax, sc_th0, sc_thf, sc_Kmax,
                    lambda);

    // Define the functions corresponding to the different primitives, and the
    // corresponding curvatue signs
    maneuver  LSLfunc = &LSL;
    maneuver  RSRfunc = &RSR;
    maneuver  LSRfunc = &LSR;
    maneuver  RSLfunc = &RSL;
    maneuver  RLRfunc = &RLR;
    maneuver  LRLfunc = &LRL;
    maneuver primitives[6] = { LSLfunc, RSRfunc, LSRfunc, RSLfunc, RLRfunc,
                               LRLfunc };

    short ksigns[6][3]{
                        { 1, 0, 1},  // LSL
                        {-1, 0,-1},  // RSR
                        { 1, 0,-1},  // LSR
                        {-1, 0, 1},  // RSL
                        {-1, 1,-1},  // RLR
                        { 1,-1, 1}  // LRL
    };

    // Try all the possible primitives, to find the optimal solution
    pidx = -1;  // set index to impossible value
    double L = std::numeric_limits<double>::infinity();
    double sc_s1 = 0.0, sc_s2 = 0.0, sc_s3 = 0.0; // current s values

    for (int i = 0; i < 6; ++i) {
        // current values
        double sc_s1_c, sc_s2_c, sc_s3_c;
        // call to the current primitive
        bool ok;
        ok = primitives[i](sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c);
        // Compute current path length as the sum of the length of all three arcs
        double Lcur = sc_s1_c + sc_s2_c + sc_s3_c;

        #ifdef DEBUG_VERBOSE
            // when in need to check correctness, this prints all the lengths of the
            // candidate paths (feasible)
            if (ok)
                printf("Candidate path (i:%d) Lcur %f\n", i, Lcur);
            fflush(stdout);
        #endif

        if (ok && Lcur < L) {
            // if the current solution is feasible and minimum, update minimum vals.
            L = Lcur;
            sc_s1 = sc_s1_c;
            sc_s2 = sc_s2_c;
            sc_s3 = sc_s3_c;
            pidx = i;
        }
    }

    #ifdef DEBUG_VERBOSE
        // when in need to check correctness, this prints the index of the best path
        printf("Best path is pidx:%i\n", pidx);
        fflush(stdout);
    #endif

    double s1 = 0, s2 = 0, s3 = 0;
    if (pidx > -1) {
        // If a feasible solution was found

        // Transform the solution to the problem in standard form to the solution
        // of the original problem (scale the lengths)
        scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);

        // Construct the Dubins curve object with the computed optimal parameters
        Curve res(x0, y0, th0, s1, s2, s3, ksigns[pidx][0] * Kmax,
                  ksigns[pidx][1] * Kmax, ksigns[pidx][2] * Kmax);

        // Check the correctness of the algorithm
        assert(check(sc_s1, ksigns[pidx][0] * sc_Kmax,
               sc_s2, ksigns[pidx][1] * sc_Kmax,
               sc_s3, ksigns[pidx][2] * sc_Kmax,
               sc_th0,
               sc_thf)
        );
        return res;
    }
    return Curve(); // in case no curve was found return empty curve
}

/*
* Evaluate an arc (circular or straight) composing a Dubins curve, at a
* given arc-length s
*/
void circline(double s, double x0, double y0, double th0, double k,
    double& x, double& y, double& th) {
    x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2.);
    y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2.);
    th = mod2pi(th0 + k * s);
}

Position::Position(double s, double x, double y, double th, double k) {
    this->s  = s;
    this->x  = x;
    this->y  = y;
    this->th = th;
    this->k  = k;
}

/*
* Returns a discrete point vector, sampling at every delta interval
* @param[in]  arc             arc object to discretize
* @param[in]  delta           interval at which to sample points
* @param[i/o] remainingDelta  carry
* @return                     std::vector of points along the arc
*/
std::vector<Position>
Arc::discretizeArc(double delta, double& remainingDelta, double& last_s,
                   bool add_endpoint) {
    std::vector<Position> res;

    /*
      This functions saves a point every "delta" interval along the Arc
      What happens when a particular delta does not divide perfectly the length
      of the arc?
      In that case, a variable "remainingDelta" is set to a value that indicates
      the length of the last interval that we were not able to include in the
      sampling.
      When the next arc is discretized, the process will start from a point that
      is not the start of the arc but one that is "delta"-far from the last
      point sampled from the previous arc.
    */

    if (remainingDelta == 0) {
        int nPoints = floor(this->L / delta);    // find the number of points required
        remainingDelta = this->L-((double)delta * nPoints);  // save the amount of delta left in the end

        for (int j = 0; j <= nPoints; ++j) {
            double xc,yc,thc;
            double s = delta * j;
            circline(s,this->x0,this->y0,this->th0,this->k,xc,yc,thc);
            Position current(s + last_s,xc,yc,thc,this->k);
            res.push_back(current);

            // In this case remainingDelta was 0 and now it's not 0
            // It will be considered by the next discretize

            //TODO: Remember that the last point is not the final point and we should remember the final points
            //TODO: also we should compute the sum of the s when adding a curve to a curve
        }
    } else {
        // remainingDelta is not 0, meaning that a previous discretization left a
        // bit of the curve out

        //Now the first delta is what remains to be sampled
        double firstdelta = delta - remainingDelta;
        // the number of points excludes the first one
        int nPoints = floor((this->L - firstdelta)/delta);

        remainingDelta = this->L-(firstdelta + (double)delta*nPoints);  // save the amount of delta left in the end

        // Deal with the other points
        for (int j = 0; j <= nPoints; ++j) {
            double xc,yc,thc;
            double s = firstdelta + (delta * j);
            circline(s,this->x0,this->y0,this->th0,this->k,xc,yc,thc);
            Position current(s + last_s,xc,yc,thc,this->k);
            res.push_back(current);
        }
    }

    if (add_endpoint) {
        double xc,yc,thc;
        double s = this->L;
        circline(s,this->x0,this->y0,this->th0,this->k,xc,yc,thc);
        Position current(s + last_s,xc,yc,thc,this->k);
        res.push_back(current);
    }


    last_s += this->L;
    return res;
}

std::vector<Position>
Curve::discretizeCurve(double delta, double& remainingDelta, double& last_s,
                       bool add_endpoint) {
    std::vector<Position> res;
    std::vector<Position> posA1 = this->a1.discretizeArc(delta,remainingDelta,last_s,false);
    std::vector<Position> posA2 = this->a2.discretizeArc(delta,remainingDelta,last_s,false);
    std::vector<Position> posA3 = this->a3.discretizeArc(delta,remainingDelta,last_s,add_endpoint);

    res = posA1;
    res.insert( res.end(), posA2.begin(), posA2.end() );
    res.insert( res.end(), posA3.begin(), posA3.end() );

    return res;
}

std::vector<Position> Curve::discretizeSingleCurve(double delta) {
    double remainingDelta = 0.0;
    double last_s = 0.0;
    return this->discretizeCurve(delta, remainingDelta, last_s, true);
}

} // namespace dubins
