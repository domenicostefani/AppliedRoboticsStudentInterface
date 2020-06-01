/** \file polygon_utils.hpp
 * @brief Utilities for polygon manipulation.
 *
 * polygon refers to the Polygon objects from the AppliedRoboticsEnvironment(*).

 * (*)  https://github.com/ValerioMa/AppliedRoboticsEnvironment/blob/master/src/9_project_interface/include/utils.hpp
 *
 * Date: 25/05/2020
*/
#pragma once

//! Polygon utilities
namespace PUtils {
    /** Finds the baricenter coordinates of a utils::Polygon.
     * @param polygon Input polygon.
     * @param cx Output x coordinate.
     * @param cy Output y coordinate.
    */
    void baricenter(const Polygon& polygon, double& cx, double& cy) {
        cx = 0;
        cy = 0;
        for (auto vertex: polygon) {
            cx += vertex.x;
            cy += vertex.y;
        }
        cx /= static_cast<double>(polygon.size());
        cy /=  static_cast<double>(polygon.size());
    }

    /** Finds the baricenter Point of a utils::Polygon.
     * @param polygon Input polygon.
     * @return The baricenter Point.
    */
    Point baricenter(const Polygon& polygon) {
        double cx = 0,cy = 0;
        baricenter(polygon,cx,cy);
        return Point(cx,cy);
    }

    //
    //  Polygon points reordering
    //

    /** Point comparison operator.
     * It's used to establish a less-than relationship that allows to order
     * points of a polygon in either clockwise or counter-clockwise.
     *
     * @param a first point
     * @param b second point
     * @param center baricenter of the polygon
     * @return true if a less than b
    */
    bool pointOrderComparison(Point a, Point b, Point center)
    {
        if (a.x - center.x >= 0 && b.x - center.x < 0)
            return true;
        if (a.x - center.x < 0 && b.x - center.x >= 0)
            return false;
        if (a.x - center.x == 0 && b.x - center.x == 0) {
            if (a.y - center.y >= 0 || b.y - center.y >= 0)
                return a.y > b.y;
            return b.y > a.y;
        }

        // compute the cross product of vectors (center -> a) x (center -> b)
        float det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
        if (det < 0)
            return true;
        if (det > 0)
            return false;

        // points a and b are on the same line from the center
        // check which point is closer to the center
        float d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
        float d2 = (b.x - center.x) * (b.x - center.x) + (b.y - center.y) * (b.y - center.y);
        return d1 > d2;
    }

    /** Sort points of a polygon in clockwise order.
     *
     * @param polygon the polygon which points have to be sorter
    */
    void sortPolygonPoints(Polygon& polygon) {
        Point polyCenter = PUtils::baricenter(polygon);
        // bubblesort
        bool swapped = true;
        while (swapped) {
            swapped = false;
            for (size_t i = 0; i< polygon.size()-1; ++i) {
                if (pointOrderComparison(polygon[i],polygon[i+1],polyCenter)) {
                    //Swap
                    Point temp = polygon[i];
                    polygon[i] = polygon[i+1];
                    polygon[i+1] = temp;
                    swapped = true;
                }
            }
        }
    }

    /** Project point P to the line passing between A and B.
     * @param pA first point of the line
     * @param pB second point of the line
     * @param pP point to project
     * @return the orthogonal projection point
    */
    Point projectPointToLine(Point pA, Point pB, Point pP) {

        if (abs(pB.x - pA.x) < 0.001)   // Vertical line
            return Point(pA.x, pP.y);
        else if (abs(pB.y - pA.y) < 0.001) // Horizontal line
            return Point(pP.x, pA.y);
        else{ // general line
            // Find line passing for A and B
            float mAB = (pB.y - pA.y) / (pB.x - pA.x);
            float cAB = pA.y - (mAB * pA.x);
            // Find line perpendicular line to the one for A and B, passing for P
            float mPR = -1.0 / mAB;
            float cPR = pP.y - (mPR * pP.x);
            // Find intersection
            Point res;
            res.x = (cAB - cPR) / (mPR - mAB);
            res.y = res.x * mPR + cPR;

            return res;
        }
    }

    /** Find the point in the vector that is nearest to A.
     *
     * @param pa point A
     * @param points vector of points
     * @return the points in the vector closer to pA
    */
    Point nearestPoint(Point pA, const std::vector<Point>& points) {
        float smallestDistance = std::numeric_limits<float>::max();
        Point res;
        for (Point pB : points) {
            float dist = pow(pA.y - pB.y ,2) + pow(pA.x - pB.x,2);
            if(dist < smallestDistance) {
                smallestDistance = dist;
                res = pB;
            }
        }
        return res;
    }

    /** return true if two poins have the same coordinates.
     *
     * @param pa first point
     * @param pa second point
     * @return true if two poins have the same coordinates
    */
    bool pointsEquals(Point pa, Point pb){
        return ((pa.x == pb.x)&&(pa.y == pb.y));
    }

}    // namespace PUtils
