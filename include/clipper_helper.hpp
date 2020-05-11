/*
 * Clipper Library Bridge/Helper
 *
 * This header contains functions used to inflate/deflate Polygon objects from
 * the AppliedRoboticsEnvironment(*). Functions act as a bridge for the Clipper
 * library(**) that uses its own types and classes.
 *
 * (*)  https://github.com/ValerioMa/AppliedRoboticsEnvironment/blob/master/src/9_project_interface/include/utils.hpp
 * (**) http://www.angusj.com/delphi/clipper.php
 * Author: Domenico Stefani
 * Date: 11/05/2020
*/
#pragma once

#include "clipper.hpp"

namespace ClipperHelper {

const int clipperScale = 10000;

//------------------------------------------------------------------------------
// Clipper Lib conversion utilities
//------------------------------------------------------------------------------

/**
 * Convert Point objects into ClipperLib::IntPoint
 * This converts Point objects (*** read below) into IntPoint
 * objects. The Clipper Library uses integer values for numeric stability so
 * float values have to be scaled with an appropriate scaling factor that
 * accounts for the desired accuracy.
 *
 * (***) struct from https://github.com/ValerioMa/AppliedRoboticsEnvironment/blob/master/src/9_project_interface/include/utils.hpp
 *
 * @param pt point to convert
 * #return Converted IntPoint
*/
ClipperLib::IntPoint Point2CIntPoint(const Point& pt) {
    return ClipperLib::IntPoint((ClipperLib::cInt)(pt.x * clipperScale),
                                (ClipperLib::cInt)(pt.y * clipperScale));
}

/**
 * Convert ClipperLib::IntPoint to Point
 * This converts ClipperLib::IntPoint objects into Point objects (*** read
 * below) by applying a scale factor (equivalent to the factor applied during
 * the inverse tranformation).
 *
 * (***) struct from https://github.com/ValerioMa/AppliedRoboticsEnvironment/blob/master/src/9_project_interface/include/utils.hpp
 *
 * @param pt point to convert
 * @return converted point
*/
Point CIntPoint2Point(const ClipperLib::IntPoint& pt) {
    return Point((float)pt.X /clipperScale,
                 (float)pt.Y /clipperScale);
}

/**
 * Convert Polygon into ClipperLib::Path
 * This converts Polygon objects (*** read below) to ClipperLib::Path objects.
 *
 * (***) struct from https://github.com/ValerioMa/AppliedRoboticsEnvironment/blob/master/src/9_project_interface/include/utils.hpp
 *
 * @param poly polygon to convert
 * @return converted path
*/
ClipperLib::Path Polygon2CPath(const Polygon& poly) {
    ClipperLib::Path path;
    for(const Point& p : poly)
        path.push_back(Point2CIntPoint(p));
    assert(poly.size() == path.size());
    return path;
}

/**
 * Convert ClipperLib::Path into Polygon
 * This converts ClipperLib::Path objects to Polygon objects (*** read below).
 *
 * (***) struct from https://github.com/ValerioMa/AppliedRoboticsEnvironment/blob/master/src/9_project_interface/include/utils.hpp
 *
 * @param path path to convert
 * @return converted polygon
*/
Polygon CPath2Polygon(const ClipperLib::Path& path) {
    Polygon poly;
    for(ClipperLib::IntPoint p : path)
        poly.push_back(CIntPoint2Point(p));
    assert(poly.size() == path.size());
    return poly;
}

/**
 * Inflater method
 * This inflates (or deflates) a Polygon (*** read below) of a specified amount
 * using the Clipper library.
 *
 * (***) struct from https://github.com/ValerioMa/AppliedRoboticsEnvironment/blob/master/src/9_project_interface/include/utils.hpp
 *
 * @param poly polygon to inflate
 * @param amount inflation magnitude/size
 * @return inflated polygon
*/
Polygon inflateWithClipper(const Polygon& poly, float amount) {
    double delta = amount * clipperScale;  // scale the inflation amount

    ClipperLib::Path toInflate = Polygon2CPath(poly);  // convert to Clipper path
    ClipperLib::Paths inflated;
    ClipperLib::ClipperOffset co;
    co.AddPath(toInflate, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
    co.Execute(inflated, delta);
    assert(inflated.size() == 1);

    return CPath2Polygon(inflated[0]);
}

/**
 * Polygon vector inflater method
 * This inflates (or deflates) a vector of Polygon objects (*** read below) of a
 * specified amount using the Clipper library.
 *
 * (***) struct from https://github.com/ValerioMa/AppliedRoboticsEnvironment/blob/master/src/9_project_interface/include/utils.hpp
 *
 * @param polygons vector of polygons to inflate
 * @param amount inflation magnitude/size
 * @return vector of inflated polygons
*/
std::vector<Polygon> inflatePolygons(const std::vector<Polygon>& polygons, float amount) {
    std::vector<Polygon> res;
    for(Polygon poly : polygons)
        res.push_back(ClipperHelper::inflateWithClipper(poly,amount));
    return res;
}

} //namespace ClipperHelper