#ifndef _LINE_SEGMENT_HPP_
#define _LINE_SEGMENT_HPP_


#include <cmath>
#include <iostream>

#include "utils.hpp"


namespace geom3d
{


class Point;  // forward declaration
class Plane;
class Line;
class Polygon;


class LineSegment
{
public:
    LineSegment(const Point &pt1, const Point &pt2)
        : pt1_(const_cast<Point&>(pt1))
        , pt2_(const_cast<Point&>(pt2))
    {
        if (almost_equal(pt1.distance_to(pt2)))
          {
            std::cout << "WARNING: "
                      << "LineSegment(const Point &pt1, const Point &pt2): "
                      << "points are very close\n";
            throw "Bad LineSegment constructor arguments!";
          }
    };

    double dx() const;
    double dy() const;
    double dz() const;
    Point pt1() const;
    Point pt2() const;

    double length() const;

    double distance_to(const Point &pt) const;
    double distance_to(const LineSegment &ls) const;
    double distance_to(const Line &l) const;
    double distance_to(const Plane &pl) const;
    double distance_to(const Polygon &poly) const;

private:
    Point &pt1_, &pt2_;
};


}  //ns geom3d


#endif //  _LINE_SEGMENT_HPP_ (line_segment.hpp include guard)
