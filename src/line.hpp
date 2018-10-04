#ifndef _LINE_HPP_
#define _LINE_HPP_


#include <cmath>


namespace geom3d
{


class Point;  // forward declarations
class LineSegment;
class Line;
class Polygon;


class Line
{
public:
    Line(const Point &pt1, const Point &pt2)
        : pt1_(const_cast<Point &>(pt1))
        , pt2_(const_cast<Point &>(pt2))
    { };

    Point pt1() const;
    Point pt2() const;

    double distance_to(const Point &pt) const;
    double distance_to(const LineSegment &ls) const;
    double distance_to(const Line &l) const;
    double distance_to(const Plane &pl) const;
    double distance_to(const Polygon &poly) const;

private:
    Point &pt1_, &pt2_;
};


}  // ns geom3d


#endif  // _LINE_HPP_ (line.hpp include guard)
