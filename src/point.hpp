#ifndef _POINT_HPP_
#define _POINT_HPP_


#include <cmath>


namespace geom3d
{


class LineSegment;  // forward declaration
class Line;
class Plane;
class Polygon;


class Point
{
public:
    Point()
        : x_(0.)
        , y_(0.)
        , z_(0.)
    { };
    Point(const double x, const double y, const double z)
        : x_(const_cast<double&>(x))
        , y_(const_cast<double&>(y))
        , z_(const_cast<double&>(z))
    { };

    double x() const;
    double y() const;
    double z() const;

    void translate(const double dx, const double dy, const double dz);
    void rotate(const double around_ox, const double around_oy,
                const double around_oz);

    double distance_to(const Point &pt) const;
    double distance_to(const LineSegment &ls) const;
    double distance_to(const Line &l) const;
    double distance_to(const Plane &pl) const;
    double distance_to(const Polygon &poly) const;

private:
    double x_, y_, z_;
};


} // ns geom3d

#endif  // _POINT_HPP_ (point.hpp include guard)
