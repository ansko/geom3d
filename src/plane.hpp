#ifndef _PLANE_HPP_
#define _PLANE_HPP_


#include <iostream>
#include <cmath>


namespace geom3d
{


class Point;  // forward declarations
class LineSegment;
class Line;
class Polygon;


class Plane
{
public:
    Plane(const double a, const double b, const double c, const double d)
        : a_(a)
        , b_(b)
        , c_(c)
        , d_(d)
    {
        if(almost_equal(a) && almost_equal(b) && almost_equal(c))
          {  // Incorrect parameters!
            std::cerr << "ERROR: "
                      << "Plane(): a, b and c may not be 0 simultaneously!\n";
            throw "Bad Plane constructor arguments!";
          }
    };
    Plane(const Point &pt1, const Point &pt2, const Point &pt3);

    double a() const;
    double b() const;
    double c() const;
    double d() const;

    double distance_to(const Point &pt) const;
    double distance_to(const LineSegment &ls) const;
    double distance_to(const Line &l) const;
    double distance_to(const Plane &pl) const;
    double distance_to(const Polygon &poly) const;

private:
    double a_, b_, c_, d_;
};


}  // ns geom3d


#endif  // _PLANE_HPP_ (plane.hpp include guard)
