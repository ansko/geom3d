#ifndef _POLYGON_HPP_
#define _POLYGON_HPP_


#include <iostream>
#include <memory>
#include <vector>

#include "utils.hpp"


extern double pi;


namespace geom3d
{


class Point;  // forward declaration
class LineSegment;
class Line;
class Plane;


class Polygon
{
public:
    Polygon(const std::vector<std::shared_ptr<Point> > pt_ptrs)
        : pt_ptrs_(pt_ptrs)
    { };

    double square() const;
    Point center() const;
    std::vector<std::shared_ptr<Point> > vertices() const;

    double distance_to(const Point &pt) const;
    double distance_to(const LineSegment &ls) const;
    double distance_to(const Line &l) const;
    double distance_to(const Plane &pl) const;
    double distance_to(const Polygon &poly) const;

private:
    std::vector<std::shared_ptr<Point> > pt_ptrs_;
    double distance_to_in_plane_point(const Point &pt) const;
    bool contains_point(const Point &pt) const;
    bool contains_point(const double x, const double y, const double z) const;
};


}  // ns goem3d


#endif  // _POLYGON_HPP_ (polygon.hpp include guard)
