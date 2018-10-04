#include "point.hpp"
#include "line.hpp"
#include "line_segment.hpp"
#include "plane.hpp"
#include "polygon.hpp"


namespace geom3d
{


double
Point::x() const
{
    return x_;
}


double
Point::y() const
{
    return y_;
}


double
Point::z() const
{
    return z_;
}


void
Point::translate(const double dx, const double dy, const double dz)
{
    x_ += dx;
    y_ += dy;
    z_ += dz;
}


void
Point::rotate(const double around_ox, const double around_oy,
              const double around_oz)
{
/*
Rotation matrices around:
   x:             y:            z:
1  0    0     cos 0 sin   cos -sin 0
0 cos -sin     0  1  0    sin  cos 0
0 sin  cos   -sin 0 cos    0    0  1
*/

    // Rotating around ox
    double angle = around_ox;
    double tmp_x = x_;
    double tmp_y = y_ * cos(angle) + z_ * sin(angle);
    double tmp_z = -y_ * sin(angle) + z_ * cos(angle);
    x_ = tmp_x;
    y_ = tmp_y;
    z_ = tmp_z;

    // Rotating around oy
    angle = around_oy;
    tmp_x = x_ * cos(angle) - z_ * sin(angle);
    tmp_y = y_;
    tmp_z = x_ * sin(angle) + z_ * cos(angle);
    x_ = tmp_x;
    y_ = tmp_y;
    z_ = tmp_z;

    // Rotating around oz
    angle = around_oz;
    tmp_x = x_ * cos(angle) + y_ * sin(angle);
    tmp_y = -x_ * sin(angle) + y_ * cos(angle);
    tmp_z = z_;
    x_ = tmp_x;
    y_ = tmp_y;
    z_ = tmp_z;
}


double
Point::distance_to(const Point &pt) const
{
    double dx = pt.x() - x_;
    double dy = pt.y() - y_;
    double dz = pt.z() - z_;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

double
Point::distance_to(const LineSegment &ls) const
{
    return ls.distance_to(*this);
}

double
Point::distance_to(const Line &l) const
{
    return l.distance_to(*this);
}

double
Point::distance_to(const Plane &pl) const
{
    return pl.distance_to(*this);
}

double
Point::distance_to(const Polygon &poly) const
{
    return poly.distance_to(*this);
}


}  // ns geom3d
