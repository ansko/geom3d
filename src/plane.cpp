#include "point.hpp"
#include "line_segment.hpp"
#include "line.hpp"
#include "plane.hpp"
#include "polygon.hpp"

#include "utils.hpp"


namespace geom3d
{


Plane::Plane(const Point &pt1, const Point &pt2, const Point &pt3)
{
    if (almost_equal(pt1.distance_to(LineSegment(pt2, pt3)))
        || almost_equal(pt2.distance_to(LineSegment(pt1, pt3)))
        || almost_equal(pt3.distance_to(LineSegment(pt2, pt1))))
      {
        std::cerr << "Plane(): "
                  << "ERROR: all points belong to a single line!\n";
         throw "Bad Plane constructor arguments!";
      }
    double x1 = pt1.x(), y1 = pt1.y(), z1 = pt1.z(), // pt1 coords
           x2 = pt2.x(), y2 = pt2.y(), z2 = pt2.z(),
           x3 = pt3.x(), y3 = pt3.y(), z3 = pt3.z(),
           dx1 = x2-x1, dy1 = y2-y1, dz1 = z2-z1, // vector 1 -> 2
           dx2 = x3-x1, dy2 = y3-y1, dz2 = z3-z1; // vector 1 -> 3
    a_ = dy1*dz2 - dy2*dz1;
    b_ = dx2*dz1 - dx1*dz2;
    c_ = dx1*dy2 - dx2*dy1;
    d_ = -a_*x1 - b_*y1 - c_*z1;
}


double
Plane::a() const
{
    return a_;
}

double
Plane::b() const
{
    return b_;
}

double
Plane::c() const
{
    return c_;
}

double
Plane::d() const
{
    return d_;
}


double
Plane::distance_to(const Point &pt) const
{
    double d = -a_*pt.x() - b_*pt.y() - c_*pt.z();
    return std::abs(d_ - d);
}

double
Plane::distance_to(const LineSegment &ls) const
{
    double d1 = -a_*ls.pt1().x() - b_*ls.pt1().y() - c_*ls.pt1().z(),
           d2 = -a_*ls.pt2().x() - b_*ls.pt2().y() - c_*ls.pt2().z();

    if (d1*d2 < 0)
      { // Different signs mean that ls crosses *this
        return 0.;
      }
    if (std::abs(d1-d_) < std::abs(d2-d_))
      {
        return  std::abs(d1-d_);
      }
    return std::abs(d2-d_);
}

double
Plane::distance_to(const Line &l) const
{
    double x1o = l.pt1().x(), y1o = l.pt1().y(), z1o = l.pt1().z(),
           x2o = l.pt2().x(), y2o = l.pt2().y(), z2o = l.pt2().z(),
           dx = x2o-x1o, dy = y2o-y1o, dz=z2o-z1o,
           scalar_product = a_*dx + b_*dy + c_*dz;

    if (!almost_equal(scalar_product))
      { // If normal is not perpendicular to line, line crosses *this
        return 0.;
      }
    return distance_to(l.pt1());
}

double
Plane::distance_to(const Plane &pl) const
{
    if (!almost_equal(a_, pl.a()) || !almost_equal(b_, pl.b())
        || !almost_equal(c_, pl.c()))
      {
        return 0.;
      }
    return std::abs(d_ - pl.d());
}

double
Plane::distance_to(const Polygon &poly) const
{
    return poly.distance_to(*this);
}

}  // ns geom3d
