#include "point.hpp"
#include "line_segment.hpp"
#include "line.hpp"
#include "plane.hpp"
#include "polygon.hpp"

#include <iostream>


namespace geom3d
{


double
LineSegment::dx() const
{
    return pt2_.x() - pt1_.x();
}

double
LineSegment::dy() const
{
    return pt2_.y() - pt1_.y();
}

double
LineSegment::dz() const
{
    return pt2_.z() - pt1_.z();
}

Point
LineSegment::pt1() const
{
    return pt1_;
}

Point
LineSegment::pt2() const
{
    return pt2_;
}

double
LineSegment::length() const
{
    double dx = this->dx(),
           dy = this->dy(),
           dz = this->dz();
    return sqrt(dx*dx + dy*dy + dz*dz);
}


double
LineSegment::distance_to(const Point &pt) const
{
    // Calculate the square of a triangle formed by
    // pt1_(x1,y1,z1), pt2_(x2,y2,z2) and pt(x,y,z),
    // in two different ways.
    //
    // The height of this triangle equals to the distance of interest.
    // Used variables:
    //     v1(dx1, dy1, dz1) from pt to pt1_ of length len1
    //     v2(dx2, dy2, dz2) from pt to pt2_ of length len2

    double x = pt.x(), x1 = pt1_.x(), x2 = pt2_.x(),
           y = pt.y(), y1 = pt1_.y(), y2 = pt2_.y(),
           z = pt.z(), z1 = pt1_.z(), z2 = pt2_.z(),
           dx1 = x - x1, dx2 = x - x2, dx12 = x1 - x2,
           dy1 = y - y1, dy2 = y - y2, dy12 = y1 - y2,
           dz1 = z - z1, dz2 = z - z2, dz12 = z1 - z2,
           len1 = sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1),
           len2 = sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2),
           len = sqrt(dx12*dx12 + dy12*dy12 + dz12*dz12),
           scalar_product_12 = dx1*dx2 + dy1*dy2 + dz1*dz2;
    if (almost_equal(scalar_product_12))
      {  // Avoiding devision by 0 below
          if (len1 < len2)
            {
              return len1;
            }
          return len2;
      }
    double cos_angle_12 = scalar_product_12 / (len1 * len2),
           sin_angle_12 = sqrt(1 - cos_angle_12*cos_angle_12),
           square = 0.5 * len1 * len2 * sin_angle_12,
           height = square / 0.5 / len;

    // In case of obtuse angle near the triangle's base
    // return corresponding triangle's edge length not height
    double cos_angle_1_sign = -dx1*dx12 - dy1*dy12 - dz1*dz12,
           cos_angle_2_sign = dx2*dx12 + dy2*dy12 + dz2*dz12;
    if (cos_angle_1_sign < 0)
      {
        return len1;
      }
    else if (cos_angle_2_sign < 0)
      {
        return len2;
      }
    return height;
}


double
LineSegment::distance_to(const LineSegment &ls) const
{
    auto distance_to_a_from_end_of_b = [](double xa, double ya, double za,
                                          double xb, double yb, double zb)
           { // Function made to find a distance
             // to the vector a(xa, ya, za)
             // from the end of the vector b(xb, yb, zb)
             double len_a = sqrt(xa*xa + ya*ya + za*za),
                    len_b = sqrt(xb*xb + yb*yb + zb*zb),
                    cos_angle = (xa*xb + ya*yb + za*zb) / len_a / len_b,
                    sin_alpha = sqrt(1 - cos_angle*cos_angle);

             return len_b * sin_alpha;
           };


    double x1t = pt1_.x(), x2t = pt2_.x(),
           y1t = pt1_.y(), y2t = pt2_.y(),
           z1t = pt1_.z(), z2t = pt2_.z(),
           x1o = ls.pt1().x(), x2o = ls.pt2().x(),
           y1o = ls.pt1().y(), y2o = ls.pt2().y(),
           z1o = ls.pt1().z(), z2o = ls.pt2().z(),
           dxt = x2t - x1t, dyt = y2t - y1t, dzt = z2t - z1t,
           dxo = x2o - x1o, dyo = y2o - y1o, dzo = z2o - z1o;

    // Check whether some end-points of segments are the same
    if (almost_equal(pt1_.distance_to(ls.pt1()))
        || almost_equal(pt1_.distance_to(ls.pt2()))
        || almost_equal(pt2_.distance_to(ls.pt1()))
        || almost_equal(pt2_.distance_to(ls.pt2())))
      {
        return 0.0;
      }

    // Check whether segments are in a single plane.
    // It happens when cross-product of vectors A,B,C equals 0
    // A * (B x C) == 0, where vector are:
    //     A - from pt1_ to pt2_
    //     B - from pt1_ to ls.pt1()
    //     C - from pt1_ to ls.pt2()
    double xa = dxt, ya = dyt, za = dzt,
           xb = x1o - x1t, yb = y1o - y1t, zb = z1o - z1t,
           xc = x2o - x1t, yc = y2o - y1t, zc = z2o - z1t,
           mix_product = xa * (yb*zc - yc*zb) + ya * (-xb*zc + xc*zb)
                       + za * (xb*yc - xc*yb);

    if (almost_equal(mix_product))
      { // Yes, segments are in a single plane
        // Check whether segments are parallel
        // (this is here to avoid problems when checking cross
        // of segments by solving equations system)
        double product = dxt*dxo + dyt*dyo + dzt*dzo,
               len_t2 = (dxt*dxt + dyt*dyt + dzt*dzt),
               len_o2 = (dxo*dxo + dyo*dyo + dzo*dzo);
        if (almost_equal(product*product / len_t2 / len_o2, 1.0))
          { // Yes, segments are parallel
            return distance_to_a_from_end_of_b(xa, ya, za, xb, yb, zb);
          }
        else
          { // Check whether (surely not-parallel) segments cross
            // Carrying lines surely cross, but the cross-point
            // should belong to both vectors
            // x_cross = pt1_.x() + alpha * dxt, y_cross =...
            //         = ls.pt1() + beta * dxo, y_cross = ...
            //  got matrix for equations system:
            //      dxt,  -dxo  |  x1o-x1t
            //      dyt,  -dyo  |  y1o-y1t
            double det = dyt*dxo - dxt*dyo,
                   det_alpha = dxo*(y1o-y1t) - dyo*(x1o-x1t),
                   det_beta = dxt*(y1o-y1t) - dyt*(x1o-x1t),
                   alpha = det_alpha / det,
                   beta = det_beta / det;
            if (alpha >= 0. && alpha <= 1. && beta >= 0 && beta <= 1.)
              { // crossing point belongs to both vectors
                return 0.;
              }
            else
              { // segments do not cross
                return distance_to_a_from_end_of_b(xa, ya, za, xb, yb, zb);
              }
          }
      } // not in single plane

    // Segments are not in a single plane.
    // Distance equals to the minimal distance
    // between poonts on segments: T on *this, O on ls
    // T = (x1t + alpha*dxt, y1t + alpha*dyt, z1t + alpha*dzt)
    // O = (x1o + beta*dxo, y1o + beta*dyo, z1o + beta*dzo)
    // distance_squared = (x1o-x1t + beta*dxo - alpha*dxt)**2 + ...
    // -d(distance_squared) / d(alpha) / 2 =
    //     = (x1o-x1t + beta*dxo - alpha*dxt) * dxt + ... = 0
    // d(distance_squared) / d(beta) / 2 =
    //     = (x1o-x1t + beta*dxo - alpha*dxt) * dxo + ... = 0
    // System is:
    //     a11 a12 | b1
    //     a21 a22 | b2
    double a11 = -dxt*dxt - dyt*dyt - dzt*dzt,
           a12 = dxo*dxt + dyo*dyt + dzo*dzt,
           a21 = -dxt*dxo -dyt*dyo - dzt*dzo,
           a22 = dxo*dxo + dyo*dyo + dzo*dzo,
           b1 = dxt*(x1t-x1o) + dyt*(y1t-y1o) + dzt*(z1t-z1o),
           b2 = dxo*(x1t-x1o) + dyo*(y1t-y1o) + dzt*(z1t-z1o);
    double det = a11*a22 - a21*a12,
           det_alpha = b1*a22 - b2*a12,
           det_beta = a11*b2 - a21*b1;
    // det != 0 because segments are not parallel
    //          and are not in a single plane
    double alpha = det_alpha / det,
           beta = det_beta / det;
    // Found points belong to carrying lines.
    // Now they have to be placed into thier segments
    if (alpha < 0.)
      {
        alpha = 0.;
      }
    else if (alpha > 1.)
      {
        alpha = 1.;
      }
    if (beta < 0.)
      {
        beta = 0.;
      }
    else if (beta > 1.)
      {
        beta = 1.;
      }
    double xtc = x1t + alpha*dxt, ytc = y1t + alpha*dyt, ztc = z1t + alpha*dzt,
           xoc = x1o + beta*dxo, yoc = y1o + beta*dyo, zoc = z1o + beta*dzo,
           dx = xoc - xtc,  dy = yoc - ytc, dz = zoc - ztc;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

double
LineSegment::distance_to(const Line &l) const
{
    return l.distance_to(*this);
}

double
LineSegment::distance_to(const Plane &pl) const
{
    return pl.distance_to(*this);
}

double
LineSegment::distance_to(const Polygon &poly) const
{
    return poly.distance_to(*this);
}


}  // ns geom3d
