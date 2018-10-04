#include "point.hpp"
#include "line.hpp"
#include "line_segment.hpp"
#include "plane.hpp"
#include "polygon.hpp"


namespace geom3d
{


Point
Line::pt1() const
{
    return pt1_;
}


Point
Line::pt2() const
{
    return pt2_;
}


double
Line::distance_to(const Point &pt) const
{
    double x1 = pt1_.x(), x2 = pt2_.x(), x = pt.x(),
           y1 = pt1_.y(), y2 = pt2_.y(), y = pt.y(),
           z1 = pt1_.z(), z2 = pt2_.z(), z = pt.z(),
           dx = x2 - x1, dy = y2 - y1, dz = z2 - z1,  // vec(pt1_ -> pt2_)
           len = sqrt(dx*dx + dy*dy + dz*dz),
           dx1 = x - x1, dy1 = y - y1, dz1 = z - z1,  // vec(pt1_ -> pt)
           len1 = sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1);
    if (almost_equal(len1) || almost_equal(len))
      {
        return 0;
      }
    double cos_angle = (dx*dx1 + dy*dy1 + dz*dz1) / len / len1,
           sin_angle = sqrt(1 - cos_angle*cos_angle),
           distance = len * sin_angle;
    return std::abs(distance);
}

double
Line::distance_to(const LineSegment &ls) const
{
    double x1t = pt1_.x(), x2t = pt2_.x(), x1o = ls.pt1().x(), x2o = ls.pt2().x(),
           y1t = pt1_.y(), y2t = pt2_.y(), y1o = ls.pt1().y(), y2o = ls.pt2().y(),
           z1t = pt1_.z(), z2t = pt2_.z(), z1o = ls.pt1().z(), z2o = ls.pt2().z(),
           dxt = x2t - x1t, dyt = y2t - y1t, dzt = z2t - z1t,
           lent = sqrt(dxt*dxt + dyt*dyt + dzt*dzt),
           dxo = x2o - x1o, dyo = y2o - y1o, dzo = z2o - z1o,
           leno = sqrt(dxo*dxo + dyo*dyo + dzo*dzo),
           cos_angle = (dxt*dxo + dyt*dyo + dzt*dzo) / lent / leno;

    if (almost_equal(std::abs(cos_angle), 1))
      {  // Parallel case
        return distance_to(ls.pt1());
      }

    // surely not parallel
    // ptt(xt, yt, zt) on *this, pto(xo, yo, zo) on other;
    //   dxt,  -dxo = x1o-x1t
    //   dyt,  -dyo = y1o - y1t
    // minimum exists, det != 0

    double det = dxo*dyt - dxt*dyo,
           det_alpha = dxo*(y1o-y1t) - dyo*(x1o-x1t),
           det_beta = dxt*(y1o-y1t) - dyt*(x1o-x1t),
           alpha = det_alpha / det,
           beta = det_beta / det;
    if (beta < 0.)
      {
        beta = 0.;
      }
    if (beta > 1.)
      {
        beta = 1.;
      }
    double xt = x1t + alpha*dxt, yt = y1t + alpha*dyt, zt = z1t + alpha*dzt,
           xo = x1o + beta*dxo, yo = y1o + beta*dyo, zo = z1o + beta*dzo,
           dx = xo - xt, dy = yo - yt, dz = zo - zt;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

double
Line::distance_to(const Line &l) const
{
    double x1t = pt1_.x(), x2t = pt2_.x(), x1o = l.pt1().x(), x2o = l.pt2().x(),
           y1t = pt1_.y(), y2t = pt2_.y(), y1o = l.pt1().y(), y2o = l.pt2().y(),
           z1t = pt1_.z(), z2t = pt2_.z(), z1o = l.pt1().z(), z2o = l.pt2().z(),
           dxt = x2t - x1t, dyt = y2t - y1t, dzt = z2t - z1t,
           lent = sqrt(dxt*dxt + dyt*dyt + dzt*dzt),
           dxo = x2o - x1o, dyo = y2o - y1o, dzo = z2o - z1o,
           leno = sqrt(dxo*dxo + dyo*dyo + dzo*dzo),
           cos_angle = (dxt*dxo + dyt*dyo + dzt*dzo) / lent / leno;

    if (almost_equal(std::abs(cos_angle), 1))
      {  // Parallel case
        return distance_to(l.pt1());
      }

    // surely not parallel
    // ptt(xt, yt, zt) on *this, pto(xo, yo, zo) on other;
    //   dxt,  -dxo = x1o-x1t
    //   dyt,  -dyo = y1o - y1t
    // minimum exists, det != 0

    double det = dxo*dyt - dxt*dyo,
           det_alpha = dxo*(y1o-y1t) - dyo*(x1o-x1t),
           det_beta = dxt*(y1o-y1t) - dyt*(x1o-x1t),
           alpha = det_alpha / det,
           beta = det_beta / det,
           xt = x1t + alpha*dxt, yt = y1t + alpha*dyt, zt = z1t + alpha*dzt,
           xo = x1o + beta*dxo, yo = y1o + beta*dyo, zo = z1o + beta*dzo,
           dx = xo - xt, dy = yo - yt, dz = zo - zt;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

double
Line::distance_to(const Plane &pl) const
{
    return pl.distance_to(*this);
}

double
Line::distance_to(const Polygon &poly) const
{
    return poly.distance_to(*this);
}


}  // ns geom3d
