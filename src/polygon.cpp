#include "point.hpp"
#include "line_segment.hpp"
#include "line.hpp"
#include "plane.hpp"
#include "polygon.hpp"

#include "pretty_printer.hpp"


namespace geom3d
{


double
Polygon::square() const
{
    double half_central_angle = pi / pt_ptrs_.size(),
           x0 = pt_ptrs_[0]->x(), y0 = pt_ptrs_[0]->y(), z0 = pt_ptrs_[0]->z(),
           x1 = pt_ptrs_[1]->x(), y1 = pt_ptrs_[1]->y(), z1 = pt_ptrs_[1]->z(),
           dx = x1-x0, dy = y1-y0, dz = z1-z0,
           edge_length = sqrt(dx*dx + dy*dy + dz*dz),
           outer_r = edge_length / 2 / sin(half_central_angle);
    return outer_r*outer_r * sin(2*half_central_angle) / 2 * pt_ptrs_.size();
}

Point
Polygon::center() const
{
    double xc = 0, yc = 0, zc = 0;
    int vertices_number = pt_ptrs_.size();
    for (int idx = 0; idx < vertices_number; ++ idx)
      {
        xc += pt_ptrs_[idx]->x();
        yc += pt_ptrs_[idx]->y();
        zc += pt_ptrs_[idx]->z();
      }
    return Point(xc/vertices_number, yc/vertices_number, zc/vertices_number);
}

std::vector<std::shared_ptr<Point> >
Polygon::vertices() const
{
    return pt_ptrs_;
}


double
Polygon::distance_to_in_plane_point(const Point &pt) const
{
    int vertices_number = pt_ptrs_.size();
    if (contains_point(pt))
      {
        return 0;
      }
    double distance = LineSegment(*(pt_ptrs_[0]), *(pt_ptrs_[1])).distance_to(pt);
    for (int idx = 1; idx < vertices_number; ++idx)
      {
        LineSegment ls(*(pt_ptrs_[idx-1]), *(pt_ptrs_[idx]));
        double tmp = ls.distance_to(pt);
        if (tmp < distance)
          {
            distance = tmp;
          }
      }
    return distance;
}

double
Polygon::distance_to(const Point &pt) const
{
    // Result equals to the minimal velue from:
    //     distance to plane containing this polygon
    //     distances to edges
    int vertices_number = pt_ptrs_.size();
    Plane pl(*(pt_ptrs_[0]), *(pt_ptrs_[1]), *(pt_ptrs_[2]));
    double distance_to_pt_plane = pl.distance_to(pt);
    if (almost_equal(distance_to_pt_plane))
      {
        return distance_to_in_plane_point(pt);
      }
    // Pt is in other plane
    double dx1 = pt_ptrs_[1]->x() - pt_ptrs_[0]->x(),
           dy1 = pt_ptrs_[1]->y() - pt_ptrs_[0]->y(),
           dz1 = pt_ptrs_[1]->z() - pt_ptrs_[0]->z(),
           dx2 = pt_ptrs_[2]->x() - pt_ptrs_[0]->x(),
           dy2 = pt_ptrs_[2]->y() - pt_ptrs_[0]->y(),
           dz2 = pt_ptrs_[2]->z() - pt_ptrs_[0]->z(),
           nx = dy1*dz2 - dy2*dz1, ny = dx2*dz1 - dx1*dz2,
           nz = dx1*dy2 - dx2*dy1,
           lenn = sqrt(nx*nx + ny*ny + nz*nz);
    nx /= (lenn / distance_to_pt_plane);
    ny /= (lenn / distance_to_pt_plane);
    nz /= (lenn / distance_to_pt_plane);
    {  // Discovering orientation of normal
       // and setting it to 'high'
        double dxlo = pt_ptrs_[0]->x() - nx, dxhi = pt_ptrs_[0]->x() + nx,
               dylo = pt_ptrs_[0]->y() - ny, dyhi = pt_ptrs_[0]->y() + ny,
               dzlo = pt_ptrs_[0]->z() - nz, dzhi = pt_ptrs_[0]->z() + nz,
               lenlo = sqrt(dxlo*dxlo + dylo*dylo + dzlo*dzlo),
               lenhi = sqrt(dxhi*dxhi + dyhi*dyhi + dzhi*dzhi);
        if (lenhi > lenlo)
          {
            nx *= -1; ny *= -1; nz *= -1;
          }
    }
    std::vector<std::shared_ptr<Point> > new_pt_ptrs;
    for (int idx = 0; idx < vertices_number; idx++)
      {
        double x = pt_ptrs_[idx]->x() + nx,
               y = pt_ptrs_[idx]->y() + ny,
               z = pt_ptrs_[idx]->z() + nz;
        new_pt_ptrs.push_back(std::make_shared<Point>(Point(x, y, z)));
      }
    Polygon new_poly = Polygon(new_pt_ptrs);
    double in_plane_distance = new_poly.distance_to(pt);
    //std::cout << "in-plane: " << in_plane_distance << std::endl;
    return sqrt(in_plane_distance*in_plane_distance
                + distance_to_pt_plane*distance_to_pt_plane);
}

double
Polygon::distance_to(const LineSegment &ls) const
{
    // If ls crosses *this, distance equals 0.
    //
    // If both ls' ends are inside polyhedron,
    // distance equals to the minimal from distances
    // to the ls' ends.
    //
    // If both ends are outside, distance equals
    // to the minimum from the distnaces between
    // edge and ls.
    Plane pl(*pt_ptrs_[0], *pt_ptrs_[1], *pt_ptrs_[2]);
    double plane_distance = pl.distance_to(ls);
    if (almost_equal(plane_distance))  // intersection exists
      {
        double x1 = ls.pt1().x(), x2 = ls.pt2().x(),
               y1 = ls.pt1().y(), y2 = ls.pt2().y(),
               z1 = ls.pt1().z(), z2 = ls.pt2().z(),
               xt0 = pt_ptrs_[0]->x(),
               yt0 = pt_ptrs_[0]->y(),
               zt0 = pt_ptrs_[0]->z(),
               xt1 = pt_ptrs_[1]->x(),
               yt1 = pt_ptrs_[1]->y(),
               zt1 = pt_ptrs_[1]->z(),
               xt2 = pt_ptrs_[2]->x(),
               yt2 = pt_ptrs_[2]->y(),
               zt2 = pt_ptrs_[2]->z(),
               dx10 = x1-xt0, dy10 = y1-yt0, dz10 = z1-zt0,
               dx11 = x1-xt1, dy11 = y1-yt1, dz11 = z1-zt1,
               dx12 = x1-xt2, dy12 = y1-yt2, dz12 = z1-zt2,
               dx20 = x2-xt0, dy20 = y2-yt0, dz20 = z2-zt0,
               dx21 = x2-xt1, dy21 = y2-yt1, dz21 = z2-zt1,
               dx22 = x2-xt2, dy22 = y2-yt2, dz22 = z2-zt2,
               vol1 = dx10 * (dy11*dz12 - dy12*dz11)
                      - dy10 * (dx11*dz12 - dx12*dz11)
                      + dz10 * (dx11*dy12 - dx12*dy11),
               vol2 = dx20 * (dy21*dz22 - dy22*dz21)
                      - dy20 * (dx21*dz22 - dx22*dz21)
                      + dz20 * (dx21*dy22 - dx22*dy21),
               dxls = x2-x1, dyls = y2-y1, dzls = z2-z1;
               if (almost_equal(vol1) || almost_equal(vol2))
                 {
                   return 0;
                 }
        double coeff = std::abs(vol1) / (std::abs(vol1) + std::abs(vol2)),
               x_cross = x1 + coeff*dxls,
               y_cross = y1 + coeff*dyls,
               z_cross = z1 + coeff*dzls;
        if (contains_point(x_cross, y_cross, z_cross))
          {
            return 0;
          }
      }
    // Find minimum from the distances between ls' ends and *this
    // and the ditances between ls a nd edges ofg *this.
    double min_distance_found = this->distance_to(ls.pt1());
    {
        double ls_pt2_distance = this->distance_to(ls.pt2());
        if (ls_pt2_distance < min_distance_found)
          {
            min_distance_found = ls_pt2_distance;
          }
    }
    int vertices_number = pt_ptrs_.size();
    for (int idx = 0; idx < vertices_number; ++idx)
      {
        Point pt_vertex;
        if (idx == 0)
          {
            pt_vertex = *pt_ptrs_[vertices_number-1];
          }
        else
          {
            pt_vertex = *pt_ptrs_[idx-1];
          }
        LineSegment edge(*pt_ptrs_[idx], pt_vertex);
        double distance = edge.distance_to(ls);
        if (distance < min_distance_found)
          {
            min_distance_found = distance;
          }
      }
    return min_distance_found;
}

double
Polygon::distance_to(const Line &l) const
{
    Plane pl(*pt_ptrs_[0], *pt_ptrs_[1], *pt_ptrs_[2]);
    double plane_distance = pl.distance_to(l);
    if (almost_equal(plane_distance))  // intersection exists
      {
        double x1 = l.pt1().x(), x2 = l.pt2().x(),
               y1 = l.pt1().y(), y2 = l.pt2().y(),
               z1 = l.pt1().z(), z2 = l.pt2().z(),
               xt0 = pt_ptrs_[0]->x(),
               yt0 = pt_ptrs_[0]->y(),
               zt0 = pt_ptrs_[0]->z(),
               xt1 = pt_ptrs_[1]->x(),
               yt1 = pt_ptrs_[1]->y(),
               zt1 = pt_ptrs_[1]->z(),
               xt2 = pt_ptrs_[2]->x(),
               yt2 = pt_ptrs_[2]->y(),
               zt2 = pt_ptrs_[2]->z(),
               dx10 = x1-xt0, dy10 = y1-yt0, dz10 = z1-zt0,
               dx11 = x1-xt1, dy11 = y1-yt1, dz11 = z1-zt1,
               dx12 = x1-xt2, dy12 = y1-yt2, dz12 = z1-zt2,
               dx20 = x2-xt0, dy20 = y2-yt0, dz20 = z2-zt0,
               dx21 = x2-xt1, dy21 = y2-yt1, dz21 = z2-zt1,
               dx22 = x2-xt2, dy22 = y2-yt2, dz22 = z2-zt2,
               vol1 = dx10 * (dy11*dz12 - dy12*dz11)
                      - dy10 * (dx11*dz12 - dx12*dz11)
                      + dz10 * (dx11*dy12 - dx12*dy11),
               vol2 = dx20 * (dy21*dz22 - dy22*dz21)
                      - dy20 * (dx21*dz22 - dx22*dz21)
                      + dz20 * (dx21*dy22 - dx22*dy21),
               dxls = x2-x1, dyls = y2-y1, dzls = z2-z1,
               coeff = std::abs(vol1) / (std::abs(vol1) + std::abs(vol2)),
               x_cross = x1 + coeff*dxls,
               y_cross = y1 + coeff*dyls,
               z_cross = z1 + coeff*dzls;
        if (contains_point(x_cross, y_cross, z_cross))
          {
            return 0;
          }
      }
    // Find minimum from the distances between ls' ends and *this
    // and the ditances between ls a nd edges ofg *this.
    double min_distance_found = pl.distance_to(l);
    int vertices_number = pt_ptrs_.size();
    for (int idx = 0; idx < vertices_number; ++idx)
      {
        Point pt_vertex;
        if (idx == 0)
          {
            pt_vertex = *pt_ptrs_[vertices_number-1];
          }
        else
          {
            pt_vertex = *pt_ptrs_[idx-1];
          }
        LineSegment edge(*pt_ptrs_[idx], pt_vertex);
        double distance = l.distance_to(edge);
        if (distance < min_distance_found)
          {
            min_distance_found = distance;
          }
      }
    return min_distance_found;
}

double
Polygon::distance_to(const Plane &pl) const
{
    // If any adge of *this crosses pl, distance equals 0.
    // Otherwise it equals to the minimum from all
    // distances between vertices of *this and plane.

    int vertices_number = pt_ptrs_.size();
    for (int idx = 0; idx < vertices_number; ++idx)
      {
        Point pt_vertex;
        if (idx == 0)
          {
            pt_vertex = *pt_ptrs_[vertices_number-1];
          }
        else
          {
            pt_vertex = *pt_ptrs_[idx-1];
          }
        LineSegment edge(*pt_ptrs_[idx], pt_vertex);
        if (almost_equal(pl.distance_to(edge)))
          {
            return 0;
          }
      }
    double minimal_vertex_distance = pl.distance_to(
        LineSegment(*pt_ptrs_[vertices_number-1], *pt_ptrs_[0]));
    for (int idx = 1; idx < vertices_number; ++idx)
      {
        double vertex_distance = pl.distance_to(*pt_ptrs_[idx]);
        if (vertex_distance < minimal_vertex_distance)
          {
            minimal_vertex_distance = vertex_distance;
          }
      }
    return minimal_vertex_distance;
}

double
Polygon::distance_to(const Polygon &poly) const
{
    std::vector<std::shared_ptr<Point> > verts_other = poly.vertices();
    {  // Check intersection
        Plane plane_this(*pt_ptrs_[0], *pt_ptrs_[1], *pt_ptrs_[2]),
              plane_other(*verts_other[0], *verts_other[1], *verts_other[2]);
        if (almost_equal(poly.distance_to(plane_this))
            && almost_equal(this->distance_to(plane_other)))
          {
            //std::cout << "POLY CROSS\n";
            return 0;
          }
    }
    // No intersection found
    //std::cout << "NOCROSS\n";
    int vertices_number_this = pt_ptrs_.size(),
        vertices_number_other = verts_other.size();
    double minimal_distance_found;
    {
        LineSegment ls(*pt_ptrs_[0], *pt_ptrs_[1]);
        minimal_distance_found = poly.distance_to(ls);
    }
    {
        LineSegment ls(*verts_other[0], *verts_other[1]);
        double minimal_distance = this->distance_to(ls);
        if (minimal_distance < minimal_distance_found)
          {
            minimal_distance_found = minimal_distance;
          }
    }
    for (int idx = 1; idx < vertices_number_this; ++idx)
      {
        LineSegment ls(*pt_ptrs_[idx], *pt_ptrs_[idx-1]);
        double minimal_distance = poly.distance_to(ls);
        if (minimal_distance < minimal_distance_found)
          {
            minimal_distance_found = minimal_distance;
          }
      }
    for (int idx = 1; idx < vertices_number_other; ++idx)
      {
        LineSegment ls(*verts_other[idx], *verts_other[idx-1]);
        double minimal_distance = this->distance_to(ls);
        if (minimal_distance < minimal_distance_found)
          {
            minimal_distance_found = minimal_distance;
          }
      }


    return minimal_distance_found;
}


bool
Polygon::contains_point(const Point &pt) const
{
    return contains_point(pt.x(), pt.y(), pt.z());
}

bool
Polygon::contains_point(const double x, const double y, const double z) const
{
    double twice_square = 0.;
    int vertices_number = pt_ptrs_.size();
    for (int idx = 0; idx < vertices_number; ++idx)
      {
        double x1 = pt_ptrs_[idx]->x(), x2,
               y1 = pt_ptrs_[idx]->y(), y2,
               z1 = pt_ptrs_[idx]->z(), z2;
        if (idx == 0)
          {
            x2 = pt_ptrs_[vertices_number-1]->x();
            y2 = pt_ptrs_[vertices_number-1]->y();
            z2 = pt_ptrs_[vertices_number-1]->z();
          }
        else
          {
            x2 = pt_ptrs_[idx-1]->x();
            y2 = pt_ptrs_[idx-1]->y();
            z2 = pt_ptrs_[idx-1]->z();
          }
        double dx1 = x-x1, dy1 = y-y1, dz1 = z-z1,
               dx2 = x-x2, dy2 = y-y2, dz2 = z-z2,
               len1 = sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1),
               len2 = sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2),
               cos_angle = (dx1*dx2 + dy1*dy2 + dz1*dz2) / len1 / len2,
               sin_angle = sqrt(1 - cos_angle*cos_angle);
        twice_square += std::abs(len1 * len1 * sin_angle);
      }
    if (almost_equal(twice_square/2, square(), vertices_number))
      {
        return true;
      }
    return false;
}


} // ns geom3d
