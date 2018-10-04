#include "point.hpp"
#include "line_segment.hpp"
#include "line.hpp"
#include "plane.hpp"
#include "polygon.hpp"

#include "pretty_printer.hpp"


void
PrettyPrinter::pprint(const geom3d::Point &pt, const int indent_level) const
{
    std::string begin_str = prompt_;

    if (indent_level > 0)
      {
        begin_str += std::string(indent_level * indent_, ' ');
      }

    std::cout << begin_str
        << "Point at ("
            << pt.x() << ", "
            << pt.y() << ", "
            << pt.z() << ")"
        << std::endl;
}


void
PrettyPrinter::pprint(const geom3d::LineSegment &ls, const int indent_level) const
{
    std::string begin_str = prompt_;
    if (indent_level > 0)
      {
        begin_str += std::string(indent_level * indent_, ' ');
      }

    std::cout << begin_str
        << "LineSegment with:\n";
    geom3d::Point pt1 = ls.pt1();
    geom3d::Point pt2 = ls.pt2();
    pprint(pt1, indent_level+1);
    pprint(pt2, indent_level+1);
}


void
PrettyPrinter::pprint(const geom3d::Line &l, const int indent_level) const
{
    std::string begin_str = prompt_;
    if (indent_level > 0)
      {
        begin_str += std::string(indent_level * indent_, ' ');
      }

    std::cout << begin_str
        << "Line with:\n";
    geom3d::Point pt1 = l.pt1();
    geom3d::Point pt2 = l.pt2();
    pprint(pt1, indent_level+1);
    pprint(pt2, indent_level+1);
}


void
PrettyPrinter::pprint(const geom3d::Plane &pl, const int indent_level) const
{
    std::string begin_str = prompt_;
    if (indent_level > 0)
      {
        begin_str += std::string(indent_level * indent_, ' ');
      }


    std::cout << begin_str
        << "Plane with a = " << pl.a() << ", b = " << pl.b() << ", c = " 
                             << pl.c() << ", d = " << pl.d() << std::endl;
}


void
PrettyPrinter::pprint(const geom3d::Polygon &poly, const int indent_level) const
{
    std::string begin_str = prompt_;
    if (indent_level > 0)
      {
        begin_str += std::string(indent_level * indent_, ' ');
      }

    int vertices_number = poly.vertices().size();
    std::cout << begin_str
        << "Polygon with " << vertices_number << " vetices at:\n";
    for (int idx = 0; idx < vertices_number; ++idx)
      {
        geom3d::Point pt = *(poly.vertices()[idx]);
        pprint(pt, indent_level+1);
      }
}
