#include <iostream>
#include <memory>

#include "src/point.hpp"
#include "src/line_segment.hpp"
#include "src/polygon.hpp"
#include "src/plane.hpp"
#include "src/line.hpp"

#include "src/pretty_printer.hpp"
#include "src/utils.hpp"


double pi = 3.14159265358979;


void test_distances()
{
    using namespace geom3d;

    Point pt00(0, 0, 0), pt01(1, 0, 0), pt02(1, 1, 0), pt03(0, 1, 0),
          pt10(0, 0, 1), pt11(1, 0, 1), pt12(1, 1, 1), pt13(0, 1, 1);
    LineSegment ls0(pt00, pt01), ls1(pt10, pt11);
    Line l0(pt00, pt01), l1(pt10, pt11);
    Plane pl0(pt00, pt01, pt02), pl1(pt10, pt11, pt12);
    Polygon poly0(std::vector<std::shared_ptr<Point> > {
            std::make_shared<Point>(pt00),
            std::make_shared<Point>(pt01),
            std::make_shared<Point>(pt02),
            std::make_shared<Point>(pt03),
        }), poly1(std::vector<std::shared_ptr<Point> > {
            std::make_shared<Point>(pt10),
            std::make_shared<Point>(pt11),
            std::make_shared<Point>(pt12),
            std::make_shared<Point>(pt13),
        });
    std::cout << "*********************** DISTANCES ***********************\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "|\t\tpt\tls\tline\tpl\tpoly\t|\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "|\tpt\t" << pt00.distance_to(pt01) << "\t"
                        << pt00.distance_to(ls0) << "\t"
                        << pt00.distance_to(l0) << "\t"
                        << pt00.distance_to(pl0) << "\t"
                        << pt00.distance_to(poly0) << "\t|\n";
    std::cout << "|\tls\t" << ls0.distance_to(pt00) << "\t"
                        << ls0.distance_to(ls1) << "\t"
                        << ls0.distance_to(l0) << "\t"
                        << ls0.distance_to(pl0) << "\t"
                        << ls0.distance_to(poly0) << "\t|\n";
    std::cout << "|\tl\t" << l0.distance_to(pt00) << "\t"
                       << l0.distance_to(ls0) << "\t"
                       << l0.distance_to(l1) << "\t"
                       << l0.distance_to(pl0) << "\t"
                       << l0.distance_to(poly0) << "\t|\n";
    std::cout << "|\tpl\t" << pl0.distance_to(pt00) << "\t"
                        << pl0.distance_to(ls0) << "\t"
                        << pl0.distance_to(l0) << "\t"
                        << pl0.distance_to(pl1) << "\t"
                        << pl0.distance_to(poly0) << "\t|\n";
    std::cout << "|\tpoly\t" << poly0.distance_to(pt00) << "\t"
                          << poly0.distance_to(ls0) << "\t"
                          << poly0.distance_to(l0) << "\t"
                          << poly0.distance_to(pl0) << "\t"
                          << poly0.distance_to(poly1) << "\t|\n";
    std::cout << "---------------------------------------------------------\n";

    return;
}


int main()
{
    test_distances();
    return 0;

}
