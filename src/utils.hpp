#ifndef _UTILS_HPP_
#define _UTILS_HPP_


#include <limits>

#include <iostream>


inline bool
almost_equal(const double x, const double y=0.0, const int difficulty=1)
{
    // The simplest algorithm for comparison of doubles.
    // difficulty represents how complicated equation is,
    //     i.e., how much the error might increase compared
    //     to the error of double representation,
    //     for example, when almost_equal(x, y), difficulty is 1;
    //         almost_equalt(x**4, yy**5), difficulty is 9 (9 = 4+5).

    double eps = std::numeric_limits<double>::epsilon();

    //std::cout << eps * std::abs(x + y) * difficulty << std::endl;

    return std::abs(x - y) <= eps * std::abs(x + y) * difficulty;
}


#endif // _UTILS_HPP_ (utils.hpp include guard)
