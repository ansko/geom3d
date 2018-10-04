#ifndef _PRETTY_PRINTER_HPP
#define _PRETTY_PRINTER_HPP


#include <iostream>
#include <string>


class PrettyPrinter
{
public:
    PrettyPrinter(int indent)
        : indent_(indent)
        , prompt_("")
    { };
    PrettyPrinter(std::string prompt)
        : indent_(4)
        , prompt_(prompt + std::string(int(prompt.length() > 0), ' '))
    { };
    PrettyPrinter(int indent, std::string prompt)
        : indent_(indent)
        , prompt_(prompt + std::string(int(prompt.length() > 0), ' '))
    { };

    void pprint(const geom3d::Point &pt, const int indent_level=0) const;
    void pprint(const geom3d::LineSegment &ls, const int indent_level=0) const;
    void pprint(const geom3d::Line &l, const int indent_level=0) const;
    void pprint(const geom3d::Plane &pl, const int indent_level=0) const;
    void pprint(const geom3d::Polygon &poly, const int indent_level=0) const;

private:
    int indent_;  /* difference in identation between inner and outer levels */
    std::string prompt_;  /* like promt in terminal */
};


#endif // _PRETTY_PRINTER_HPP (pretty_printer include guard)
