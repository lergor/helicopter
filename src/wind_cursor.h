#pragma once

#include <ostream>
#include <memory>

struct point {
    double x;
    double y;
};

std::ostream &operator<<(std::ostream &o, point const &p) {
    o << "{x: " << p.x << ", y: " << p.y  << "}";
}

struct wind_cursor {
    virtual void move(point const& p) = 0;
    virtual point get_wind() const = 0;
    virtual ~wind_cursor(){}
};

std::shared_ptr<wind_cursor> get_wind_cursor();