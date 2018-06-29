#pragma once

#include <ostream>

struct point {
    double x;
    double y;
};

struct wind_cursor {
//    virtual void move(point const& p) = 0;
//    virtual point get_wind() const = 0;
//    virtual ~wind_cursor(){}
    void move(point const& p) {
        return;
    }

    point get_wind() const {
        return {10, 10};
    }

    ~wind_cursor() = default;
};