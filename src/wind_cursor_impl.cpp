#include <iostream>
#include "wind_cursor.h"

struct wind_cursor_impl: wind_cursor {

    void move(point const& p) override {}

    point get_wind() const override {
        return {20, 10};
    }
};

std::shared_ptr<wind_cursor> get_wind_cursor() {
    return std::shared_ptr<wind_cursor>(new wind_cursor_impl());
}