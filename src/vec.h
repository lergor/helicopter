#pragma once

#include <cmath>
#include <ostream>

namespace simulation {

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    struct vec {
        double x{};
        double y{};

        vec() = default;

        vec(double x, double y): x(x), y(y) {}

        vec(const point &p): x(p.x), y(p.y) {}

        double angle() const {
            return std::acos(x / length()) * sgn(y);
        }

        double length() const {
            return std::sqrt(x * x + y * y);
        }

        vec operator*(double m) const {
            return {x * m, y * m};
        }

        vec operator/(double m) const {
            return {x / m, y / m};
        }

        vec operator+(vec const &other) const {
            return {x + other.x, y + other.y};
        }

        vec &operator+=(vec const &other) {
            x += other.x;
            y += other.y;
            return *this;
        }

        vec operator-(vec const &other) const {
            return {x - other.x, y - other.y};
        }

        double operator*(vec const &other) const {
            return x * other.x + y * other.y;
        }
    };

    std::ostream &operator<<(std::ostream &o, vec const &v) {
        o << "{dx: " << v.x << ", dy: " << v.y << ", angle: " << v.angle() * 180 / 3.14159265 << "}";
    }
}