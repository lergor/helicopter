#pragma once

#include "helicopter.h"

namespace simulation {

    struct screw {
        int num_of_blades;
        double blade_length;
        double delta_angle;

        screw(int num, double length)
                : num_of_blades(num),
                  blade_length(length),
                  delta_angle( 2 * PI / num) {}

        vec calculate_aero_force(vec const &wind, vec const &helicopter_velocity) {
            double attack_angle = std::acos((helicopter_velocity * wind) /
                                                    (helicopter_velocity.length() * wind.length()));


        }

        vec calculate_blade_force(vec const &wind, int num, double attack_angle, double time, double w) {
            double angle = num * delta_angle + time * w;
            double tangential_force = w * blade_length + wind.length() * std::sin(angle);
        }
    };
}