#pragma once

#include <cmath>
#include <vector>
#include "wind_cursor.h"
#include "vec.h"
#include "engine.h"
#include "../include/matplotlibcpp.h"
namespace plt = matplotlibcpp;

#define PI 3.14159265
//#define DEBUG

//http://www.physicedu.ru/phy-1592.html
const double AIR_RESISTANCE_COEFFICIENT_FRONT = 0.04; //

namespace simulation {

    double const G = 9.8; // gravity acceleration, m/s^2
    const double rho = 1.2754; // air density, kg/m^3

    struct helicopter {
        const double mass_;
        const double cabin_square_;
        const double screw_diameter_; // m
        const double screw_square_; // m^2
        const double C_R_;
//        const double screw_angle = 0.0785; // rad

        point position_;
        double pitch_;

        double screw_rotation_;
        vec velocity_;
        vec acceleration_;
        point goal_;
        bool moving_left;

        helicopter(double mass, double cabin_square, double screw_diameter,
                   double C_R, point const &pos, double pitch, double screw_rotation,
                   double velocity = 0, vec const &acceleration = {}, point const &goal = {})
                : mass_(mass),
                  cabin_square_(cabin_square),
                  screw_diameter_(screw_diameter),
                  screw_square_(PI * screw_diameter_ * screw_diameter_ / 4),
                  C_R_(C_R),
                  position_(pos),
                  pitch_(pitch),
                  screw_rotation_(screw_rotation),
                  velocity_({velocity * std::cos(angle()), velocity * std::sin(angle())}),
                  acceleration_(acceleration),
                  goal_(goal),
                  moving_left((goal_.x - position_.x) < 0) {}

        double angle() {
            return (moving_left) ? (PI - pitch_) : pitch_;
        }

        void update_position(double dT) {
            auto acc_change = acceleration_ * dT * dT / 2;
            auto speed_change = velocity_ * dT;
            position_.x += (speed_change + acc_change).x;
            position_.y += (speed_change + acc_change).y;
        }

        void update_speed(double dT) {
            velocity_ += acceleration_ * dT;
        }

        void update_pitch(double dT) {
            pitch_ = (moving_left) ? (PI - velocity_.angle()) : velocity_.angle();
            if (pitch_ > PI / 2) {
                pitch_ -= 2 * PI;
            }
        }

        void update_screw_rotation(double dT) {

        }

        vec aerodynamic_force(vec const &wind) {
            vec  total_flow = wind - velocity_;
            double force = C_R_ * screw_square_ * rho * std::pow(total_flow.y, 2) / 2;
            double angle = wind.angle() + PI / 2;
            return {force * std::cos(angle), force * std::sin(angle)};
        }

        vec air_resistance(vec const &wind) {
            vec  total_flow = wind - velocity_;
            double force = AIR_RESISTANCE_COEFFICIENT_FRONT * cabin_square_ * rho * std::pow(total_flow.x, 2) / 2;
            return {force, 0};
        }

        std::pair<std::vector<vec>, std::vector<std::string>> collect_forces(vec const &wind) {
            std::pair<std::vector<vec>, std::vector<std::string>> forces_and_names;

            forces_and_names.first.push_back({0, -G * mass_});
            forces_and_names.second.push_back("gravity_force");

            forces_and_names.first.push_back(air_resistance(wind));
            forces_and_names.second.push_back("air_resistance");

            forces_and_names.first.push_back(aerodynamic_force(wind));
            forces_and_names.second.push_back("aerodynamic_force");

#ifdef DEBUG
            std::cout << "velocity: " << velocity_ << '\n';
            std::cout << "wind: " << wind << '\n';
            vec total;
            for (int i = 0; i < forces_and_names.first.size(); ++i) {
                total += forces_and_names.first[i];
                std::cout << forces_and_names.second[i] << forces_and_names.first[i] << '\n';
            }
            std::cout << "total: " << total << '\n';
#endif
            return forces_and_names;
        }

        void update_state(point const &wind, double dT) {
            std::vector<vec> forces = collect_forces(wind).first;
            update_acceleration(forces);
            update_speed(dT);
            update_position(dT);
            update_pitch(dT);
            update_screw_rotation(dT);
#ifdef DEBUG
            std::cout << "pitch: " << pitch_ * 180 / PI << "\n----\n";
#endif
        }

        void update_acceleration(std::vector<vec> const &forces) {
            vec total_force;
            for(auto force : forces) {
                total_force += force;
            }
            acceleration_.x = total_force.x / mass_;
            acceleration_.y = total_force.y / mass_;
        }

        void plot_force(const vec &force, const std::string &name) {
            std::vector<double> xs = {position_.x, position_.x + force.x};
            std::vector<double> ys = {position_.y, position_.y + force.y};
            plt::named_plot(name, xs, ys);
        }

        void plot_helicopter(vec const &wind, bool with_forces) {
            if (with_forces) {
                auto vv = collect_forces(wind);
                for (int i =0; i < vv.first.size(); ++i) {
                    plot_force(vv.first[i], vv.second[i]);
                }
            }
            double length_x = screw_diameter_ * std::cos(angle());
            double length_y = screw_diameter_ * std::sin(angle());
            std::vector<double> xs = {position_.x - length_x / 2, position_.x + length_x / 2};
            std::vector<double> ys = {position_.y - length_y / 2, position_.y + length_y / 2};
            plt::plot(xs, ys);
            plt::grid(true);
        }

    };


}