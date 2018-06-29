#pragma once

#include <cmath>
#include <vector>
#include "wind_cursor.h"
#include "vec.h"
#include "engine.h"
#include "../include/matplotlibcpp.h"
namespace plt = matplotlibcpp;

#define PI 3.14159265

//http://www.physicedu.ru/phy-1592.html
const double AIR_RESISTANCE_COEFFICIENT_FRONT = 0.04; // Па
const double AIR_RESISTANCE_COEFFICIENT_BOTTOM = 0.47; // Па
const double SCREW_AIR_RESISTANCE_COEFFICIENT_FRONT = 0.82; // Па
const double SCREW_AIR_RESISTANCE_COEFFICIENT_BOTTOM = 1; // Па

const double ATMOSPHERE_PRESSURE = 101325; // Па

namespace simulation {

    double const G = 9.8; // gravity acceleration, m / c
    const double rho = 1.2754; // air density, kg/m^3

    struct helicopter {
        const double mass_;
        const double width_; // m^2
        const double height_; // m^2
        const double length_; // m^2
        const double screw_diameter_; // m
//        const double screw_angle = 0.0785;

        point position_;
        double pitch_;

        double screw_rotation_; //
        vec velocity_;
        vec acceleration_;
        point goal_;

        helicopter(double mass, double width, double height, double length,
                   double screw_diameter, point const &pos, double pitch,
                   double screw_rotation, double velocity = 0,
                   vec const &acceleration = {0, -G}, point const &goal = {})
                : mass_(mass),
                  width_(width),
                  height_(height),
                  length_(length),
                  screw_diameter_(screw_diameter),
                  position_(pos),
                  pitch_(pitch),
                  screw_rotation_(screw_rotation),
                  velocity_({velocity * std::cos(angle()), velocity * std::sin(angle())}),
                  acceleration_(acceleration),
                  goal_(goal) {}

        double angle() {
            return ((goal_.x - position_.x) < 0) ? (PI - pitch_) : pitch_;
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
            pitch_ = ((goal_.x - position_.x) < 0) ? (PI - velocity_.angle()) : velocity_.angle();
        }

        void update_screw_rotation(double dT) {

        }

        vec air_resistance(vec const &wind) {
            double air_resistance_x = AIR_RESISTANCE_COEFFICIENT_FRONT * width_ * height_ * rho * std::pow(wind.x + velocity_.x, 2) / 2;
            double air_resistance_y = AIR_RESISTANCE_COEFFICIENT_BOTTOM * width_ * length_ * rho * std::pow(wind.y + velocity_.y, 2) / 2;
            return {air_resistance_x, air_resistance_y};
        }

        vec aerodynamic_force(vec const &wind) {
            double  W = screw_rotation_ * screw_diameter_ / 2;
            double  square = PI * screw_diameter_ * screw_diameter_ / 4;

            double air_resistance_y = 5 * SCREW_AIR_RESISTANCE_COEFFICIENT_BOTTOM * square * rho * std::pow(wind.y + velocity_.y, 2) / 2;
            vec screw_air_resistance = {air_resistance_y * std::tan(angle()), air_resistance_y};

//            double traction_force_x = W * W * rho / 2 * std::cos(angle() + sgn(velocity_.x) * PI / 2);
//            double traction_force_y = W * W * rho / 2 * std::sin(angle() + sgn(velocity_.x) * PI / 2);
            double traction_force_x = 0.55 * square * rho * W * W / 2 * std::cos(angle() + sgn(velocity_.x) * PI / 2);
            vec traction_force;
            vec aerodynamic_force;

//            double air_resistance_x = AIR_RESISTANCE_COEFFICIENT_FRONT * square_ * rho * std::pow(wind.x + velocity_.x, 2) / 2;
//            double air_resistance_y = AIR_RESISTANCE_COEFFICIENT_BOTTOM * square_ * rho * std::pow(wind.y + velocity_.y, 2) / 2;

            return {};
        }

        std::pair<std::vector<vec>, std::vector<std::string>> collect_forces(vec const &wind) {
            std::pair<std::vector<vec>, std::vector<std::string>> forces_names;

            forces_names.first.push_back({0, -G * mass_});
            forces_names.second.push_back("gravity_force");

            vec air = air_resistance(wind);
            forces_names.first.push_back(air);
            forces_names.second.push_back("air_resistance");
            std::cout << air.angle() * 180 / PI << ' '
                      << angle() * 180 / PI << ' '
                      << (air.angle() - angle()) * 180 / PI << '\n';

//            forces_names.first.push_back(aerodynamic_force(wind));
//            forces_names.second.push_back("aerodynamic_force");

            return forces_names;
        }

        void update_state(point const &wind, double dT) {
            std::vector<vec> forces = collect_forces(wind).first;
            update_acceleration(forces);
            update_position(dT);
            update_speed(dT);
            update_pitch(dT);
            update_screw_rotation(dT);
        }

        void update_acceleration(std::vector<vec> const &forces) {
            vec total_force;
            for(auto force : forces) {
                total_force += force;
            }
            acceleration_.x = total_force.x / mass_;
            acceleration_.y = total_force.y / mass_;
        }

        void plot_forces(point const &wind) {
            auto forces_names = collect_forces(wind);
            plot_helicopter();
            for (int i = 0; i < forces_names.first.size(); ++i) {
                plot_force(forces_names.first[i], forces_names.second[i]);
            }
            plot_force(velocity_, "velocity");
            plt::grid(true);
            plt::legend();
            plt::show();
        }

        void plot_force(const vec &force, const std::string &name) {
            std::cout << name + ": " << force << '\n';
            std::vector<double> xs = {position_.x, position_.x + force.x};
            std::vector<double> ys = {position_.y, position_.y + force.y};
            plt::named_plot(name, xs, ys);
        }

        void plot_helicopter() {
            double length_x = length_ * std::cos(angle());
            double length_y = length_ * std::sin(angle());
            std::vector<double> xs = {position_.x - length_x / 2, position_.x + length_x / 2};
            std::vector<double> ys = {position_.y - length_y / 2, position_.y + length_y / 2};
            plt::named_plot("helicopter", xs, ys);
        }

    };


}