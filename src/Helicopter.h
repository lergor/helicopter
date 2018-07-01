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

const double AIR_RESISTANCE_COEFFICIENT_FRONT = 0.04;

namespace simulation {

    double const G = 9.8; // gravity acceleration, m/s^2
    const double rho = 1.2754; // air density, kg/m^3

    struct Helicopter {
        const double mass_;
        const double cabin_square_;
        const double screw_diameter_; // m
        const double screw_square_; // m^2
        const double C_R_;

        point position_;
        point goal_;

        bool moving_left_;
        double pitch_;
        double angular_velocity_;
        vec velocity_;
        vec acceleration_;
        double  fill_factor_;

        Helicopter(double mass, double cabin_square, double screw_diameter, double fill_factor,
                   double C_R, point const &pos, double pitch, double screw_rotation,
                   double velocity = 0, vec const &acceleration = {}, point const &goal = {})
                : mass_(mass),
                  cabin_square_(cabin_square),
                  screw_diameter_(screw_diameter),
                  screw_square_(PI * screw_diameter_ * screw_diameter_ / 4),
                  C_R_(C_R),
                  position_(pos),
                  goal_(goal),
                  moving_left_(goal_.x < position_.x),
                  pitch_(pitch),
                  angular_velocity_(screw_rotation),
                  velocity_({velocity * std::cos(angle()), velocity * std::sin(angle())}),
                  acceleration_(acceleration),
                  fill_factor_(fill_factor) {}

        double angle() {
            return (moving_left_) ? (PI - pitch_) : pitch_;
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
            pitch_ = moving_left_ ? (PI - velocity_.angle()) : velocity_.angle();
            if (pitch_ > PI / 2) {
                pitch_ -= 2 * PI;
            }
        }

        void update_screw_rotation(double dT, const vec &wind) {
            double blade_mass = 50; // kg
            double n = 5; // number of blades
            double c = 0.52; // cord length, m
            double tw = 0.0785; // rad
            double a = 0.5; // lift slope

            double W = angular_velocity_ * screw_diameter_ / 2;

            double phi = std::atan((wind.y + velocity_.y) / W);

            double theta_0 = std::abs(velocity_ * wind) / (velocity_.length() * wind.length());
            double theta_r = theta_0 / 3 - tw / 4 - phi / 2;
            double C_i = phi * c * n * a * theta_r / PI;
            double Q_i = C_i * rho * fill_factor_ * screw_square_ * W * W / 2;
            double force_moment = Q_i * screw_diameter_ / 2;
            double moment_of_inertia = 5 * blade_mass * std::pow(screw_diameter_, 2) / 12;
            double angular_acceleration = force_moment / moment_of_inertia;
            angular_velocity_ += angular_acceleration * dT;
        }

        vec aerodynamic_force(vec const &wind) {
            vec total_flow = wind - velocity_;
            double force = C_R_ * screw_square_ * rho * std::pow(total_flow.y, 2) / 2;
            double angle = wind.angle() + PI / 2;
            return {force * std::cos(angle), force * std::sin(angle)};
        }

        vec air_resistance(vec const &wind) {
            vec total_flow = wind - velocity_;
            double force = AIR_RESISTANCE_COEFFICIENT_FRONT * cabin_square_ * rho * std::pow(total_flow.x, 2) / 2;
            return {force * std::cos(total_flow.angle()), force * std::sin(total_flow.angle())};
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
            std::cout << "position: " << position_ << '\n';
            std::cout << "wind: " << wind << '\n';
            vec total;
            for (int i = 0; i < forces_and_names.first.size(); ++i) {
                total += forces_and_names.first[i];
                std::cout << forces_and_names.second[i] + ": " << forces_and_names.first[i] << '\n';
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
            update_screw_rotation(dT, wind);
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