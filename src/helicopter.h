#pragma once

#include <cmath>
#include <vector>
#include "wind_cursor.h"
#include "vec.h"
#include "simulator.h"
#include "../include/matplotlibcpp.h"

namespace plt = matplotlibcpp;

//#define DEBUG

namespace simulation {

    double const G = 9.8; // gravity acceleration, m/s^2
    const double rho = 1.2754; // air density, kg/m^3

    struct propeller_params {
        const double propeller_diameter_;
        const double C_x_;
        const double C_y_;
        const double blade_mass_;
        const double num_of_blades_;
        const double fill_factor_;
        const double cord_length_;
        const double tw_;
        const double lift_slope_;
    };

    struct Propeller {
        propeller_params params_;
        double angular_velocity_;

        double radius()const {
            return params_.propeller_diameter_ / 2;
        }

        double square() {
            return M_PI * std::pow(radius(), 2);
        }

        double ff_square() {
            return params_.fill_factor_ * square();
        }

        double C_R() {
            return std::sqrt(params_.C_x_ * params_.C_x_ + params_.C_y_ * params_.C_y_);
        }
    };

    struct helicopter_params{
        const double mass_;
        const double height_;
        const double width_;
        const double C_X_front;

        double cabin_square() {
            return  height_ * width_;
        }
    };

    struct helicopter {

        Propeller propeller_;
        helicopter_params params_;
        point position_;
        double pitch_;
        vec velocity_;
        bool moving_left_;
        vec acceleration_;

        helicopter(const Propeller &prop, const helicopter_params &params,
                   point const &pos, double pitch, vec velocity)
                : propeller_(prop),
                  params_(params),
                  position_(pos),
                  pitch_(pitch),
                  velocity_(velocity),
                  moving_left_(velocity_.x < 0),
                  acceleration_({0, 0}) {}

        double angle() {
            return (moving_left_) ? (M_PI - pitch_) : pitch_;
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
            pitch_ = moving_left_ ? (M_PI - velocity_.angle()) : velocity_.angle();
        }

        void update_screw_rotation(double dT, vec const &wind) {

            double n = propeller_.params_.num_of_blades_;
            double c = propeller_.params_.cord_length_;
            double a = propeller_.params_.lift_slope_;
            double tw = propeller_.params_.tw_;
            double m = propeller_.params_.blade_mass_;
            double D = propeller_.params_.propeller_diameter_;
            double R = propeller_.radius();

            double W = propeller_.angular_velocity_ * propeller_.radius();

            double phi = std::atan((wind.y + velocity_.y) / W);

            double theta_0 = std::abs(velocity_.dot(wind)) / (velocity_.length() * wind.length());
            double theta_r = theta_0 / 3 - tw / 4 - phi / 2;
            double C_i = phi * c * n * a * theta_r / M_PI;
            double Q_i = C_i * rho * propeller_.ff_square() * W * W / 2;
            double force_moment = Q_i * R;
            double moment_of_inertia = 5 * m * std::pow(D, 2) / 12;
            double angular_acceleration = force_moment / moment_of_inertia;

            propeller_.angular_velocity_ += angular_acceleration * dT;
        }

        vec aerodynamic_force(vec const &wind) {
            vec total_flow = wind - velocity_;

            double force = propeller_.C_R() * propeller_.square() * rho * std::pow(total_flow.y, 2) / 2;
            double angle = wind.angle() + M_PI / 2;

            return {force * std::cos(angle),
                    force * std::sin(angle)};
        }

        vec air_resistance(vec const &wind) {
            vec total_flow = wind - velocity_;

            double force = params_.C_X_front * params_.cabin_square() * rho * std::pow(total_flow.x, 2) / 2;

            return {force * std::cos(total_flow.angle()),
                    force * std::sin(total_flow.angle())};
        }

        std::pair<std::vector<vec>, std::vector<std::string>> collect_forces(vec const &wind) {
            std::pair<std::vector<vec>, std::vector<std::string>> forces_and_names;

            forces_and_names.first.push_back({0, -G * params_.mass_});
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

        void update_state(double dT, point const &wind) {
            auto forces = collect_forces(wind).first;
            update_acceleration(forces);
            update_speed(dT);
            update_position(dT);
            update_pitch(dT);
            update_screw_rotation(dT, wind);
#ifdef DEBUG
            std::cout << "pitch: " << pitch_ * 180 / M_PI << "\n----\n";
#endif
        }

        void update_acceleration(std::vector<vec> const &forces) {
            vec total_force;
            for (auto &force : forces) {
                total_force += force;
            }
            acceleration_.x = total_force.x / params_.mass_;
            acceleration_.y = total_force.y / params_.mass_;
        }

        point position() {
            return position_;
        }

        double angular_velocity() {
            return propeller_.angular_velocity_;
        }

        vec velocity() {
            return velocity_;
        }

        double pitch() {
            return pitch_;
        }

        double propeller_radius()const {
            return propeller_.radius();
        }

        void plot_force(vec const &force, std::string const &name) {
            std::vector<double> xs = {position_.x, position_.x + force.x};
            std::vector<double> ys = {position_.y, position_.y + force.y};
            plt::named_plot(name, xs, ys);
        }

        void plot_helicopter(vec const &wind, bool with_forces) {

            if (with_forces) {
                auto vv = collect_forces(wind);
                for (size_t i = 0; i < vv.first.size(); ++i) {
                    plot_force(vv.first[i], vv.second[i]);
                }
            }

            double length_x = propeller_radius() * 2 * std::cos(angle());
            double length_y = propeller_radius() * 2 * std::sin(angle());

            std::vector<double> xs = {position_.x - length_x / 2, position_.x + length_x / 2};
            std::vector<double> ys = {position_.y - length_y / 2, position_.y + length_y / 2};

            plt::plot(xs, ys);
            plt::grid(true);
        }

    };


}

//velocity: {dx: -285.294, dy: -20.5173, angle: -175.887}
//wind: {dx: 20, dy: 10, angle: 26.5651}
//gravity_force: {dx: 0, dy: -108780, angle: -90}

//air_resistance: {dx: 9964.2, dy: 996.026, angle: 5.70835}
//aerodynamic_force: {dx: -53902.3, dy: 107805, angle: 116.565}
//total: {dx: -43938.1, dy: 20.5941, angle: 179.973}
//TOTAL TIME: 49.5 s;
//POSITION: {x: -7954.7, y: -2.66911}
//VELOCITY: {dx: -285.294, dy: -20.5173, angle: -175.887}