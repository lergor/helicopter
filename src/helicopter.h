#pragma once

#include <cmath>
#include <vector>
#include "wind_cursor.h"
#include "vec.h"
#include "simulator.h"
#include "../include/matplotlibcpp.h"

namespace plt = matplotlibcpp;


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
        const double phi_0_;
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

    struct helicopter_params {

        const double mass_;
        const double height_;
        const double width_;
        const double C_X_front;

        double cabin_square() {
            return  height_ * width_;
        }
    };

    struct Helicopter {

        Propeller propeller_;
        helicopter_params params_;
        point position_;
        double pitch_;
        vec velocity_;
        bool moving_left_;
        vec acceleration_;

        Helicopter(const Propeller &prop, const helicopter_params &params,
                   point const &pos, double pitch, vec velocity)
                : propeller_(prop),
                  params_(params),
                  position_(pos),
                  pitch_(pitch),
                  velocity_(velocity),
                  moving_left_(velocity_.x < 0),
                  acceleration_({0, 0}) {}

        double angle() {
            double angle = moving_left_ ? (M_PI - pitch_) : pitch_;
            return (std::abs(angle) > 2 * M_PI) ? (angle + sgn(angle) * angle) : angle;
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
            if (pitch_ > M_PI / 2) {
                pitch_ -= 2 * M_PI;
            }
        }

        void update_propeller_rotation(double dT, vec const &wind) {

            double n = propeller_.params_.num_of_blades_;
            double c = propeller_.params_.cord_length_;
            double a = propeller_.params_.lift_slope_;
            double tw = propeller_.params_.phi_0_;
            double m = propeller_.params_.blade_mass_;
            double D = propeller_.params_.propeller_diameter_;
            double R = propeller_.radius();

            double W = propeller_.angular_velocity_ * propeller_.radius();

            double theta_0 = std::abs(velocity_.dot(wind)) / (velocity_.length() * wind.length());
            double theta_r = theta_0 / 3 - tw / 4 - pitch_ / 2;
            double C_i = pitch_ * c * n * a * theta_r / M_PI;
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

        std::vector<vec> collect_forces(vec const &wind) {
            std::vector<vec> forces;

            //gravity_force
            forces.push_back({0, -G * params_.mass_});

            forces.push_back(air_resistance(wind));

            forces.push_back(aerodynamic_force(wind));

            return forces;
        }

        void update_state(double dT, point const &wind) {
            auto forces = collect_forces(wind);
            update_acceleration(forces);
            update_speed(dT);
            update_position(dT);
            update_pitch(dT);
            update_propeller_rotation(dT, wind);
        }

        void update_acceleration(std::vector<vec> const &forces) {
            vec total_force;
            for (auto &force : forces) {
                total_force += force;
            }
            acceleration_.x = total_force.x / params_.mass_;
            acceleration_.y = total_force.y / params_.mass_;
        }

        point position()const {
            return position_;
        }

        double angular_velocity() {
            return propeller_.angular_velocity_;
        }

        vec velocity()const {
            return velocity_;
        }

        double pitch() {
            return pitch_;
        }

        double propeller_radius()const {
            return propeller_.radius();
        }
    };
}
