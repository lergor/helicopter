#pragma once

#include "wind_cursor.h"
#include "vec.h"
#include "helicopter.h"
#include "../include/matplotlibcpp.h"

//#define DEBUG

namespace plt = matplotlibcpp;
const std::string PLOT_DIR = "./plots/";

namespace simulation {

    template <typename T>
    std::pair<std::vector<double>, std::vector<double>> split_dots(std::vector<T> const &dots) {
        std::vector<double> xs(dots.size());
        std::vector<double> ys(dots.size());

        std::transform(dots.begin(), dots.end(), xs.begin(), [](T const& p) { return p.x; });
        std::transform(dots.begin(), dots.end(), ys.begin(), [](T const& p) { return p.y; });

        return {xs, ys};
    };

    void plot_dependency(std::vector<double> const &xs, std::vector<double> const &ys,
                         std::string const &x_label, std::string const &y_label) {
        plt::figure();
        plt::plot(xs, ys);
        std::string title = y_label + "(" + x_label + ")";
        plt::title(title);
        plt::grid(true);
        plt::xlabel(x_label);
        plt::ylabel(y_label);
        plt::save(PLOT_DIR + title + ".png");
    };

    void plot_dependencies(std::vector<double> const &times, std::vector<point> const &positions, std::vector<vec> const &speeds,
                               std::vector<double> const &angular_velocities, std::vector<double> const &pitches) {
        plot_dependency(times, split_dots(positions).second, "t", "Z");
        plot_dependency(times, split_dots(speeds).first, "t", "dZ");
        plot_dependency(times, angular_velocities, "t", "w");
        plot_dependency(times, pitches, "t", "p");
    }

    void plot_path(helicopter const &helic, std::vector<double> const &angles,
                   std::vector<point> const &positions) {
        plt::figure();

        auto [xs, ys] = split_dots(positions);

        for (size_t i = 0; i < positions.size(); ++i) {
            double length_x = helic.propeller_radius() * std::cos(angles[i]);
            double length_y = helic.propeller_radius() * std::sin(angles[i]);

            std::vector<double> x_coords = {positions[i].x - length_x / 2, positions[i].x + length_x / 2};
            std::vector<double> y_coords = {positions[i].y - length_y / 2, positions[i].y + length_y / 2};

            plt::plot(x_coords, y_coords);
        }

        plt::plot(xs, ys);
        plt::title("Z(Y)");
        plt::grid(true);
        plt::xlabel("Y");
        plt::ylabel("Z");
        plt::save(PLOT_DIR + "helicopter_path.png");
    }

    class Simulator {
    public:

        Simulator() = default;

        Simulator(point const &target_position, double X_limit, double V_limit)
                : target_position_(target_position),
                  position_limit_(X_limit),
                  velocity_limit_(V_limit) {}

        bool run(helicopter &helicopter, double dT, bool show_plots) {
            auto w_cursor = get_wind_cursor();

            std::vector<double> times;
            std::vector<point> positions;
            std::vector<double> angular_velocities;
            std::vector<vec> speeds;
            std::vector<double> pitches;
            std::vector<double> angles;

            double time = 0;
            int iter = 0;
            bool with_forces = false;
            int max_iterations = static_cast<int>(200 / dT);

            while (helicopter.position_.y > 1 && iter < max_iterations) {
                w_cursor->move(helicopter.position_);
                helicopter.update_state(dT, w_cursor->get_wind());
#ifdef DEBUG
                helicopter.plot_helicopter(w_cursor->get_wind(), with_forces);
#endif
                positions.push_back(helicopter.position_);
                speeds.push_back(helicopter.velocity_);
                angles.push_back(helicopter.angle());
                pitches.push_back(helicopter.pitch_);
                angular_velocities.push_back(helicopter.angular_velocity());

                times.push_back(time);
                time += dT;
                iter += 1;
            }

            if (show_plots) {
                plot_dependencies(times, positions, speeds, angular_velocities, pitches);
                plot_path(helicopter, angles, positions);
                plt::show();
            }

            std::cout << "TOTAL TIME: " << time << " s;" << std::endl;
            std::cout <<"POSITION: " << helicopter.position_ << std::endl;
            std::cout << "VELOCITY: " << helicopter.velocity_ << std::endl;

            return check_landing(helicopter);
        }

        void set_target_position(point const &pos) {
            target_position_ = pos;
        }

        void set_velocity_limits(double limit) {
            velocity_limit_ = limit;
        }

        void set_position_limits(double limit) {
            position_limit_ = limit;
        }

    private:

        point target_position_;
        double velocity_limit_;
        double position_limit_;

        bool check_landing(helicopter const& helicopter) {

            double eps = std::abs(target_position_.x - helicopter.position_.x);
            return helicopter.velocity_.length() < velocity_limit_ && eps < position_limit_;
        }
    };



}