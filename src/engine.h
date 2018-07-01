#pragma once

#include "wind_cursor.h"
#include "vec.h"
#include "helicopter.h"
#include "../include/matplotlibcpp.h"

#define DEBUG

namespace plt = matplotlibcpp;
const std::string PLOT_DIR = "../plots/";

namespace simulation {

    template <typename T>
    std::pair<std::vector<double>, std::vector<double>> split_dots(std::vector<T> const &dots) {
        std::vector<double> xs(dots.size()), ys(dots.size());

        std::transform(dots.begin(), dots.end(), xs.begin(), [](T const& p) { return p.x; });
        std::transform(dots.begin(), dots.end(), ys.begin(), [](T const& p) { return p.y; });

        return {xs, ys};
    };

    template <typename T, typename U>
    void plot_dependency(std::vector<T> const &xs, std::vector<U> const &ys,
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

    class Engine {

    public:

        bool run(helicopter &helicopter, point const &goal) {
            wind_cursor w_cursor = wind_cursor();

            std::vector<double> times;
            std::vector<point> positions;
            std::vector<double> angles;
            std::vector<vec> speeds;
            std::vector<double> pitches;
            std::vector<double> screw_rotations;

            double time = 0;
            int iter = 0;
            bool with_forces = true;
            while (helicopter.position_.y > 1 && iter < 500 / dT) {
                w_cursor.move(helicopter.position_);
                helicopter.update_state(w_cursor.get_wind(), dT);
#ifdef DEBUG
                helicopter.plot_helicopter(w_cursor.get_wind(), with_forces);
#endif
                positions.push_back(helicopter.position_);
                angles.push_back(helicopter.angle());
                speeds.push_back(helicopter.velocity_);
                pitches.push_back(helicopter.pitch_);
                screw_rotations.push_back(helicopter.screw_rotation_);

                times.push_back(time);
                time += dT;
                iter+= 1;
            }

            plot_dependencies(times, positions, speeds, pitches, screw_rotations);
            plot_path(helicopter.screw_diameter_ / 2, angles, positions);
            plt::show();

            std::cout << "TOTAL TIME: " << time << " s;" << std::endl;
            std::cout <<"POSITION: " << helicopter.position_ << std::endl;
            std::cout << "VELOCITY: " << helicopter.velocity_ << std::endl;
            return check_landing(goal, helicopter, 0.5, 0.5);
        }

    private:
        double const dT = 0.5;

        bool check_landing(point const &goal, helicopter const& helicopter,
                           double velocity_limit, double position_limit) {
            double eps = std::abs(goal.x - helicopter.position_.x);
            return helicopter.velocity_.length() < velocity_limit
                   && helicopter.position_.y <= 0
                   && eps < position_limit;
        }

        void plot_dependencies(std::vector<double> const &times, std::vector<point> const &positions,
                               std::vector<vec> const &speeds, std::vector<double> const &pitches,
                               std::vector<double> const &screw_rotations) {
            plot_dependency(times, split_dots(positions).second, "t", "Z");
            plot_dependency(times, split_dots(speeds).first, "t", "dZ");
            plot_dependency(times, screw_rotations, "t", "w");
            plot_dependency(times, pitches, "t", "p");
        }

        void plot_path(double length, std::vector<double> const &angles, std::vector<point> const &positions) {
            plt::figure();
            auto [xs, ys] = split_dots(positions);
            for (int i = 0; i < positions.size(); ++i) {
                double length_x = length * std::cos(angles[i]);
                double length_y = length * std::sin(angles[i]);
                std::vector<double> xs = {positions[i].x - length_x / 2, positions[i].x + length_x / 2};
                std::vector<double> ys = {positions[i].y - length_y / 2, positions[i].y + length_y / 2};
                plt::plot(xs, ys);
            }
            plt::plot(xs, ys);
            plt::title("Z(Y)");
            plt::grid(true);
            plt::xlabel("Y");
            plt::ylabel("Z");
            plt::save(PLOT_DIR + "helicopter_path.png");
        }
    };

}