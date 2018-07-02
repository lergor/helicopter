#include <iostream>
#include "src/simulator.h"

using namespace std;
using namespace simulation;


const double MASS = 11100; // kg
const double HEIGHT = 2.34; // m
const double WIDTH = 1.8; // m
const double FRONT_AIR_RESISTANCE_COEFFICIENT = 0.04;

const double PROPELLER_DIAMETER = 21.29; // m
double BLADE_MASS = 50; // kg
double NUM_OF_BLADES = 5;
const double FILL_FACTOR = 0.0777; // coefficient for traction force
double CORD_LENGTH = 0.52; // m
double PHI_0 = 0.0785; // fixed angle, rad
double LIFT_SLOPE = 0.5;
const double PROPELLER_ROTATION = 2 * M_PI * 192 / 60; // rad/s
const double C_x = 0.15; // coefficient for propeller air resistance
const double C_y = 0.55; // coefficient for traction force

const point POSITION = {1000, 1000};
const double PITCH = -M_PI / 10; // rad
const vec VELOCITY = {90, -6}; // m/s
const point TARGET_POSITION = {0, 0};
const double dT = 0.5;
const double X_limit = 40;
const double V_limit = 20;

const bool SHOW_PLOTS = true;


int main() {
    propeller_params p_params = {PROPELLER_DIAMETER, C_x, C_y, BLADE_MASS, NUM_OF_BLADES,
                                         FILL_FACTOR, CORD_LENGTH, PHI_0, LIFT_SLOPE};

    helicopter_params h_params = {MASS, HEIGHT, WIDTH, FRONT_AIR_RESISTANCE_COEFFICIENT};

    Helicopter helicopter({p_params, PROPELLER_ROTATION}, h_params,
                          POSITION, PITCH, VELOCITY);

    Simulator engine = Simulator(TARGET_POSITION, X_limit, V_limit);

    bool landing = engine.run(helicopter, dT, SHOW_PLOTS);
    std::cout << (landing ? "SUCCESS" : "FAIL") << std::endl;
    return 0;
}
