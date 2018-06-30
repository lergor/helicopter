#include <iostream>
#include "src/engine.h"

using namespace std;
using namespace simulation;

const double MASS = 11100; // kg
const double LENGTH = 18.22; // m
const double HEIGHT = 2.34; // m
const double WIDTH = 1.8; // m
const double SCREW_DIAMETER = 21.29; // m

const point POSITION = {1000, 1000};
const double PITCH = -PI / 4; // rad
const double SCREW_ROTATION = 2 * PI * 192 / 60; // rad/s
const double VELOCITY = 72; // m/s
const point TARGET_POSITION = {0, 0};
const double C_R = 0.57; // coefficient for screw aerodynamic force
const double C_x = 0.15; // coefficient for screw air resistance
const double C_y = 0.55; // coefficient for traction force


int main() {
    helicopter helicopter(MASS, SCREW_DIAMETER, C_R, POSITION, PITCH,
                          SCREW_ROTATION, VELOCITY, TARGET_POSITION);
    Engine engine;
    engine.run(helicopter, TARGET_POSITION);
    return 0;
}

