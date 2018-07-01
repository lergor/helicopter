#include <iostream>
#include "src/engine.h"

using namespace std;
using namespace simulation;

// http://aviadocs.com/RLE/Mi-8/CD1/TO/Mi-8_TO_kn1.pdf
const double MASS = 11100; // kg
const double HEIGHT = 2.34; // m
const double WIDTH = 1.8; // m
const double SCREW_DIAMETER = 21.29; // m

const point POSITION = {1000, 1000};
const double PITCH = -PI / 18 / 2; // rad
const double SCREW_ROTATION = 2 * PI * 192 / 60; // rad/s
const double VELOCITY = 72; // m/s
const point TARGET_POSITION = {0, 0};
const double C_x = 0.15; // coefficient for screw air resistance
const double C_y = 0.55; // coefficient for traction force
const double FILL_FACTOR = 0.0777; // coefficient for traction force


int main() {
    Helicopter helicopter(MASS, WIDTH * HEIGHT, SCREW_DIAMETER, FILL_FACTOR,
                          std::sqrt(C_x * C_x + C_y * C_y), POSITION, PITCH,
                          SCREW_ROTATION, VELOCITY, TARGET_POSITION);
    Engine engine;
    engine.run(helicopter, TARGET_POSITION);
    return 0;
}
