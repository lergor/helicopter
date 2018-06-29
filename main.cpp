#include <iostream>
#include "src/engine.h"
#include "src/screw.h"

using namespace std;
using namespace simulation;

const double MASS = 11100; // kg
const double LENGTH = 18.22; // m
const double HEIGHT = 2.34; // m
const double WIDTH = 1.8; // m
const double SCREW_DIAMETER = 21.29; // m

const point POSITION = {1000, 1000};
const double PITCH = -PI / 4; // rad
const double SREW_ROTATION = 2 * PI * 192 / 60; // rad/s
const double VELOCITY = 100; // m/s
const point TARGET_POSITION = {0, 0};


int main() {
    helicopter helicopter(MASS, WIDTH, HEIGHT, LENGTH, SCREW_DIAMETER, POSITION,
                          PITCH, SREW_ROTATION, VELOCITY, TARGET_POSITION);
    Engine engine;
    engine.run(helicopter, TARGET_POSITION);
    return 0;
}

