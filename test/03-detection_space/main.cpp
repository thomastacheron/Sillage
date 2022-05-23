#include <iostream>
#include <vector>
#include <codac.h>

#include "scene.hpp"
#include "boat.hpp"
#include "sensor.hpp"

using namespace std;


int main(int argc, char *argv[]) {
    // Frame of the problem
    codac::IntervalVector X0({{-5, 5}, {-5, 5}, {1, 5}});

    // Boat
    std::vector<Boat> boats;
    Boat b1(0, 0, 2);
    boats.push_back(b1);

    // Sensors
    std::vector<Sensor> sensors;
    Sensor s1(2, 0);
    sensors.push_back(s1);
    Sensor s2(4, 3);
    sensors.push_back(s2);

    // Scene
    Scene scene(X0, sensors, boats);
    scene.detection_space(0, 1, true);
}