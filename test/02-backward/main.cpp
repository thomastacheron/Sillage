#include <iostream>
#include <vector>
#include <codac.h>

#include "scene.hpp"
#include "boat.hpp"
#include "sensor.hpp"

using namespace std;


int main(int argc, char *argv[]) {
    // Frame of the problem
    codac::IntervalVector X0({{-5, 5}, {-5, 5}, {-5, -1}});

    // Boat
    std::vector<Boat> boats;
    Boat b1(0, 0, -2);
    boats.push_back(b1);

    // Sensor
    std::vector<Sensor> sensors;
    Sensor s(2, 0);
    sensors.push_back(s);

    // Scene
    Scene scene(X0, sensors, boats);
    scene.solve(0, 0.1, "02-backward.png");

    // Detection Time
    Sensor sensor = scene.get_sensors()[0];
    std::cout << "Detection Time : " << sensor.t[0] << std::endl;
}