#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include <codac.h>

#include "scene.hpp"
#include "boat.hpp"
#include "sensor.hpp"

using namespace std;


int main(int argc, char *argv[]) {
    // Boats
    std::vector<Boat> boats;
    Boat b1(0, 5, 1);
    Boat b2(20, 7, -1);
    boats.push_back(b1);
    boats.push_back(b2);

    // Sensors
    std::vector<Sensor> sensors;
    Sensor s1(5, 0);
    Sensor s2(10, 10);
    sensors.push_back(s1);
    sensors.push_back(s2);

    // Scene
    codac::IntervalVector X0(3);
    X0[0] = codac::Interval(0., 100.);
    X0[1] = codac::Interval(0., 100.);
    X0[2] = codac::Interval(-10., 10.);
    Scene scene(sensors, boats, X0);

    scene.process();

    scene.solve();

    // scene.m_sensors[0].save("s1");
}

