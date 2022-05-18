#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <random>

#include <codac.h>

#include "scene.hpp"
#include "boat.hpp"
#include "sensor.hpp"

using namespace std;


int main(int argc, char *argv[]) {
    // Frame of the problem
    codac::IntervalVector X0(3);
    X0[0] = codac::Interval(-25., 25.);
    X0[1] = codac::Interval(-10, 10);
    X0[2] = codac::Interval(-5, 5);

    // Boats
    std::vector<Boat> boats;
    Boat b1(-20, 0, 1);
    Boat b2(-5, -5, 1);
    Boat b3(5, 3, 1);
    Boat b4(15, 8, 1);
    Boat b5(20, -3, 1);
    boats.push_back(b1);
    // boats.push_back(b2);
    // boats.push_back(b3);
    // boats.push_back(b4);
    // boats.push_back(b5);

    // Sensors
    int n_sensors = 2;
    std::vector<Sensor> sensors;

    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<> dist(X0[0].lb(), X0[0].ub());
    // for(int i=0; i<int(n_sensors / 2); i++) {
    //     Sensor sl (dist(gen), -8);
    //     Sensor su (dist(gen), 8);
    //     sensors.push_back(sl);
    //     sensors.push_back(su);
    // }

    Sensor sl (0, -1/tan(19.5*M_PI/180));
    Sensor su (0, 1/tan(19.5*M_PI/180));
    sensors.push_back(sl);
    sensors.push_back(su);

    // Scene
    Scene scene(sensors, boats, X0);

    scene.process();

    scene.detection_space();
    scene.solve();

    // scene.m_sensors[0].save("s1");
}

