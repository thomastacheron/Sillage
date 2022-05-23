#include <iostream>
#include <string>
#include <vector>

#include <codac.h>
#include <codac-unsupported.h>

#include "scene.hpp"
#include "boat.hpp"
#include "sensor.hpp"

using namespace std;


int main(int argc, char *argv[]) {
    // Frame of the problem
    codac::IntervalVector X0({{-25, 25}, {-10, 10}, {-5, 5}});

    // Boats
    std::vector<Boat> boats;
    Boat b1(0, 0, -3);
    // Boat b1(-15, 5, 3);
    Boat b2(-5, -5, 2);
    Boat b3(5, 3, 4);
    Boat b4(15, 6, 2);
    Boat b5(20, -3, 5);
    boats.push_back(b1);
    boats.push_back(b2);
    // boats.push_back(b3);
    // boats.push_back(b4);
    // boats.push_back(b5);

    // Sensors
    int nx_sensors = 10;
    int ny_sensors = 4;
    std::vector<Sensor> sensors;

    std::vector<std::pair<double, double>> coords{{-13, -13}, {-10, -10}, {-6, -6}, {4, 4}, {8, 8}, {11, 11}};

    double angle = 30*M_PI/180;
    for (const auto &p: coords) {
        Sensor s(p.first*cos(angle)+10, p.second*sin(angle));
        sensors.push_back(s);
    }

    // for(int i=0; i<nx_sensors; ++i) {
    //     for (int j=0; j<ny_sensors; ++j) {
    //         Sensor s(X0[0].lb()+(i+0.5)*X0[0].diam()/nx_sensors, X0[1].lb()+(j+0.5)*X0[1].diam()/ny_sensors);
    //         sensors.push_back(s);
    //     }
    // }

    // Scene
    Scene scene(sensors, boats, X0);

    scene.detection_space(0, 1, 0.01, true);
    scene.solve(0.1);
}