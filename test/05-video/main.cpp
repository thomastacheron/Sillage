#include <array>
#include <iostream>
#include <random>
#include <vector>

#include <codac.h>

#include "scene.hpp"
#include "boat.hpp"
#include "sensor.hpp"

using namespace std;


int main(int argc, char *argv[]) {
    // Frame of the problem
    codac::IntervalVector X0({{-25, 25}, {-10, 10}, {0.1, 5}});

    // Boats
    std::vector<Boat> boats;
    std::vector<std::vector<double>> b_coords{{-15, 5, 3}, {-5, -5, 2}, {2, 5, 4}, {15, 6, 2}, {20, -3, 5}};
    for (const auto &c: b_coords) {
        Boat b(c[0], c[1], c[2]);
        boats.push_back(b);
    }

    // Sensors
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<double> distribution_x(X0[0].lb(), X0[0].ub());
    std::uniform_real_distribution<double> distribution_y(X0[1].lb(), X0[1].ub());
    int n_sensors = 50;
    std::vector<Sensor> sensors;
    for (int i=0; i<n_sensors; ++i) {
        Sensor s(distribution_x(generator), distribution_y(generator));
        sensors.push_back(s);
    }

    // Time
    double tf = 10;
    double h = 0.1;
    std::vector<double> time(int(tf/h));
    std::generate(time.begin(), time.end(), [n = 0, h] () mutable { return (n++)*h; });

    // Scene
    double precision = 0.1;
    Scene scene(X0, sensors, boats);
    for (const auto &t: time) {
        std::string filename = fmt::format("data/05-video/Wake_{0:0>{1}d}", int(t/h), std::to_string(int(tf/h)).size());
        scene.boat_space(t, precision, filename);
    }
}