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
    codac::IntervalVector X0({{-25, 25}, {-10, 10}, {-6, 6}});

    // Boats
    std::vector<Boat> boats;
    std::vector<std::vector<double>> b_coords{{-15, 5, 3}, {-10, 3, 2}, {2, 5, 4}, {18, 8, 2}, {20, -3, 5}, {16, -6, -3}, {5, 3, -1}, {-3, 0, -4}};
    for (const auto &c: b_coords) {
        Boat b(c[0], c[1], c[2]);
        boats.push_back(b);
    }

    // Sensors
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<double> distribution_x(X0[0].lb(), X0[0].ub());
    std::uniform_real_distribution<double> distribution_y(X0[1].lb(), X0[1].ub());
    int n_sensors = 40;
    std::vector<Sensor> sensors;
    for (int i=0; i<n_sensors; ++i) {
        Sensor s(distribution_x(generator), distribution_y(generator));
        sensors.push_back(s);
    }

    // Scene
    Scene scene(X0, sensors, boats);

    // Figure
    std::string filename = "04-complex";
    #ifdef WITH_VIBES
        vibes::beginDrawing();
        codac::VIBesFig fig("Wake");
        scene.boat_space(fig, 0, 0.1);
        fig.set_properties(600, 260, int(500*X0[0].diam()/X0[1].diam()), 500);
        fig.axis_limits(X0.subvector(0, 1));
        fig.save_image(filename, "png");
    #endif
    #ifdef WITH_IPE
        ipegenerator::Figure fig(X0.subvector(0, 1), 100, 100*X0[1].diam()/X0[0].diam());
        fig.set_graduation_parameters(X0[0].lb(), 5, X0[1].lb(), 5);
        fig.set_number_digits_axis_x(1);
        fig.set_number_digits_axis_y(1);
        scene.boat_space(fig, 0, 0.1);
        fig.draw_axis("x","y");
        fig.save_ipe(filename + ".ipe");
        fig.save_pdf(filename + ".pdf");
    #endif
}