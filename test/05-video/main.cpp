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
    std::vector<std::vector<double>> b_coords{{-22, 6, 5}, {-20, -3, 3}, {-16, -8, 2}, {-14, 3, 4}, {-12, -1, 2}, {8, -4, -3}, {14, 7, -4}, {20, -7, -2}, {23, 0, -5}};
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
    double tf = 30;
    double h = 0.1;
    std::vector<double> time(int(tf/h));
    std::generate(time.begin(), time.end(), [n = 0, h] () mutable { return (n++)*h; });

    // Scene
    double precision = 0.1;
    Scene scene(X0, sensors, boats);
    for (const auto &t: time) {
        std::string filename = fmt::format("data/05-video/Wake_{0:0>{1}d}", int(t/h), std::to_string(int(tf/h)).size());
        #ifdef WITH_VIBES
            vibes::beginDrawing();
            codac::VIBesFig fig("Wake");
            scene.boat_space(fig, t, precision);
            fig.set_properties(600, 260, int(500*X0[0].diam()/X0[1].diam()), 500);
            fig.axis_limits(X0.subvector(0, 1));
            fig.save_image(filename, "png");
        #endif
        #ifdef WITH_IPE
            ipegenerator::Figure fig(X0.subvector(0, 1), 100, 100*X0[1].diam()/X0[0].diam());
            fig.set_graduation_parameters(X0[0].lb(),5,X0[1].lb(),5);
            fig.set_number_digits_axis_x(1);
            fig.set_number_digits_axis_y(1);
            scene.boat_space(fig, t, precision);
            scene.draw_sensors(fig);
            fig.draw_axis("x","y");
            fig.save_ipe(filename + ".ipe");
            fig.save_pdf(filename + ".pdf");
        #endif
    }
}