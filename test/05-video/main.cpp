#include <array>
#include <iostream>
#include <random>
#include <vector>

#include <codac.h>

#include "scene.hpp"
#include "boat.hpp"
#include "sensor.hpp"

#include <cxxopts.hpp>
#include <filesystem>
#include <thread>

using namespace std;

void step(Scene scene, codac::IntervalVector X, double t, double tf, double h, double precision, std::filesystem::path p){
    std::string filename = std::filesystem::absolute(p) / fmt::format("Wake_{0:0>{1}d}", int(t/h), std::to_string(int(tf/h)).size());
    #ifdef WITH_VIBES
        vibes::beginDrawing();
        codac::VIBesFig fig("Wake");
        scene.boat_space(fig, t, precision);
        fig.set_properties(600, 260, int(500*X[0].diam()/X[1].diam()), 500);
        fig.axis_limits(X.subvector(0, 1));
        fig.save_image(filename, "png");
    #endif
    #ifdef WITH_IPE
        ipegenerator::Figure fig(X.subvector(0, 1), 100, 100*X[1].diam()/X[0].diam());
        fig.set_graduation_parameters(X[0].lb(),5,X[1].lb(),5);
        fig.set_number_digits_axis_x(1);
        fig.set_number_digits_axis_y(1);
        scene.boat_space(fig, t, precision);
        scene.draw_sensors(fig, t);
        fig.draw_axis("x","y");
        fig.save_ipe(filename + ".ipe");
        fig.save_pdf(filename + ".pdf");
    #endif
}


int main(int argc, char *argv[]) {
    // Parsing args
    cxxopts::Options options("05-video", "Video generation of boat's enclosing state using sensors");

    options.add_options()
        ("p,path", "Output path", cxxopts::value<std::string>())
        ("v,verbose", "Verbose output", cxxopts::value<bool>()->default_value("false"))
        ("h,help", "Print usage")
    ;
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        std::exit(EXIT_SUCCESS);
    }

    std::filesystem::path p(result["path"].as<std::string>());
    if (!std::filesystem::is_directory(p)) {

        std::exit(EXIT_FAILURE);
    }

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

    // double precision = 1;
    // Scene scene(X0, sensors, boats);
    // for (const auto &t: time) {
    //     step(scene, X0, t, tf, h, precision, p);
    // }

    double precision = 0.1;
    Scene scene(X0, sensors, boats);
    for (const auto &t: time) {
        std::string filename = std::filesystem::absolute(p) / fmt::format("Wake_{0:0>{1}d}", int(t/h), std::to_string(int(tf/h)).size());
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
            scene.draw_sensors(fig, t);
            fig.draw_axis("x","y");
            fig.save_ipe(filename + ".ipe");
            fig.save_pdf(filename + ".pdf");
        #endif
    }
}