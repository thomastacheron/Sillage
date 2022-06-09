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
#include "thread_pool.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>

using namespace std;

double step(codac::IntervalVector X, std::vector<Sensor> sensors, std::vector<Boat> boats, double t, double tf, double h, double precision, std::filesystem::path p, bool verbose, bool causal){
    if (verbose) {
        fmt::print(stdout, "Computing time : {0}\n", t);
    }

    Scene scene(X, sensors, boats);
    std::string filename = std::filesystem::absolute(p) / fmt::format("Wake_{0:0>{1}d}", int(t/h), std::to_string(int(tf/h)).size());
    
    ipegenerator::Figure fig(X.subvector(0, 1), 100, 100*X[1].diam()/X[0].diam());
    fig.set_graduation_parameters(X[0].lb(),5,X[1].lb(),5);
    fig.set_number_digits_axis_x(1);
    fig.set_number_digits_axis_y(1);
    scene.boat_space(fig, t, precision, causal);
    fig.draw_axis("x","y");
    fig.reset_attribute();
    fig.draw_text(fmt::format("{0:.1f} s", t), -1, 100*X[1].diam()/(2*X[0].diam()) - 9, true, ipe::EAlignLeft);
    fig.save_ipe(filename + ".ipe");

    return t;
}


int main(int argc, char *argv[]) {
    // Parsing args
    cxxopts::Options options("05-video", "Video generation of boat's enclosing state using sensors");

    options.add_options()
        ("p,path", "Output path", cxxopts::value<std::string>())
        ("d,duration", "Duration of the simulation", cxxopts::value<double>()->default_value("10"))
        ("s,step", "Time step", cxxopts::value<double>()->default_value("0.1"))
        ("precision", "Projection precision", cxxopts::value<double>()->default_value("1"))
        ("v,verbose", "Verbose output", cxxopts::value<bool>()->default_value("false"))
        ("c,causal", "Causal simulation", cxxopts::value<bool>()->default_value("false"))
        ("h,help", "Print usage")
    ;
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        std::exit(EXIT_SUCCESS);
    }

    bool verbose = result["verbose"].as<bool>();
    if (verbose) {
        unsigned int n = std::thread::hardware_concurrency();
        std::cout << n << " concurrent threads are supported.\n";
    }

    std::filesystem::path p(result["path"].as<std::string>());
    if (!std::filesystem::is_directory(p)) {
        fmt::print(stderr, "The provided path does not point to a directory !");
        std::exit(EXIT_FAILURE);
    }

    if (verbose) {
        fmt::print(stdout, "File path {0}\n", p.c_str());
    }

    double tmax = result["duration"].as<double>();
    if (tmax < 0) {
        std::cerr << "Simulation cannot have a negative duration !\n";
        std::exit(EXIT_FAILURE);
    }

    double h = result["step"].as<double>();
    if (h < 0) {
        std::cerr << "Simulation cannot have a negative time-step !\n";
        std::exit(EXIT_FAILURE);
    }
    
    if (h > tmax) {
        std::cerr << "Simulation cannot have a time-step greater than the time of the simulation !\n";
        std::exit(EXIT_FAILURE);
    }

    double precision = result["precision"].as<double>();
    if (precision < 0) {
        std::cerr << "Projection precision cannot be negative !\n";
        std::exit(EXIT_FAILURE);
    }

    if (verbose) {
        fmt::print(stdout, "Duration : {0} s, Time-step {1} s\n", tmax, h);
    }

    bool causal = result["causal"].as<bool>();
    if (causal) {
        fmt::print("Causal simulation\n");
    }

    // Frame of the problem
    codac::IntervalVector X0({{-25, 25}, {-10, 10}, {-6, 6}});

    // Random engine
    std::random_device rd;
    std::mt19937 generator(rd());

    // Boats
    std::vector<Boat> boats;
    unsigned int n_boats = 5;

    std::uniform_real_distribution<double> distribution_bnx(X0[0].lb()-5, X0[0].lb());
    std::uniform_real_distribution<double> distribution_bpx(X0[0].ub(), X0[0].ub()+5);
    std::uniform_real_distribution<double> distribution_by(X0[1].lb(), X0[1].ub());
    std::uniform_real_distribution<double> distribution_bv(1, 5);
    for (int i=0; i<n_boats; ++i) {
        Boat bn(distribution_bnx(generator), distribution_by(generator), distribution_bv(generator));
        Boat bp(distribution_bpx(generator), distribution_by(generator), -distribution_bv(generator));
        boats.push_back(bn);
        boats.push_back(bp);
    }

    // Sensors
    std::uniform_real_distribution<double> distribution_x(X0[0].lb(), X0[0].ub());
    std::uniform_real_distribution<double> distribution_y(X0[1].lb(), X0[1].ub());
    int n_sensors = 50;
    std::vector<Sensor> sensors;
    for (int i=0; i<n_sensors; ++i) {
        Sensor s(distribution_x(generator), distribution_y(generator));
        sensors.push_back(s);
    }

    // Tread Pool
    std::size_t n = std::thread::hardware_concurrency();
    n = 1;
    ThreadPool pool(n);
    std::vector<std::future<double>> results;

    // Time
    std::vector<double> time(int(tmax/h));
    std::generate(time.begin(), time.end(), [counter = 0, h] () mutable { return (counter++)*h; });

    std::vector<std::thread> v_threads;
    for (const auto &t: time) {
        results.emplace_back(
            pool.enqueue(std::bind(step, X0, sensors, boats, t, tmax, h, precision, p, verbose, causal))
        );
    }
}