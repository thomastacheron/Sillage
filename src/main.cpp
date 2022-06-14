#include <array>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
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

#include <ryml_std.hpp>
#include <ryml.hpp>

#include "PoissonGenerator.h"

using namespace std;

void draw_ipe(double t, double tmin, double tmax, double h, Scene &scene, std::map<codac::SetValue,std::list<codac::IntervalVector>> &M, codac::IntervalVector &X, std::filesystem::path &p, bool verbose = false) {
    std::string filename = std::filesystem::absolute(p) / fmt::format("Wake_{0:0>{1}d}", int((t-tmin)/h), std::to_string(int((tmax-tmin)/h)).size());
   
    if (verbose) {
        fmt::print(stdout,"Saving time : {0}\n", t);
    }

    ipegenerator::Figure fig(X.subvector(0, 1), 100, 100*X[1].diam()/X[0].diam());
    fig.set_graduation_parameters(X[0].lb(),5,X[1].lb(),5);
    fig.set_number_digits_axis_x(1);
    fig.set_number_digits_axis_y(1);
    scene.boat_space(M, fig, t);
    fig.draw_axis("x","y");
    fig.reset_attribute();
    fig.draw_text(fmt::format("{0:.1f} s", t), -1, 100*X[1].diam()/(2*X[0].diam()) - 9, true, ipe::EAlignLeft);
    fig.save_ipe(filename + ".ipe");
}

int main(int argc, char *argv[]) {
    // Parsing args
    cxxopts::Options options("05-video", "Video generation of boat's enclosing state using sensors");

    options.add_options()
        ("c,config", "Configuration file", cxxopts::value<std::string>())
        ("p,path", "Output path", cxxopts::value<std::string>()->default_value(""))
        ("v,verbose", "Verbose output", cxxopts::value<bool>()->default_value("false"))
        ("h,help", "Print usage")
    ;
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        std::exit(EXIT_SUCCESS);
    }

    bool verbose = result["verbose"].as<bool>();

    std::filesystem::path p(result["config"].as<std::string>());
    if (!std::filesystem::exists(p)) {
        fmt::print(stderr, "The provided path does not point to config file !");
        std::exit(EXIT_FAILURE);
    }

    std::ifstream stream(p.c_str(), std::ios::binary);
    std::stringstream buffer;
    buffer << stream.rdbuf();
    ryml::Tree tree = ryml::parse_in_arena(ryml::to_csubstr((buffer.str()).c_str()));
    ryml::NodeRef root = tree.rootref();

    // Causal
    bool causal = false;
    if (root["causal"].is_keyval()) {
        root["causal"] >> causal;
    }

    // Precision
    double precision;
    if (root["precision"].is_keyval()) {
        root["precision"] >> precision;
    }
    else {
        fmt::print(stderr, "No provided precision !");
        std::exit(EXIT_FAILURE);
    }

    // Boat space
    codac::IntervalVector X0(3);
    std::vector<std::string> state{"x", "y", "v"};
    std::for_each(state.cbegin(), state.cend(),
        [&X0, &root, i=0](std::string s) mutable {
            if (root[ryml::to_csubstr(s)].is_seq()) {
                double lb, ub;
                root[ryml::to_csubstr(s)][0] >> lb;
                root[ryml::to_csubstr(s)][1] >> ub;
                X0[i] = codac::Interval(lb, ub);
            }
            ++i;
        });

    // Time
    double tmin;
    double tmax;
    if (root["t"].is_seq()) {
        root["t"][0] >> tmin;
        root["t"][1] >> tmax;
    }
    else {
        fmt::print(stderr, "No provided time data !");
        std::exit(EXIT_FAILURE);
    }

    double h;
    if (root["h"].is_keyval()) {
        root["h"] >> h;
    }
    else {
        fmt::print(stderr, "No provided time step !");
        std::exit(EXIT_FAILURE);
    }

    // Boats
    std::size_t n_boats;
    if (root["n_boats"].is_keyval()) {
        root["n_boats"] >> n_boats;
    }
    else {
        fmt::print(stderr, "No provided number of boats !");
        std::exit(EXIT_FAILURE);
    }

    // Sensors
    std::size_t n_sensors;
    if (root["n_sensors"].is_keyval()) {
        root["n_sensors"] >> n_sensors;
    }
    else {
        fmt::print(stderr, "No provided number of sensors !");
        std::exit(EXIT_FAILURE);
    }

    // Path
    std::string path;
    std::filesystem::path output_directory;

    if (!result["path"].as<std::string>().empty()) {
        path = result["path"].as<std::string>();
    }
    else if (root["path"].is_keyval()) {
        root["path"] >> path;
    }
    else {
        fmt::print(stderr, "No provided output directory !");
        std::exit(EXIT_FAILURE);
    }

    output_directory = std::filesystem::path(path);
    if (!std::filesystem::is_directory(output_directory)) {
        fmt::print(stderr, "The provided path does not point to a directory !");
        std::exit(EXIT_FAILURE);
    }

    if (verbose) {
        std::cout << "Output directory: " << output_directory << std::endl;
        std::cout << "Boats: " << n_boats << " Sensors: " << n_sensors << std::endl;
        std::cout << "Precision: " << precision << " Causal: " << causal << std::endl;
        std::cout << "Time: [" << tmin << ", " << tmax << "] s, h: " << h << " s" << std::endl;
        std::cout << "Boat space: " << X0 << std::endl;
        std::cout << std::thread::hardware_concurrency() << " concurrent threads are supported\n";
    }
    
    // Random engine
    std::random_device rd;
    std::mt19937 generator(rd());

    // Boats
    std::vector<Boat> boats;

    std::uniform_real_distribution<double> distribution_bnx(X0[0].lb()-5, X0[0].lb());
    std::uniform_real_distribution<double> distribution_bpx(X0[0].ub(), X0[0].ub()+5);
    std::uniform_real_distribution<double> distribution_by(0.8*X0[1].lb(), 0.8*X0[1].ub());
    std::uniform_real_distribution<double> distribution_bv(1.5, 5.5);
    for (int i=0; i<n_boats; ++i) {
        Boat bn(distribution_bnx(generator), distribution_by(generator), distribution_bv(generator));
        Boat bp(distribution_bpx(generator), distribution_by(generator), -distribution_bv(generator));
        boats.push_back(bn);
        boats.push_back(bp);
    }

    // Sensors
    PoissonGenerator::DefaultPRNG PRNG;
    const auto Points = PoissonGenerator::generatePoissonPoints(n_sensors, PRNG, false);
    std::vector<Sensor> sensors;
    for (auto i = Points.begin(); i != Points.end(); i++) {
        double x = i->x * X0[0].diam() + X0[0].lb();
        double y = i->y * X0[1].diam() + X0[1].lb();
        Sensor s(x, y);
        sensors.push_back(s);
    }

    // Tread Pool
    std::size_t n = std::thread::hardware_concurrency();
    ThreadPool pool(n);
    std::vector<std::future<std::pair<double,std::map<codac::SetValue,std::list<codac::IntervalVector>>>>> results;

    // Time
    Scene scene(X0, sensors, boats);
    std::vector<double> time(int((tmax-tmin)/h));
    std::generate(time.begin(), time.end(), [counter=0, tmin, h] () mutable { return tmin + (counter++) * h; });

    std::vector<std::thread> v_threads;
    for (const auto &t: time) {
        results.emplace_back(
            pool.enqueue(std::bind(&Scene::solve, &scene, t, precision, causal))
        );
    }

    for(auto && result: results) {
        std::pair<double,std::map<codac::SetValue,std::list<codac::IntervalVector>>> p = result.get();
        draw_ipe(p.first, tmin, tmax, h, scene, p.second, X0, output_directory, verbose);
    }
}