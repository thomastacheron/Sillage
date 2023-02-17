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

// #include <ryml_std.hpp>
// #include <ryml.hpp>

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
    fig.draw_text(fmt::format("{0:.2f} s", t), -2, 100*X[1].diam()/(2*X[0].diam()) - 6, true, ipe::EAlignLeft);
    fig.save_ipe(filename + ".ipe");
}

struct latlon{
    float lat;
    float lon;
};

struct local{
    float x;
    float y;
};

void latlon2local(const latlon& pos0, const latlon& pos, local& loc){
    float rho = 6371009; //m
    loc.x = rho*cos(pos.lat*M_PI/180)*(pos.lon - pos0.lon)*M_PI/180;
    loc.y = rho*(pos.lat - pos0.lat)*M_PI/180;
}


int main(int argc, char *argv[]) {
    // Parsing args
    cxxopts::Options options("05-video", "Video generation of boat's enclosing state using sensors");

    options.add_options()
        // ("c,config", "Configuration file", cxxopts::value<std::string>())
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

    
    // Causal
    bool causal = false;
    
    // Precision
    double precision = 0.05;

    // Boat space
    codac::IntervalVector X0(3);
    std::vector<std::string> state{"x", "y", "v"};

    X0[0] = codac::Interval(-5,70);
    X0[1] = codac::Interval(-25,25);
    X0[2] = codac::Interval(5, 12);

    if (X0[2].lb() <= 0) {
        fmt::print(stderr, "The minimal velocity should be strictly positive !");
        std::exit(EXIT_FAILURE);
    }

    // Time
    double tmin=0.;
    double tmax=11.; // CHANGER

    double h = 1./30.;
    // Boats
    std::size_t n_boats=1;

    // Sensors
    std::size_t n_sensors=4;

    // Path
    std::string path;
    std::filesystem::path output_directory;

    if (!result["path"].as<std::string>().empty()) {
        path = result["path"].as<std::string>();
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
    Boat bn(0, 0, 9);
    boats.push_back(bn);
    // Sensors
    std::vector<Sensor> sensors;
    latlon g1, g2, g3, g4;
    g1.lat = 48.198315;
    g1.lon = -3.01888;
    g2.lat = 48.198315;
    g2.lon = -3.01888;
    g3.lat = 48.19799833333333;
    g3.lon = -3.01895;
    g4.lat = 48.19799833333333;
    g4.lon = -3.01895;
    local l1, l2, l3, l4;
    latlon2local(g1, g1, l1);
    latlon2local(g1, g2, l2);
    latlon2local(g1, g3, l3);
    latlon2local(g1, g4, l4);
    Sensor s1((l1.x) * cos((180-39)*M_PI/180.) - (l1.y) * sin((180-39)*M_PI/180.) + 6, (l1.x + 5) * sin((180-39)*M_PI/180.) + (l1.y + 5) * cos((180-39)*M_PI/180.)  - 15);
    Sensor s2((l2.x) * cos((180-39)*M_PI/180.) - (l2.y) * sin((180-39)*M_PI/180.) + 6, (l2.x + 5) * sin((180-39)*M_PI/180.) + (l2.y + 5) * cos((180-39)*M_PI/180.)  - 15);
    Sensor s3((l3.x) * cos((180-39)*M_PI/180.) - (l3.y) * sin((180-39)*M_PI/180.) + 6, (l3.x + 5) * sin((180-39)*M_PI/180.) + (l3.y + 5) * cos((180-39)*M_PI/180.)  - 15);
    Sensor s4((l4.x) * cos((180-39)*M_PI/180.) - (l4.y) * sin((180-39)*M_PI/180.) + 6, (l4.x + 5) * sin((180-39)*M_PI/180.) + (l4.y + 5) * cos((180-39)*M_PI/180.)  - 15);
    sensors.push_back(s1);
    sensors.push_back(s2);
    sensors.push_back(s3);
    sensors.push_back(s4);
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
    size_t c = 0;
    for(auto && result: results) {
        std::cout << c << "\n";
        c++;
        std::pair<double,std::map<codac::SetValue,std::list<codac::IntervalVector>>> p = result.get();
        draw_ipe(p.first, tmin, tmax, h, scene, p.second, X0, output_directory, verbose);
    }
}