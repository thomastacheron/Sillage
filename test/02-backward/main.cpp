#include <iostream>
#include <vector>
#include <codac.h>

#include "scene.hpp"
#include "boat.hpp"
#include "sensor.hpp"

using namespace std;


int main(int argc, char *argv[]) {
    // Frame of the problem
    codac::IntervalVector X0({{-10, 10}, {-10, 10}, {-2, 2}});

    // Boat
    std::vector<Boat> boats;
    Boat b1(0, -2, -1);
    boats.push_back(b1);

    // Sensor
    std::vector<Sensor> sensors;
    Sensor s(-2, 0);
    sensors.push_back(s);

    // Scene
    Scene scene(X0, sensors, boats);

    // Figure
    std::string filename = "02-backward";
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
        fig.set_graduation_parameters(X0[0].lb(), 2, X0[1].lb(), 2);
        fig.set_number_digits_axis_x(1);
        fig.set_number_digits_axis_y(1);
        scene.boat_space(fig, 0, 0.1);
        fig.draw_axis("x","y");
        fig.save_ipe(filename + ".ipe");
        fig.save_pdf(filename + ".pdf");
    #endif

    // Detection Time
    Sensor sensor = scene.get_sensors()[0];
    std::cout << "Detection Time : " << sensor.t[0] << std::endl;
}