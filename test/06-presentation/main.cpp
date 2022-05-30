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
    codac::IntervalVector X0({{0, 6}, {0, 6}, {-1, 1}});

    // Boats
    std::vector<Boat> boats;
    Boat b_forward(1, 5, 1);
    Boat b_backward(5, 1, -1);
    boats.push_back(b_forward);
    boats.push_back(b_backward);

    // Sensors
    std::vector<Sensor> sensors;
    Sensor s1(2, 4);
    Sensor s2(4, 3);
    sensors.push_back(s1);
    sensors.push_back(s2);

    // Scene
    double precision = 0.1;
    Scene scene(X0, sensors, boats);
    std::string filename("06-presentation");

    // Boat space
    ipegenerator::Figure boat_space(X0.subvector(0, 1), 100, 100*X0[1].diam()/X0[0].diam());
    boat_space.set_graduation_parameters(X0[0].lb(),1,X0[1].lb(),1);
    boat_space.set_number_digits_axis_x(1);
    boat_space.set_number_digits_axis_y(1);
    scene.boat_space(boat_space, 0, precision);
    boat_space.draw_axis("x","y");
    boat_space.save_ipe(filename + "-boat.ipe");
    boat_space.save_pdf(filename + "-boat.pdf");

    // Detection space
    ipegenerator::Figure detection_space(X0.subvector(0, 1), 100, 100*X0[1].diam()/X0[0].diam());
    detection_space.set_graduation_parameters(X0[0].lb(),1,X0[1].lb(),1);
    detection_space.set_number_digits_axis_x(1);
    detection_space.set_number_digits_axis_y(1);
    detection_space.draw_axis("x","y");
    detection_space.save_ipe(filename + "-detection.ipe");
    detection_space.save_pdf(filename + "-detection.pdf");
    
}