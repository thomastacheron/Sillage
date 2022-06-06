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
    codac::IntervalVector X({{0, 6}, {0, 6}, {-1, 1}});

    // Boats
    std::vector<Boat> boats;
    Boat b_forward(1, 4.5, 1);
    Boat b_backward(4, 1, -1);
    boats.push_back(b_forward);
    boats.push_back(b_backward);

    // Sensors
    std::vector<Sensor> sensors;
    Sensor s1(2, 4);
    Sensor s2(4, 2.5);
    sensors.push_back(s1);
    sensors.push_back(s2);

    // Scene
    double precision = 0.01;
    Scene scene(X, sensors, boats);
    std::string filename("06-presentation");

    // Boat space
    ipegenerator::Figure boat_space(X.subvector(0, 1), 100, 100*X[1].diam()/X[0].diam());
    boat_space.set_graduation_parameters(X[0].lb(),1,X[1].lb(),1);
    boat_space.set_number_digits_axis_x(1);
    boat_space.set_number_digits_axis_y(1);
    scene.boat_space(boat_space, 0, precision);
    boat_space.draw_axis("x","y");
    boat_space.save_ipe(filename + "-boat.ipe");
    boat_space.save_pdf(filename + "-boat.pdf");

    // Detection space
    codac::IntervalVector Y{{0, 10}, {0, 10}};
    ipegenerator::Figure detection_space(Y.subvector(0, 1), 100, 100*Y[1].diam()/Y[0].diam());
    detection_space.set_graduation_parameters(Y[0].lb(),2,Y[1].lb(),2);
    detection_space.set_number_digits_axis_x(1);
    detection_space.set_number_digits_axis_y(1);
    detection_space.draw_axis("s_1","s_2");

    // Background
    detection_space.draw_box(Y.subvector(0, 1), "colorBlindOutStroke", "colorBlindOutFill");

    // Projected detection time
    sensors = scene.get_sensors();
    for (const auto &i1: sensors[0].t) {
        // Vertical dotted
        detection_space.set_color_stroke("white");
        detection_space.set_dashed("dotted");
        detection_space.set_line_width(1);
        detection_space.draw_segment(i1.lb(), Y[0].lb(), i1.lb(), Y[0].ub());
        detection_space.draw_segment(i1.ub(), Y[0].lb(), i1.ub(), Y[0].ub());
        for (const auto &i2: sensors[1].t) {
            // Horizontal dotted
            detection_space.set_color_stroke("white");
            detection_space.set_dashed("dotted");
            detection_space.set_line_width(1);
            detection_space.draw_segment(Y[1].lb(), i2.lb(), Y[1].ub(), i2.lb());
            detection_space.draw_segment(Y[1].lb(), i2.ub(), Y[1].ub(), i2.ub());

            // Drawing the detected box
            detection_space.reset_attribute();
            detection_space.draw_box(cart_prod(i1, i2), "colorBlindInStroke", "colorBlindInFill");
            detection_space.draw_box(cart_prod(i1, codac::Interval(0, 0.15)), "colorBlindInStroke", "colorBlindInFill");
            detection_space.draw_box(cart_prod(codac::Interval(0, 0.15), i2), "colorBlindInStroke", "colorBlindInFill");
        }
    }

    // Draw real detection
    detection_space.draw_auv(sensors[0].t[0].mid()-0.1, sensors[1].t[0].mid(), 0, 0.01);
    detection_space.draw_auv(sensors[0].t[1].mid()+0.1, sensors[1].t[1].mid(), M_PI, 0.01);

    detection_space.save_ipe(filename + "-detection.ipe");
    detection_space.save_pdf(filename + "-detection.pdf");
    
}