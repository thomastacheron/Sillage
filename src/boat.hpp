#pragma once

#include <ipegenerator/ipegenerator.h>
#include <codac/codac_IntervalVector.h>

#include "utils.hpp"

class Boat {
    public:
        Boat(float x, float y, float v) : m_x(x), m_y(y), m_v(v) {};

        float X() const { return m_x; };
        float Y() const { return m_y; };
        float V() const { return m_v; };

        void draw(ipegenerator::Figure &fig, double t, codac::IntervalVector X) const;

    private:
        float m_x = 0;
        float m_y = 0;
        float m_v = 0;
};

inline void Boat::draw(ipegenerator::Figure &fig, double t, codac::IntervalVector X) const {
    // Wake
    double x_b_u = m_x + m_v * t;
    double y_b_u = m_y;
    double x_b_l = x_b_u;
    double y_b_l = y_b_u;
    double  x_u, y_u, x_l, y_l;
    bool result_u, result_l;

    // Forward boat
    if (sgn(m_v) > 0){
        x_u = X[0].lb();
        x_l = X[0].lb();
    }
    // Backward boat
    else {
        x_u = X[0].ub();
        x_l = X[0].ub();
    }

    y_u = -sgn(m_v)*(x_u - m_x - m_v * t) * std::tan(19.5*M_PI/180) + m_y;
    y_l = sgn(m_v)*(x_l - m_x - m_v * t) * std::tan(19.5*M_PI/180) + m_y;

    result_u = cohenSutherlandClip(X, x_b_u, y_b_u, x_u, y_u);
    result_l = cohenSutherlandClip(X, x_b_l, y_b_l, x_l, y_l);

    if (result_l or result_u) {
        fig.set_line_width(1);
        fig.set_current_layer("wakes");
        fig.set_stroke_opacity(40);
        fig.set_color_stroke("colorBlind1");
    }
    if (result_u) {
        fig.draw_segment(x_b_u, y_b_u, x_u, y_u);
    }
    if (result_l) {
        fig.draw_segment(x_b_l, y_b_l, x_l, y_l);
    }
    
    // Boat
    if (X[0].contains(m_x+m_v*t)) {
        double rot = (m_v > 0) ? 0 : M_PI;
        fig.reset_attribute();
        fig.set_current_layer("boats");
        fig.draw_boat(m_x+m_v*t, m_y, rot, 0.01);
    }
}