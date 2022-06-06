#pragma once

#include <ipegenerator/ipegenerator.h>
#include <codac/codac_IntervalVector.h>

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
    std::vector<double> x;
    std::vector<double> y;
    if (m_v < 0) {
        // Upper wake
        double xu = (X[1].ub() - m_y) / std::tan(19.5*M_PI/180) + m_x;
        if (X[0].contains(xu)) {
            x.push_back(xu);
            y.push_back(X[1].ub());
        }
        else {
            x.push_back(X[0].ub());
            y.push_back(m_y + std::tan(19.5*M_PI/180) * (X[0].ub() - m_x));
        }

        // Boat
        x.push_back(m_x + m_v * t);
        y.push_back(m_y);

        // Lower wake
        double xl = - (X[1].lb() - m_y) / std::tan(19.5*M_PI/180) + m_x;
        if (X[0].contains(xl)) {
            x.push_back(xl);
            y.push_back(X[1].lb());
        }
        else {
            x.push_back(X[0].ub());
            y.push_back(m_y - std::tan(19.5*M_PI/180) * (X[0].ub() - m_x));
        }
    }
    else {
        // Upper wake
        double xu = -(X[1].ub() - m_y) / std::tan(19.5*M_PI/180) + m_x;
        if (X[0].contains(xu)) {
            x.push_back(xu);
            y.push_back(X[1].ub());
        }
        else {
            x.push_back(X[0].lb());
            y.push_back(m_y - std::tan(19.5*M_PI/180) * (X[0].lb() - m_x));
        }

        // Boat
        x.push_back(m_x + m_v * t);
        y.push_back(m_y);

        // Lower wake
        double xl = (X[1].lb() - m_y) / std::tan(19.5*M_PI/180) + m_x;
        if (X[0].contains(xl)) {
            x.push_back(xl);
            y.push_back(X[1].lb());
        }
        else {
            x.push_back(X[0].lb());
            y.push_back(m_y + std::tan(19.5*M_PI/180) * (X[0].lb() - m_x));
        }
    }
    fig.set_line_width(1);
    fig.set_stroke_opacity(40);
    fig.set_current_layer("wakes");
    fig.draw_polygon(x, y, "colorBlind1", "", ipegenerator::PATH_TYPE::STROKE_ONLY, false);

    // Boat
    if (X[0].contains(m_x+m_v*t)) {
        double rot = (m_v > 0) ? 0 : M_PI;
        fig.reset_attribute();
        fig.set_current_layer("boats");
        fig.draw_auv(m_x+m_v*t, m_y, rot, 0.01);
    }
}