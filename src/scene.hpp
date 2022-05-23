#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <string>
#include <fmt/core.h>

#include "sensor.hpp"
#include "boat.hpp"

#include <codac-unsupported.h>
#include <codac/codac_sivia.h>
#include <codac/codac_SepCartProd.h>
#include <codac/codac_SepBox.h>

#include <ibex/ibex_SepUnion.h>
#include <ibex/ibex_SepInverse.h>

void drawBoat(double cx = 0, double cy = 0, double theta = 0, double scale = 0, std::string params = "") {
    std::vector<double> x;
    std::vector<double> y;
    int n = 20;
    double angle = M_PI_4;
    for (int k=0; k<=n; ++k) {
        x.push_back(cos(M_PI_2 - k*angle/n));
        y.push_back(sin(M_PI_2 - k*angle/n) - sin(angle));
    }
    for (int k=0; k<=n; ++k) {
        x.push_back(cos((n-k)*angle/n - M_PI_2));
        y.push_back(sin((n-k)*angle/n - M_PI_2) + sin(angle));
    }
    x.push_back(-0.5);
    y.push_back(-1+sin(angle));
    x.push_back(-0.5);
    y.push_back(1-sin(angle));

    std::vector<double> px;
    std::vector<double> py;
    for (int k=0; k<x.size(); ++k) {
        px.push_back(scale * (cos(theta) * x[k] + sin(theta) * y[k]) + cx);
        py.push_back(scale * (sin(theta) * x[k] + cos(theta) * y[k]) + cy);
    }
    vibes::drawPolygon(px, py, params);
}


class Scene {
    public:
        Scene(codac::IntervalVector X0, std::vector<Sensor> sensors, std::vector<Boat> boats) : m_X(X0), m_sensors(sensors), m_boats(boats) {};
        
        inline void solve(double t = 0, double precision = 1, std::string filename = "");
        inline void detection_space(std::size_t i1, std::size_t i2, double precision = 1, bool show_truth = false);

        inline void set_sensors(const std::vector<Sensor> sensors);
        inline void set_boats(const std::vector<Boat> boats);

        inline std::vector<Sensor> get_sensors() const;
        inline std::vector<Boat> get_boats() const;

    private:
        // State space
        codac::IntervalVector m_X;

        // Sensors vector
        std::vector<Sensor> m_sensors;

        // Boats vector
        std::vector<Boat> m_boats;

        // Dirty flag
        bool m_dirty = true;

        // Process function to recompute detection time when the dirty flag is up
        inline void process();
};

// Implementation
void Scene::set_sensors(const std::vector<Sensor> sensors) {
    m_dirty = true;
    m_sensors = sensors;
}

void Scene::set_boats(const std::vector<Boat> boats) {
    m_dirty = true;
    m_boats = boats;
}

std::vector<Sensor> Scene::get_sensors() const {
    return m_sensors;
}

std::vector<Boat> Scene::get_boats() const {
    return m_boats;
}

void Scene::process() {
    for (auto &s : m_sensors) {
        for (const auto &b: m_boats) {
            // Detection Interval
            double t = 1 / b.V() * (abs(b.Y() - s.Y()) / tan(19.5 * M_PI / 180.) - (b.X() - s.X()));
            codac::Interval I(t);
            I.inflate(0.5);
            s.t.push_back(I);

            // SepBox
            auto SepBoxI =  std::make_shared<codac::SepBox>(codac::IntervalVector(I));
            s.SepBoxes.push_back(SepBoxI);
            s.ArraySepBox->add(*SepBoxI);
        }
        s.sep = make_shared<ibex::SepUnion>(*(s.ArraySepBox));
    }
    m_dirty = false;
}

void Scene::detection_space(std::size_t i1, std::size_t i2, double precision, bool show_truth) {
    if (m_dirty) {
        process();
    }

    // View window
    IntervalVector window (IntervalVector::empty(2));
    for (const auto &i: m_sensors[i1].t) {
        window[0] |= i;
    }
    for (const auto &i: m_sensors[i2].t) {
        window[1] |= i;
    }

    // Cartesian product of detections
    codac::SepCartProd Scp(*(m_sensors[i1].sep), *(m_sensors[i2].sep));

    // Graphics
    window.inflate(2);
    vibes::beginDrawing();
    vibes::newFigure("Detection Space");
    vibes::setFigureProperties("Detection Space", vibesParams("x", 100, "y", 300, "width", 500, "height", 500));
    vibes::axisLimits(window[0].lb(), window[0].ub(), window[1].lb(), window[1].ub());
    codac::SIVIA(window, Scp, precision);

    // Drawing the real points
    if (show_truth) {
        for (const auto &b: m_boats) {
            double t1 = 1 / b.V() * (abs(b.Y() - m_sensors[i1].Y()) / tan(19.5 * M_PI / 180.) - (b.X() - m_sensors[i1].X()));
            double t2 = 1 / b.V() * (abs(b.Y() - m_sensors[i2].Y()) / tan(19.5 * M_PI / 180.) - (b.X() - m_sensors[i2].X()));
            vibes::drawCircle(t1, t2, 0.15, "black[red]");
        }
    }
}

void Scene::solve(double t, double precision, std::string filename) {
    if (m_dirty) {
        process();
    }

    // Cartesian product between detected times
    ibex::Array<ibex::Sep> cp(0);
    for(const Sensor &s: m_sensors) {
        cp.add(*(s.sep));
    }
    codac::SepCartProd Scp(cp);

    // Function used in set inversion
    std::string function = "(";
    for (const Sensor &s: m_sensors) {
        std::string fi = fmt::format("1/v*(abs(y-{1})/{2}-(x-{0}))+{3};", s.X(), s.Y(), tan(19.5 * M_PI / 180.), t);
        function += fi;
    }
    function.at(function.size() - 1) = ')';

    // Inversion of the separator on sensor's detected times
    ibex::Function f("x", "y", "v", function.c_str());
    ibex::SepInverse Si(Scp, f);

    // Projection of the separator along x and y given a v
    codac::SepProj Sp(Si, IntervalVector(m_X[2]), precision);

    // Graphics
    vibes::beginDrawing();
    vibes::newFigure("Wake");
    vibes::setFigureProperties("Wake", vibesParams("x", 600, "y", 260, "width", int(500*m_X[0].diam()/m_X[1].diam()), "height", 500));
    vibes::axisLimits(m_X[0].lb(), m_X[0].ub(), m_X[1].lb(), m_X[1].ub());
    codac::IntervalVector X = m_X.subvector(0, 1);
    codac::SIVIA(X, Sp, precision);

    // Showing sensors
    for (auto const &s : m_sensors) {
        vibes::drawCircle(s.X(), s.Y(), 0.2, "black[red]");
    }

    // Showing boats
    for (auto const &b : m_boats) {
        double rot = (b.V() > 0) ? 0 : M_PI;
        drawBoat(b.X(), b.Y(), rot, 1, "black[#34495e]");
        drawBoat(b.X()+b.V()*t, b.Y(), rot, 1, "black[yellow]");
    }

    // Saving the figure
    if (filename.size() > 0) {
        vibes::setFigureProperties("Wake", vibesParams("x", 600, "y", 260, "width", int(500*m_X[0].diam()/m_X[1].diam()), "height", 500));
        vibes::axisLimits(m_X[0].lb(), m_X[0].ub(), m_X[1].lb(), m_X[1].ub());
        vibes::saveImage(filename, "Wake");
    }
}