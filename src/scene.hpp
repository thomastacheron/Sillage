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
            I.inflate(0.25);
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

    // Variable holder
    std::vector<std::shared_ptr<ibex::Sep>> SepBoxes;

    // Sensor 1
    ibex::Array<ibex::Sep> SepBox1(0);
    for (const auto &i: m_sensors[i1].t) {
        window[0] |= i;
        auto Sbi = std::make_shared<codac::SepBox>(codac::IntervalVector(i));
        SepBoxes.push_back(Sbi);
        SepBox1.add(*Sbi);
    }
    ibex::SepUnion Su1(SepBox1);

    // Sensor 2
    ibex::Array<ibex::Sep> SepBox2(0);
    for (const auto &i: m_sensors[i2].t) {
        window[1] |= i;
        auto Sbi = std::make_shared<codac::SepBox>(codac::IntervalVector(i));
        SepBoxes.push_back(Sbi);
        SepBox2.add(*Sbi);
    }
    ibex::SepUnion Su2(SepBox2);

    // Cartesian product of detections
    ibex::Array<ibex::Sep> CartProd{*(m_sensors[i1].sep), *(m_sensors[i1].sep)};
    codac::SepCartProd Scp(CartProd);

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

    std::vector<std::shared_ptr<ibex::Sep>> vec;
    std::vector<std::shared_ptr<ibex::Array<ibex::Sep>>> vec_u;
    ibex::Array<ibex::Sep> cp(0);

    for(const Sensor &s: m_sensors) {
        auto u = std::make_shared<ibex::Array<ibex::Sep>>(0);
        for (const Interval &i : s.t) {
            auto Ci = std::make_shared<codac::SepBox>(codac::IntervalVector(i));
            vec.push_back(Ci);
            u->add(*Ci);
        }
        vec_u.push_back(u);
        auto Cu = std::make_shared<ibex::SepUnion>(*u); 
        vec.push_back(Cu);
        cp.add(*Cu);
    }
    codac::SepCartProd Scp(cp);

    std::string function = "(";
    for (const Sensor &s: m_sensors) {
        std::string fi = fmt::format("1/v*(abs(y-{1})/{2}-(x-{0}));", s.X(), s.Y(), tan(19.5 * M_PI / 180.));
        function += fi;
    }
    function.at(function.size() - 1) = ')';


    ibex::Function f("x", "y", "v", function.c_str());
    ibex::SepInverse Si(Scp, f);

    codac::SepProj Sp(Si, IntervalVector(m_X[2]), precision);

    // Graphics
    vibes::beginDrawing();
    vibes::newFigure("Wake");
    vibes::setFigureProperties("Wake", vibesParams("x", 600, "y", 260, "width", int(500*m_X[0].diam()/m_X[1].diam()), "height", 500));
    vibes::axisLimits(m_X[0].lb(), m_X[0].ub(), m_X[1].lb(), m_X[1].ub());
    codac::IntervalVector X = m_X.subvector(0, 1);
    codac::SIVIA(X, Sp, precision);

    for (auto const &s : m_sensors) {
        vibes::drawCircle(s.X(), s.Y(), 0.2, "black[red]");
    }

    for (auto const &b : m_boats) {
        double rot = (b.V() > 0) ? 0 : 180;
        vibes::drawAUV(b.X(), b.Y(), rot, 1, "black[yellow]");
    }

    if (filename.size() > 0) {
        vibes::saveImage(filename, "Wake");
    }
}