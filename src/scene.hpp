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
        Scene(std::vector<Sensor> sensors, std::vector<Boat> boats, codac::IntervalVector X0) : m_boats(boats), m_sensors(sensors), m_X(X0) {};
        
        inline void solve(double precision = 1, bool save = false, std::string filename = "");
        inline void detection_space(double precision = 1);

        inline void set_sensors(std::vector<Sensor> sensors);
        inline void set_boats(std::vector<Boat> boats);

    private:
        codac::IntervalVector m_X;

        std::vector<Boat> m_boats;
        std::vector<Sensor> m_sensors;

        bool m_dirty = true;

        inline void process();
};

// Implementation
void Scene::set_sensors(std::vector<Sensor> sensors) {
    m_dirty = true;
    m_sensors = sensors;
}

void Scene::set_boats(std::vector<Boat> boats) {
    m_dirty = true;
    m_boats = boats;
}

void Scene::process() {
    for (auto &s : m_sensors) {
        for (const auto &b: m_boats) {
            double t = 1 / b.V() * (abs(b.Y() - s.Y()) / tan(19.5 * M_PI / 180.) - (b.X() - s.X()));
            codac::Interval I(t);
            I.inflate(0.25);
            s.t.push_back(I);
        }
    }
}

void Scene::detection_space(double precision) {
    if (m_sensors.size() < 2) {
        std::cout << "Not enough sensors\n";
        return;
    }

    if (m_dirty) {
        process();
    }

    IntervalVector window (IntervalVector::empty(2));

    std::vector<std::shared_ptr<ibex::Sep>> vec;
    std::vector<std::shared_ptr<ibex::Array<ibex::Sep>>> vec_u;
    ibex::Array<ibex::Sep> cp(0);

    for(auto s = std::begin(m_sensors); s < std::begin(m_sensors)+2; ++s) {
        auto u = std::make_shared<ibex::Array<ibex::Sep>>(0);
        for (const auto &i : s->t) {
            window[std::distance(m_sensors.begin(), s)] |= i;
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

    vibes::beginDrawing();
    vibes::newFigure("Detection Space");
    vibes::setFigureProperties("Detection Space", vibesParams("x", 100, "y", 300, "width", 500, "height", 500));
    vibes::axisLimits(window[0].lb(), window[0].ub(), window[1].lb(), window[1].ub());
    window.inflate(2);
    codac::SIVIA(window, Scp, precision);
    for (const auto &b: m_boats) {
        double t1 = 1 / b.V() * (abs(b.Y() - m_sensors[0].Y()) / tan(19.5 * M_PI / 180.) - (b.X() - m_sensors[0].X()));
        double t2 = 1 / b.V() * (abs(b.Y() - m_sensors[1].Y()) / tan(19.5 * M_PI / 180.) - (b.X() - m_sensors[1].X()));
        vibes::drawCircle(t1, t2, 0.15, "black[red]");
    }
}

void Scene::solve(double precision, bool save, std::string filename) {

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

    vibes::beginDrawing();
    vibes::newFigure("Wake");
    codac::IntervalVector X = m_X.subvector(0, 1);
    codac::SIVIA(X, Sp, precision);

    for (auto const &s : m_sensors) {
        vibes::drawPoint(s.X(), s.Y(), 5, "black[red]");
    }

    for (auto const &b : m_boats) {
        double rot = (b.V() > 0) ? 0 : 180;
        vibes::drawAUV(b.X(), b.Y(), rot, 1, "black[yellow]");
    }

    vibes::setFigureProperties("Wake", vibesParams("x", 600, "y", 260, "width", int(500*m_X[0].diam()/m_X[1].diam()), "height", 500));
    vibes::axisLimits(m_X[0].lb(), m_X[0].ub(), m_X[1].lb(), m_X[1].ub());

    if (save) {
        vibes::saveImage(filename, "Wake");
    }
}