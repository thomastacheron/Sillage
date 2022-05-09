#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <string>
#include <fmt/core.h>

#include "sensor.hpp"
#include "boat.hpp"
#include "sivia.h"

#include <codac-unsupported.h>
#include <codac/codac_sivia.h>

class Scene {
    public:
        Scene(std::vector<Sensor> sensors, std::vector<Boat> boats, codac::IntervalVector X0) : m_boats(boats), m_sensors(sensors), m_X(X0) {};

        inline void process();
        inline void solve(double precision = 0.1);

        std::vector<Boat> m_boats;
        std::vector<Sensor> m_sensors;

    private:
        std::vector<codac::IntervalVector> m_in;
        std::vector<codac::IntervalVector> m_may;
        std::vector<codac::IntervalVector> m_out;
        codac::IntervalVector m_X;
};

// Implementation
void Scene::process() {
    for (auto &s : m_sensors) {
        for (const auto &b: m_boats) {
            float t = 1 / b.V() * ((b.Y() - s.Y()) / tan(19.5 * M_PI / 180.) + b.X() - s.X());
            codac::Interval I(t);
            I.inflate(0.5);
            s.t.push_back(I);
        }
    }
}

void Scene::solve(double precision) {
    
    ibex::Array<ibex::Sep> a(0);
    std::vector<std::shared_ptr<ibex::Sep>> vec(m_boats.size() * m_sensors.size());

    // for (auto const &s : m_sensors) {
    //     for (auto const &t : s.t) {
            std::string function = fmt::format("(y-{1})/((x-{0})-v*t)", m_sensors[0].X(), m_sensors[0].Y());
            ibex::Function f("x", "y", "v", "t", function.c_str());
            ibex::SepFwdBwd S_f(f, -Interval(0.3, 0.4));
            codac::IntervalVector y(2);
            y[0] = m_X[2];
            y[1] = m_sensors[0].t[0];
            std::cout << y << std::endl;
            std::shared_ptr<codac::SepProj> U = std::make_shared<codac::SepProj>(S_f, m_X[2], 10*precision);
            vec.push_back(U);
            a.add(*U);
    //     }
    // }

    codac::SepQInterProjF Sq(a);
    vibes::beginDrawing();
    codac::IntervalVector X = m_X.subvector(0, 1);
    codac::SIVIA(X, &Sq, precision);

    for (auto const &s : m_sensors) {
        vibes::drawPoint(s.X(), s.Y(), 5, "black[red]");
    }

    for (auto const &b : m_boats) {
        vibes::drawPoint(b.X(), b.Y(), 5, "black[blue]");
        // double rot = (b.V() > 0) ? 0 : 180;
        // vibes::drawAUV(b.X(), b.Y(), rot, 2, "black[yellow]");
    }
}