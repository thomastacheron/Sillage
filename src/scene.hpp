#pragma once

#include <vector>
#include <memory>
#include <cmath>

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
            float t = 1 / b.V() * ((b.Y() - s.Y()) / tan(19.5 * M_PI / 180.) + s.X() - b.X());
            codac::Interval I(t);
            I.inflate(0.1);
            s.t.push_back(I);
        }
    }
}

void Scene::solve(double precision) {
    ibex::Function f_plus("dx", "dy", "v", "1/v*(dy/0.761-dx)");
    ibex::Function f_moins("dx", "dy", "v", "1/v*(-dy/0.761-dx)");
    codac::Interval y = m_sensors[0].t[0];
    std::cout << y << std::endl;

    ibex::Array<ibex::Sep> a(0);
    std::vector<std::shared_ptr<ibex::Sep>> vec(m_boats.size() * m_sensors.size());
    
    codac::Interval t = m_sensors[0].t[0];

    // for (auto const &s : m_sensors) {
    //     for (auto const &t : s.t) {
            codac::Interval v_plus(0.5, 1.5);
            codac::Interval v_moins(-1.5, -0.5);
            ibex::SepFwdBwd S_f_plus(f_plus, t);
            ibex::SepFwdBwd S_f_moins(f_moins, t);
            codac::SepProj S_plus(S_f_plus, v_plus, 10*precision);
            codac::SepProj S_moins(S_f_moins, v_moins, 10*precision);
            std::shared_ptr<ibex::Sep> U = std::make_shared<ibex::SepUnion>(S_plus, S_moins);
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
        double rot = (b.V() > 0) ? 0 : 180;
        vibes::drawAUV(b.X(), b.Y(), rot, 2, "black[yellow]");
    }
}