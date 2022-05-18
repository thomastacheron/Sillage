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

        inline void process();
        inline void solve(double precision = 1);
        inline void detection_space(double precision = 1);

        std::vector<Boat> m_boats;
        std::vector<Sensor> m_sensors;

    private:
        codac::IntervalVector m_X;
};

// Implementation
void Scene::process() {
    for (auto &s : m_sensors) {
        for (const auto &b: m_boats) {
            float t = 1 / b.V() * ((b.Y() - s.Y()) / tan(19.5 * M_PI / 180.) - (b.X() - s.X()));
            codac::Interval I(t);
            I.inflate(1);
            s.t.push_back(I);
        }
    }
}

void Scene::detection_space(double precision) {
    if (m_sensors.size() < 2) {
        std::cout << "Not enough sensors\n";
        return;
    }

    IntervalVector window (IntervalVector::empty(2));

    std::vector<std::shared_ptr<ibex::Sep>> vec;
    std::vector<std::shared_ptr<ibex::Array<ibex::Sep>>> vec_u;
    ibex::Array<ibex::Sep> cp(0);

    for(auto s = std::begin(m_sensors); s < std::begin(m_sensors) + 2; ++s) {
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
    window.inflate(2);
    codac::SIVIA(window, Scp, precision);
    vibes::setFigureProperties("Detection Space", vibesParams("x", 100, "y", 300, "width", 500, "height", 500));
    vibes::axisLimits(window[0].lb(), window[0].ub(), window[1].lb(), window[1].ub());
}

void Scene::solve(double precision) {

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
        std::string fi = fmt::format("1/v*((abs(y)-{1})/{2}-(x-{0}));", s.X(), s.Y(), tan(19.5 * M_PI / 180.));
        function += fi;
    }
    function.at(function.size() - 1) = ')';


    ibex::Function f("x", "y", "v", function.c_str());
    ibex::SepInverse Si(Scp, f);

    codac::SepProj Sp(Si, IntervalVector(Interval(0.5, 1.5)), 0.01);

    vibes::beginDrawing();
    vibes::newFigure("Wake");
    codac::IntervalVector X = m_X.subvector(0, 1);
    codac::SIVIA(X, Sp, 0.1*precision);

    for (auto const &s : m_sensors) {
        vibes::drawPoint(s.X(), s.Y(), 5, "black[red]");
    }

    for (auto const &b : m_boats) {
        double rot = (b.V() > 0) ? 0 : 180;
        vibes::drawAUV(b.X(), b.Y(), rot, 1, "black[yellow]");
    }

    vibes::setFigureProperties("Wake", vibesParams("x", 600, "y", 260, "width", int(500*m_X[0].diam()/m_X[1].diam()), "height", 500));
    vibes::axisLimits(m_X[0].lb(), m_X[0].ub(), m_X[1].lb(), m_X[1].ub());


    // codac::IntervalVector y(2);
    // y[0] = codac::Interval(0.9, 1);
    // y[1] = m_sensors[0].t[0];
    // std::cout << y << std::endl;
    // codac::SepProj U(S_f, y, precision);
    // a.add(U);

    // ibex::Array<ibex::Sep> u(0);
    // std::vector<std::shared_ptr<ibex::Function>> func;
    // std::vector<std::shared_ptr<ibex::Sep>> vec;
    // for (auto const &s : m_sensors) {
    //     for (auto const &t : s.t) {
    //         auto S_f = std::make_shared<ibex::SepFwdBwd>(*s.f, t);
    //         std::cout << t << std::endl;
    //         vec.push_back(S_f);
    //         u.add(*S_f);
    //     }
    // }

    // ibex::SepQInter Sq(u);

    // vibes::beginDrawing();
    // vibes::newFigure("Wake");
    // codac::IntervalVector X = m_X.subvector(0, 1);
    // codac::SIVIA(X, &Sq, 0.1*precision);

    // for (auto const &s : m_sensors) {
    //     vibes::drawPoint(s.X(), s.Y(), 5, "black[red]");
    // }

    // for (auto const &b : m_boats) {
    //     double rot = (b.V() > 0) ? 0 : 180;
    //     vibes::drawAUV(b.X(), b.Y(), rot, 1, "black[yellow]");
    // }

    // vibes::setFigureProperties("Wake", vibesParams("x", 500, "y", 300, "width", 600, "height", 600));
    // vibes::axisLimits(m_X[0].lb(), m_X[0].ub(), m_X[1].lb(), m_X[1].ub());

    // std::vector<std::shared_ptr<codac::Ctc>> vec;
    // std::vector<std::shared_ptr<ibex::Array<ibex::Ctc>>> vec_u;
    // ibex::Array<ibex::Ctc> cp(0);
    // for (const auto &s : m_sensors){
    //     auto u = std::make_shared<ibex::Array<ibex::Ctc>>(0);
    //     for (const auto &i : s.t) {
    //         std::cout << i << std::endl;
    //         auto Ci = std::make_shared<codac::CtcBox>(codac::IntervalVector(i));
    //         vec.push_back(Ci);
    //         u->add(*Ci);
    //     }
    //     vec_u.push_back(u);
    //     auto Cu = std::make_shared<codac::CtcUnion>(*u); 
    //     vec.push_back(Cu);
    //     cp.add(*Cu);
    // }
    // codac::CtcCartProd Ccp(cp);

    // vibes::beginDrawing();
    // vibes::newFigure("Wake");
    // codac::IntervalVector X = m_X.subvector(0, 1);
    // codac::SIVIA(X, &Ccp, precision);

    // for (auto const &s : m_sensors) {
    //     vibes::drawPoint(s.X(), s.Y(), 5, "black[red]");
    // }

    // for (auto const &b : m_boats) {
    //     double rot = (b.V() > 0) ? 0 : 180;
    //     vibes::drawAUV(b.X(), b.Y(), rot, 1, "black[yellow]");
    // }

    // vibes::setFigureProperties("Wake", vibesParams("x", 500, "y", 300, "width", 600, "height", 600));
    // vibes::axisLimits(m_X[0].lb(), m_X[0].ub(), m_X[1].lb(), m_X[1].ub());

}