#include "scene.hpp"

#include <vector>
#include <memory>
#include <cmath>
#include <string>
#include <fmt/core.h>

#include "boat.hpp"
#include "sensor.hpp"

#include <codac-unsupported/codac_SepProj.h>
#include <codac/codac_sivia.h>
#include <codac/codac_SepCartProd.h>
#include <codac/codac_SepBox.h>
#include <codac/codac_Set.h>

#include <ibex/ibex_Interval.h>
#include <ibex/ibex_SepUnion.h>
#include <ibex/ibex_SepInverse.h>

#include <ipegenerator/ipegenerator.h>

void drawBoat(codac::VIBesFig &fig, double cx, double cy, double theta, double scale, std::string params) {
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

    std::vector<ibex::Vector> v;
    for (int k=0; k<x.size(); ++k) {
        ibex::Vector a{scale * (cos(theta) * x[k] + sin(theta) * y[k]) + cx, scale * (sin(theta) * x[k] + cos(theta) * y[k]) + cy};
        v.push_back(a);
    }

    codac::Polygon P(v);
    fig.draw_polygon(P, params);
}


void Scene::set_sensors(const std::vector<Sensor> sensors) {
    m_dirty = true;
    m_sensors = sensors;
}

void Scene::set_boats(const std::vector<Boat> boats) {
    m_dirty = true;
    m_boats = boats;
}

std::vector<Sensor> Scene::get_sensors() {
    process();
    return m_sensors;
}

std::vector<Boat> Scene::get_boats() {
    process();
    return m_boats;
}

void Scene::process() {
    for (auto &s : m_sensors) {
        s.t.clear();
        for (const auto &b: m_boats) {
            // Detection Interval
            double t = 1 / b.V() * (std::abs(b.Y() - s.Y()) / std::tan(19.5 * M_PI / 180.) - (b.X() - s.X()));
            codac::Interval I(t);
            I.inflate(0.5);
            s.t.push_back(I);
        }
    }
    m_dirty = false;
}

void Scene::detection_space(std::size_t i1, std::size_t i2, double precision, bool show_truth, bool use_ipe) {
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

void Scene::draw_sensors(ipegenerator::Figure &fig, double t, double size) {
    fig.add_layer("sensors");
    fig.set_current_layer("sensors");
    fig.set_color_stroke("black");
    for (const auto &s: m_sensors) {
        if (s.is_awake(t)) {
            fig.set_color_fill("green");
        }
        else {
            fig.set_color_fill("red");
        }
        fig.draw_circle(s.X(), s.Y(), size);
    }
}

void Scene::draw_boat(ipegenerator::Figure &fig, const Boat &b, double t) {
    // Wake
    std::vector<double> x;
    std::vector<double> y;
    if (b.V() < 0) {
        // Upper wake
        double xu = (m_X[1].ub() - b.Y()) / std::tan(19.5*M_PI/180) + b.X();
        if (m_X[0].contains(xu)) {
            x.push_back(xu);
            y.push_back(m_X[1].ub());
        }
        else {
            x.push_back(m_X[0].ub());
            y.push_back(b.Y() + std::tan(19.5*M_PI/180) * (m_X[0].ub() - b.X()));
        }

        // Boat
        x.push_back(b.X());
        y.push_back(b.Y());

        // Lower wake
        double xl = - (m_X[1].lb() - b.Y()) / std::tan(19.5*M_PI/180) + b.X();
        if (m_X[0].contains(xl)) {
            x.push_back(xl);
            y.push_back(m_X[1].lb());
        }
        else {
            x.push_back(m_X[0].ub());
            y.push_back(b.Y() - std::tan(19.5*M_PI/180) * (m_X[0].ub() - b.X()));
        }
    }
    else {
        // Upper wake
        double xu = -(m_X[1].ub() - b.Y()) / std::tan(19.5*M_PI/180) + b.X();
        if (m_X[0].contains(xu)) {
            x.push_back(xu);
            y.push_back(m_X[1].ub());
        }
        else {
            x.push_back(m_X[0].lb());
            y.push_back(b.Y() - std::tan(19.5*M_PI/180) * (m_X[0].lb() - b.X()));
        }

        // Boat
        x.push_back(b.X());
        y.push_back(b.Y());

        // Lower wake
        double xl = (m_X[1].lb() - b.Y()) / std::tan(19.5*M_PI/180) + b.X();
        if (m_X[0].contains(xl)) {
            x.push_back(xl);
            y.push_back(m_X[1].lb());
        }
        else {
            x.push_back(m_X[0].lb());
            y.push_back(b.Y() + std::tan(19.5*M_PI/180) * (m_X[0].lb() - b.X()));
        }
    }
    fig.set_line_width(1);
    fig.set_stroke_opacity(20);
    fig.draw_polygon(x, y, "colorBlind1", "", ipegenerator::PATH_TYPE::STROKE_ONLY, false);

    // Boat
    double rot = (b.V() > 0) ? 0 : M_PI;
    fig.reset_attribute();
    fig.draw_auv(b.X()+b.V()*t, b.Y(), rot, 0.01);
}

void Scene::boat_space(ipegenerator::Figure &fig, double t, double precision, bool causal) {
    // Solving the scene
    solve(t, precision, causal);

    // IN boxes
    fig.set_current_layer("inner");
    for (const auto &iv : m_M[codac::SetValue::IN]) {
        fig.draw_box(iv,"colorBlindInStroke","colorBlindInFill");
    }

    // OUT boxes
    fig.set_current_layer("outer");
    for (const auto &iv : m_M[codac::SetValue::OUT]) {
        fig.draw_box(iv,"colorBlindOutStroke","colorBlindOutFill");
    }

    // UNKNOWN boxes
    fig.set_current_layer("uncertain");
    for (const auto &iv : m_M[codac::SetValue::UNKNOWN]) {
        fig.draw_box(iv,"colorBlindMaybeStroke","colorBlindMaybeFill");
    }

    // Sensors
    draw_sensors(fig, t, 0.25);

    // Boats
    fig.add_layer("boats");
    fig.set_current_layer("boats");
    for (const auto &b: m_boats) {
        if (m_X[0].contains(b.X()+b.V()*t)) {
            draw_boat(fig, b, t);            
        }
    }
}

void Scene::boat_space(codac::VIBesFig &fig, double t, double precision, bool causal) {
    solve(t, precision, causal);

    // IN boxes
    std::vector<codac::IntervalVector> v_in{ std::begin(m_M[codac::SetValue::IN]), std::end(m_M[codac::SetValue::IN]) };
    fig.draw_boxes(v_in, "black[red]");

    // OUT boxes
    std::vector<codac::IntervalVector> v_out{ std::begin(m_M[codac::SetValue::OUT]), std::end(m_M[codac::SetValue::OUT]) };
    fig.draw_boxes(v_out, "black[blue]");

    // UNKNOWN boxes
    std::vector<codac::IntervalVector> v_unk{ std::begin(m_M[codac::SetValue::UNKNOWN]), std::end(m_M[codac::SetValue::UNKNOWN]) };
    fig.draw_boxes(v_unk, "black[yellow]");

    // Showing sensors
    for (auto const &s : m_sensors) {
        fig.draw_circle(s.X(), s.Y(), 0.2, "black[red]");
    }

    // Showing boats
    for (auto const &b : m_boats) {
        double rot = (b.V() > 0) ? 0 : M_PI;
        drawBoat(fig, b.X(), b.Y(), rot, 1, "black[#34495e]");
        drawBoat(fig, b.X()+b.V()*t, b.Y(), rot, 1, "black[yellow]");
    }
}

void Scene::solve(double t, double precision, bool causal) {
    if (m_dirty) {
        process();
    }

    // Cartesian product between detected times
    ibex::Array<ibex::Sep> cp(0);
    for(Sensor &s: m_sensors) {
        cp.add(*(s.Sep()));
    }
    codac::SepCartProd Scp(cp);

    // Function used in set inversion
    std::string function = "(";
    for (const Sensor &s: m_sensors) {
        std::string fi = s.function + fmt::format("+{0};", t);
        function += fi;
    }
    function.at(function.size() - 1) = ')';

    // Inversion of the separator on sensor's detected times
    ibex::Function f("x", "y", "v", function.c_str());
    ibex::SepInverse Si(Scp, f);

    // Projection of the separator along x and y given a v
    std::vector<std::shared_ptr<ibex::Sep>> refs;
    ibex::Array<ibex::Sep> a(0);
    codac::Interval v_p = m_X[2] & codac::Interval(precision, POS_INFINITY);
    if (!v_p.is_empty()) {
        auto Sp = std::make_shared<codac::SepProj>(Si, v_p, precision);
        refs.push_back(Sp);
        a.add(*Sp);
    }
    codac::Interval v_m = m_X[2] & codac::Interval(NEG_INFINITY, -precision);
    if (!v_m.is_empty()) {
        auto Sp = std::make_shared<codac::SepProj>(Si, v_m, precision);
        refs.push_back(Sp);
        a.add(*Sp);
    }
    ibex::SepUnion Su(a);

    // Sivia
    m_M = codac::SIVIA(m_X.subvector(0, 1), Su, precision, false, "", true);
}