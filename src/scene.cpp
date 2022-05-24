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

void Scene::boat_space(ipegenerator::Figure &fig, double t, double precision) {
    // Solving the scene
    solve(t, precision);

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
    fig.add_layer("sensors");
    fig.set_current_layer("sensors");
    fig.set_color_stroke("black");
    fig.set_color_fill("red");
    for (const auto &s: m_sensors) {
        fig.draw_circle(s.X(), s.Y(), 0.25);
    }

    // Boats
    fig.add_layer("boats");
    fig.set_current_layer("boats");
    for (const auto &b: m_boats) {
        double rot = (b.V() > 0) ? 0 : M_PI;
        fig.draw_auv(b.X()+b.V()*t, b.Y(), rot, 0.01);
    }
}

void Scene::boat_space(codac::VIBesFig &fig, double t, double precision) {
    solve(t, precision);

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

void Scene::solve(double t, double precision) {
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

    // Sivia
    codac::IntervalVector X = m_X.subvector(0, 1);
    m_M = codac::SIVIA(X, Sp, precision, false, "", true);
}