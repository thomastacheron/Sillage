#pragma once

#include <vector>
#include <memory>
#include <string>

#include <codac.h>
#include <codac/vibes.h>
#include <ibex/ibex_SepUnion.h>

#include <fmt/core.h>

#include <ipegenerator/ipegenerator.h>

struct SepSensor {
    std::vector<std::shared_ptr<ibex::Sep>> SepBoxes;
    std::shared_ptr<ibex::Array<ibex::Sep>> SepArray;
    std::shared_ptr<ibex::SepUnion> Sep;
};


class Sensor {
    public:
        inline Sensor(float x, float y);
        std::vector<codac::Interval> t;

        float X() const { return m_x; };
        float Y() const { return m_y; };

        SepSensor Sep(double time, bool causal = false, double t_min = 0);

        bool is_awake(double time) const;

        void draw(ipegenerator::Figure &fig, double t, double size) const;

        std::string function;

        // Separator
        std::vector<std::shared_ptr<ibex::Sep>> SepBoxes;
        std::shared_ptr<ibex::Array<ibex::Sep>> ArraySepBox;
        std::shared_ptr<ibex::Sep> sep;

    private:
        float m_x = 0;
        float m_y = 0;
};

inline Sensor::Sensor(float x, float y) {
    m_x = x;
    m_y = y;
    function = fmt::format("1/v*(sign(v)*abs(y-{1})/{2}-(x-{0}))", m_x, m_y, tan(19.5 * M_PI / 180.));
    ArraySepBox = std::make_shared<ibex::Array<ibex::Sep>>();
}

inline SepSensor Sensor::Sep(double time, bool causal, double t_min) {
    std::vector<std::shared_ptr<ibex::Sep>> SepBoxes;
    std::shared_ptr<ibex::Array<ibex::Sep>> SepArray = std::make_shared<ibex::Array<ibex::Sep>>(0);

    if (causal) {
        auto sb_min = std::make_shared<codac::SepBox>(codac::IntervalVector(codac::Interval(NEG_INFINITY, t_min)));
        SepBoxes.push_back(sb_min);
        SepArray->add(*sb_min);

        auto sb_max = std::make_shared<codac::SepBox>(codac::IntervalVector(codac::Interval(time, POS_INFINITY)));
        SepBoxes.push_back(sb_max);
        SepArray->add(*sb_max);

        for (const auto & i: t) {
            codac::Interval i_trunc = i & codac::Interval(NEG_INFINITY, time);
            if (!i_trunc.is_empty()) {
                auto sb = std::make_shared<codac::SepBox>(codac::IntervalVector(i_trunc));
                SepBoxes.push_back(sb);
                SepArray->add(*sb);
            }
        }
    }
    else {
        for (const auto & i: t) {
            auto sb = std::make_shared<codac::SepBox>(codac::IntervalVector(i));
            SepBoxes.push_back(sb);
            SepArray->add(*sb);
        }
    }
    
    auto Su = std::make_shared<ibex::SepUnion>(*SepArray);
    SepSensor Ss{SepBoxes, SepArray, Su};

    return Ss;
}

inline bool Sensor::is_awake(double time) const {
    for (const auto & i: t) {
        if (i.contains(time)) {
            return true;
        }
    }
    return false;
}

inline void Sensor::draw(ipegenerator::Figure &fig, double t, double size) const {
    fig.set_current_layer("sensors");
    fig.set_color_stroke("black");
    if (is_awake(t)) {
        fig.set_color_fill("green");
    }
    else {
        fig.set_color_fill("red");
    }
    fig.draw_circle(m_x, m_y, size);
}