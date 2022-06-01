#pragma once

#include <vector>
#include <memory>
#include <string>

#include <codac.h>
#include <codac/vibes.h>
#include <ibex/ibex_SepUnion.h>

#include <fmt/core.h>


class Sensor {
    public:
        inline Sensor(float x, float y);
        std::vector<codac::Interval> t;

        inline float X() const { return m_x; };
        inline float Y() const { return m_y; };

        inline std::shared_ptr<ibex::Sep> Sep();

        inline bool is_awake(double time) const;

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
    function = fmt::format("1/v*(abs(y-{1})/{2}-(x-{0}))", m_x, m_y, tan(19.5 * M_PI / 180.));
    ArraySepBox = std::make_shared<ibex::Array<ibex::Sep>>();
}

inline std::shared_ptr<ibex::Sep> Sensor::Sep() {
    for (const auto & i: t) {
        auto SepBoxI =  std::make_shared<codac::SepBox>(codac::IntervalVector(i));
        SepBoxes.push_back(SepBoxI);
        ArraySepBox->add(*SepBoxI);
    }
    sep = std::make_shared<ibex::SepUnion>(*ArraySepBox);
    return sep;
}

inline bool Sensor::is_awake(double time) const {
    for (const auto & i: t) {
        if (i.contains(time)) {
            return true;
        }
    }
    return false;
}
