#pragma once

#include <vector>
#include <codac.h>

class Sensor {
    public:
        Sensor(float x, float y) : m_x(x), m_y(y) {};
        std::vector<codac::Interval> t;

        inline float X() const { return m_x; };
        inline float Y() const { return m_y; };

    private:
        float m_x = 0;
        float m_y = 0;
};