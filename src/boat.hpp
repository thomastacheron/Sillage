#pragma once

class Boat {
    public:
        Boat(float x, float y, float v) : m_x(x), m_y(y), m_v(v) {};

        inline float X() const { return m_x; };
        inline float Y() const { return m_y; };
        inline float V() const { return m_v; };

    private:
        float m_x = 0;
        float m_y = 0;
        float m_v = 0;
};