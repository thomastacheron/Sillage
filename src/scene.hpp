#pragma once

#include <vector>
#include <cmath>

#include "sensor.hpp"
#include "boat.hpp"

class Scene {
    public:
        Scene(std::vector<Sensor> sensors, std::vector<Boat> boats) : m_sensors(sensors), m_boats(boats) {};

        inline void process();

        std::vector<Boat> m_boats;
        std::vector<Sensor> m_sensors;
};

// Implementation
void Scene::process() {
    for (auto &s : m_sensors) {
        for (const auto &b: m_boats) {
            float t = 1 / b.V() * ((b.Y() - s.Y()) / tan(19.5 * M_PI / 180.) + s.X() - b.X()); // (b.Y() - s.Y()) / atan(asin(1/3)) + 
            codac::Interval I(t);
            I.inflate(0.1);
            s.t.push_back(I);
        }
    }
}