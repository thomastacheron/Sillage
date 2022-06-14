#pragma once

#include <vector>
#include <memory>
#include <string>
#include <mutex>

#include "sensor.hpp"
#include "boat.hpp"

#include <codac/codac_IntervalVector.h>
#include <codac/codac_VIBesFig.h>
#include <codac/codac_Set.h>
#include <ipegenerator/ipegenerator.h>

void drawBoat(codac::VIBesFig &fig, double cx = 0, double cy = 0, double theta = 0, double scale = 0, std::string params = "");


class Scene {
    public:
        Scene(codac::IntervalVector X0, std::vector<Sensor> sensors, std::vector<Boat> boats) : m_X(X0), m_sensors(sensors), m_boats(boats) {};
        
        void detection_space(std::size_t i1, std::size_t i2, double precision = 1, bool show_truth = false, bool use_ipe = false);

        // Boat space on ipe figure
        void boat_space(std::map<codac::SetValue,std::list<codac::IntervalVector>> &M, ipegenerator::Figure &fig, double t=0);

        void set_sensors(const std::vector<Sensor> sensors);
        void set_boats(const std::vector<Boat> boats);

        std::vector<Sensor> get_sensors();
        std::vector<Boat> get_boats();

        double t_min();

        // Solving function
        std::pair<double,std::map<codac::SetValue,std::list<codac::IntervalVector>>> solve(double t, double precision, bool causal = false);

    private:
        // State space
        codac::IntervalVector m_X;

        // Sensors vector
        std::vector<Sensor> m_sensors;

        // Boats vector
        std::vector<Boat> m_boats;

        // Dirty flag
        bool m_dirty = true;

        // Process function to recompute detection time when the dirty flag is up
        void process();

        std::mutex m_mutex;
};