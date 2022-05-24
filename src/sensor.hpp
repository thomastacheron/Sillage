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

        void save(std::string filename);
        std::shared_ptr<ibex::Function> f;

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
    std::string function = fmt::format("abs(y-{1})/{2}-(x-{0})", m_x, m_y, tan(19.5 * M_PI / 180.));
    f = std::make_shared<ibex::Function>("x", "y", function.c_str());
    ArraySepBox = std::make_shared<ibex::Array<ibex::Sep>>(0);
    sep = nullptr;
}

inline void Sensor::save(std::string filename) {
    // ibex::IntervalVector frame_data(2, ibex::Interval::EMPTY_SET);
    // for (const auto &i: t) {
    //     frame_data[0] |= i;
    // }
    // frame_data[0].inflate(5);
    // frame_data[1] = ibex::Interval(-0.1, 0.1);
    // std::cout << frame_data << std::endl;
    // ipegenerator::Figure fig(frame_data, 120, 10);
    // fig.set_number_digits_axis_x(0);
    // fig.set_number_digits_axis_y(1);

    // fig.draw_arrow(int(frame_data[0].lb()), 0, int(frame_data[0].ub()), 0);
    // fig.draw_text("t", int(frame_data[0].lb()), int(frame_data[0].ub()), true);

    // // fig.draw_axis("t", "x_2");

    // for (const auto &i: t) {
    //     ibex::IntervalVector box(2);
    //     box[0] = i;
    //     box[1] = ibex::Interval(-0.1, 0.1);
    //     fig.draw_box(box,"red","black");
    // }

    // fig.save_ipe(filename + ".ipe");
    // fig.save_pdf(filename + ".pdf");
}