#pragma once

#include <codac.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

enum CODE {
    INSIDE = 0, // 0000
    LEFT = 1, // 0001
    RIGHT = 2, // 0010
    BOTTOM = 4, // 0100
    TOP = 8, // 1000
};

// Function to compute region code for a point(x, y)
int computeCode(codac::IntervalVector X, double x, double y);

// Cohen-Sutherland algorithm
// Clipping a line from P1 = (x2, y2) to P2 = (x2, y2)
bool cohenSutherlandClip(codac::IntervalVector X, double &x1, double &y1, double &x2, double &y2);