#include <codac.h>
#include "utils.hpp"

// Function to compute region code for a point(x, y)
int computeCode(codac::IntervalVector X, double x, double y) {
    // initialized as being inside
    int code = INSIDE;
 
    if (x < X[0].lb()) // to the left of rectangle
        code |= LEFT;
    else if (x > X[0].ub()) // to the right of rectangle
        code |= RIGHT;
    if (y < X[1].lb()) // below the rectangle
        code |= BOTTOM;
    else if (y > X[1].ub()) // above the rectangle
        code |= TOP;
 
    return code;
}
 
// Implementing Cohen-Sutherland algorithm
// Clipping a line from P1 = (x2, y2) to P2 = (x2, y2)
bool cohenSutherlandClip(codac::IntervalVector X, double &x1, double &y1, double &x2, double &y2) {
    // Compute region codes for P1, P2
    int code1 = computeCode(X, x1, y1);
    int code2 = computeCode(X, x2, y2);
 
    // Initialize line as outside the rectangular window
    bool accept = false;
 
    while (true) {
        if ((code1 == 0) && (code2 == 0)) {
            // If both endpoints lie within rectangle
            accept = true;
            break;
        }
        else if (code1 & code2) {
            // If both endpoints are outside rectangle,
            // in same region
            break;
        }
        else {
            // Some segment of line lies within the
            // rectangle
            int code_out;
            double x, y;
 
            // At least one endpoint is outside the
            // rectangle, pick it.
            if (code1 != 0)
                code_out = code1;
            else
                code_out = code2;
 
            // Find intersection point;
            // using formulas y = y1 + slope * (x - x1),
            // x = x1 + (1 / slope) * (y - y1)
            if (code_out & TOP) {
                // point is above the clip rectangle
                x = x1 + (x2 - x1) * (X[1].ub() - y1) / (y2 - y1);
                y = X[1].ub();
            }
            else if (code_out & BOTTOM) {
                // point is below the rectangle
                x = x1 + (x2 - x1) * (X[1].lb() - y1) / (y2 - y1);
                y = X[1].lb();
            }
            else if (code_out & RIGHT) {
                // point is to the right of rectangle
                y = y1 + (y2 - y1) * (X[0].ub() - x1) / (x2 - x1);
                x = X[0].ub();
            }
            else if (code_out & LEFT) {
                // point is to the left of rectangle
                y = y1 + (y2 - y1) * (X[0].lb() - x1) / (x2 - x1);
                x = X[0].lb();
            }
 
            // Now intersection point x, y is found
            // We replace point outside rectangle
            // by intersection point
            if (code_out == code1) {
                x1 = x;
                y1 = y;
                code1 = computeCode(X, x1, y1);
            }
            else {
                x2 = x;
                y2 = y;
                code2 = computeCode(X, x2, y2);
            }
        }
    }

    return accept;
}