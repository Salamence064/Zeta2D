#pragma once

#include "bodies.h"

namespace Collisions {
    // * ===================================
    // * Point vs Primitives
    // * ===================================

    // Determine if a point lays on a line.
    bool PointAndLine(ZMath::Vec2D const &point, Primitives::Line2D const &line) {
        // ? Use the point slope form equation of a line to find if the point lies on the line.

        ZMath::Vec2D min = line.getMin(), max = line.getMax();
        return point.x <= max.x && point.y <= max.y && point.x >= min.x && point.y >= min.y && 
                point.y == ((line.end.y - line.start.y)/(line.end.x - line.start.x) * (point.x - line.start.x) + line.start.y);
    };
}
