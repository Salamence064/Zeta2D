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

    // Determine if a point lays within a circle.
    bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle) { return circle.c.distSq(point) <= circle.r*circle.r; };

    // Determine if a point lays within an AABB.
    bool PointAndAABB(ZMath::Vec2D const &point, Primitives::AABB const &aabb) {
        ZMath::Vec2D min = aabb.getMin(), max = aabb.getMax();
        return min.x <= point.x && point.x <= max.x && min.y <= point.y && point.y <= max.y;
    };

    // Determine if a point lays within a Box2D.
    bool PointAndBox2D(ZMath::Vec2D const &point, Primitives::Box2D const &box) {
        // ? Rotate our point into the box2D's UV coords and perform the same check as against the AABB.

        ZMath::Vec2D min = box.getLocalMin(), max = box.getLocalMax();
        ZMath::Vec2D p = point - box.pos;

        // rotate into our UV coords
        p = box.rot.transpose() * p + box.pos;

        return min.x <= p.x && p.x <= max.x && min.y <= p.y && p.y <= max.y;
    };
}
