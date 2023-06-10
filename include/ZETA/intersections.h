#pragma once

#include "bodies.h"

namespace Collisions {
    // * ===================================
    // * Point vs Primitives
    // * ===================================

    // Determine if a point lays on a line.
    inline bool PointAndLine(ZMath::Vec2D const &point, Primitives::Line2D const &line) {
        // ? Use the point slope form equation of a line to find if the point lies on the line.

        ZMath::Vec2D min = line.getMin(), max = line.getMax();
        return point.x <= max.x && point.y <= max.y && point.x >= min.x && point.y >= min.y && 
                point.y == ((line.end.y - line.start.y)/(line.end.x - line.start.x) * (point.x - line.start.x) + line.start.y);
    };

    // Determine if a point lays within a circle.
    inline bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle) { return circle.c.distSq(point) <= circle.r*circle.r; };

    // Determine if a point lays within an AABB.
    inline bool PointAndAABB(ZMath::Vec2D const &point, Primitives::AABB const &aabb) {
        ZMath::Vec2D min = aabb.getMin(), max = aabb.getMax();
        return min.x <= point.x && point.x <= max.x && min.y <= point.y && point.y <= max.y;
    };

    // Determine if a point lays within a Box2D.
    inline bool PointAndBox2D(ZMath::Vec2D const &point, Primitives::Box2D const &box) {
        // ? Rotate our point into the box2D's UV coords and perform the same check as against the AABB.

        ZMath::Vec2D min = box.getLocalMin(), max = box.getLocalMax();
        ZMath::Vec2D p = point - box.pos;

        // rotate into our UV coords
        p = box.rot.transpose() * p + box.pos;

        return min.x <= p.x && p.x <= max.x && min.y <= p.y && p.y <= max.y;
    };

    // * ===================================
    // * Line2D vs Primitives
    // * ===================================

    // Determine if a line intersects a point.
    inline bool LineAndPoint(Primitives::Line2D const &line, ZMath::Vec2D const &point) { return PointAndLine(point, line); };

    // Determine if a line intersects another line.
    inline bool LineAndLine(Primitives::Line2D const &line1, Primitives::Line2D const &line2) {
        // ? First check if the lines are parallel.
        // ? If the lines are parallel, if the line segments overlap we know we have a collision.
        // ? We can check for this by seeing if start1 lays on line2 if the lines were infinite and that there is overlap in the first place.
        // ? If the line are not parallel, we can use the standard equation of a lines to solve a system of equations and check for an intersection point.
        // ? If this intersection point exists within the bounds of the two lines, there's an intersection.
        // ? We can add special checks for vertical and horizontal lines before checking for parallel lines.

        // check for vertical lines
        if (line1.start.x == line1.end.x) { // line 1 is vertical
            float min1y = MIN(line1.start.y, line1.end.y), max1y = MAX(line1.start.y, line1.end.y);
            ZMath::Vec2D min2 = line2.getMin(), max2 = line2.getMax();

            // check if line 2 is also vertical (reason this check exists is due to floating point rounding issues)
            if (line2.start.x == line2.end.x) { return min2.y <= max1y && min1y <= max2.y && ZMath::compare(line1.start.x, line2.start.x); }
            
            return min2.x <= line1.start.x && line1.start.x <= max2.x && min1y <= max2.y && min2.y <= max1y;
        }

        // check for horizontal lines
        // if ()
        
        if (line2.start.x == line2.end.x) { // line 2 is vertical
            float min2y = MIN(line2.start.y, line2.end.y), max2y = MAX(line2.start.y, line2.end.y);
            ZMath::Vec2D min1 = line2.getMin(), max1 = line2.getMax();
            return min1.x <= line2.start.x && line2.start.x <= max1.x && min2y <= max1.y && min1.y <= max2y;
        }

        // slopes of the lines
        float m1 = (line1.end.y - line1.start.y)/(line1.end.x - line1.start.x);
        float m2 = (line2.end.y - line2.start.y)/(line2.end.x - line2.start.x);

        // check for both lines being horizontal (check for this due to floating point rounding errors)
        if (ZMath::compare(m1, 0) && ZMath::compare(m2, 0)) {
            float min1x = MIN(line1.start.x, line1.end.x), max1x = MAX(line1.start.x, line1.end.x);
            float min2x = MIN(line2.start.x, line2.end.x), max2x = MAX(line2.start.x, line2.end.x);
            return min1x <= max2x && min2x <= max1x && ZMath::compare(line1.start.y, line2.start.y);
        }

        ZMath::Vec2D min1 = line1.getMin(), max1 = line1.getMax();
        ZMath::Vec2D min2 = line2.getMin(), max2 = line2.getMax();

        // check for parallel lines
        if (ZMath::compare(m1, m2)) {
            return min1.x <= max2.x && min2.x <= max1.x && min1.y <= max2.y && min2.y <= max1.y &&
                    line1.start.y == (m2*(line1.start.x - line2.start.x) + line2.start.y);
        }

        // standard case
        float x = (m1*line1.start.x - m2*line2.start.x + line1.start.y - line2.start.y)/(m2 - m1);
        float y = m1*(x - line1.start.x) + line1.start.y;
        
        ZMath::Vec2D min(MAX(min1.x, min2.x), MAX(min1.y, min2.y));
        ZMath::Vec2D max(MIN(max1.x, max2.x), MIN(max1.y, max2.y));

        // ensure the point of intersection falls within the bounds of the lines
        return min.x <= x && x <= max.x && min.y <= y && y <= max.y;
    };

    // Determine if a line intersects a circle.
    inline bool LineAndCircle(Primitives::Line2D const &line, Primitives::Circle const &circle) {
        // ? Find the closest point to the circle (using projection) and check if it's within radius distance from the circle's center.

        // check if either of the endpoint is inside the circle
        if (PointAndCircle(line.start, circle) || PointAndCircle(line.end, circle)) { return 1; }

        ZMath::Vec2D dC = circle.c - line.start;
        ZMath::Vec2D dL = line.end - line.start;

        // scalar projection
        float t = (dC * dL)/dL.magSq();
        if (t < 0.0f || 1.0f < t) { return 0; }

        return PointAndCircle(line.start + (dL * t), circle);
    };

    // Determine if a line intersects an AABB.
    // todo test
    inline bool LineAndAABB(Primitives::Line2D const &line, Primitives::AABB const &aabb) {
        // ? Check if the line has any point within the AABB's bounds

        ZMath::Vec2D minL = line.getMin(), maxL = line.getMax();
        ZMath::Vec2D minA = aabb.getMin(), maxA = aabb.getMax();

        return minL.x <= maxA.x && minA.x <= maxL.x && minL.y <= maxA.y && minA.y <= maxL.y;
    };

    // Determine if a line intersects a Box2D.
    // todo test
    inline bool LineAndBox2D(Primitives::Line2D const &line, Primitives::Box2D const &box) {
        // ? Rotate into the box's UV coords and perform the same check as with the AABB.

        Primitives::Line2D l(line.start - box.pos, line.end - box.pos);

        // Rotate into the box's UV coords.
        l.start = box.rot * l.start + box.pos;
        l.end = box.rot * l.end + box.pos;

        // todo try just getting the line's original min and max and rotating that into local coords
        // ! slightly more efficient

        ZMath::Vec2D minL = l.getMin(), maxL = l.getMax();
        ZMath::Vec2D minC = box.getLocalMin(), maxC = box.getLocalMax();

        return minL.x <= maxC.x && minC.x <= maxL.x && minL.y <= maxC.y && minC.y <= maxL.y;
    };

    // * =================
    // * Raycasting
    // * =================



    // * ===================================
    // * Circle vs Primitives
    // * ===================================

    // todo add intersections with returning collision normals, too

    // Determine if a circle intersects a point.
    inline bool CircleAndPoint(Primitives::Circle const &circle, ZMath::Vec2D const &point) { return PointAndCircle(point, circle); };

    // Determine if a circle intersects a line.
    inline bool CircleAndLine(Primitives::Circle const &circle, Primitives::Line2D const &line) { return LineAndCircle(line, circle); };

    // Determine if a circle intersects another circle.
    inline bool CircleAndCircle(Primitives::Circle const &circle1, Primitives::Circle const &circle2) {
        float r = circle1.r + circle2.r;
        return circle1.c.distSq(circle2.c) <= r*r;
    };

    // Determine if a circle intersects an AABB.
    inline bool CircleAndAABB(Primitives::Circle const &circle, Primitives::AABB const &aabb) {
        // ? Determine the distance from the closest point on the AABB to the center of the circle.
        // ? If that distance is less than the radius of the circle, there is an intersection.

        ZMath::Vec2D closest = circle.c;
        ZMath::Vec2D min = aabb.getMin(), max = aabb.getMax();

        closest = ZMath::clamp(closest, min, max);
        return closest.distSq(circle.c) <= circle.r*circle.r;
    };

    // Determine if a circle intersects a Box2D.
    inline bool CircleAndBox2D(Primitives::Circle const &circle, Primitives::Box2D const &box) {
        // ? Do the same thing as in the AABB check but first rotate the circle into the box's UV coords.

        ZMath::Vec2D closest = circle.c - box.pos;
        ZMath::Vec2D min = box.getLocalMin(), max = box.getLocalMax();

        closest = box.rot.transpose() * closest + box.pos;
        closest = ZMath::clamp(closest, min, max);
        return closest.distSq(circle.c) <= circle.r*circle.r;
    };

    
}
