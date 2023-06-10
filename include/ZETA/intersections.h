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

    // Determine if a ray intersects a circle.
    // dist will be modified to equal the distance from the ray it hits the circle.
    // dist is set to -1 if there is no intersection.
    inline bool raycast(Primitives::Circle const &circle, Primitives::Ray2D const &ray, float &dist) {
        // todo at some point we may want to get the hit point.
        // todo if we do, we simply do hit = ray.origin + dist*ray.dir;

        // ? First we find the closest point on the ray to the circle.
        // ? To find this point, we find the distance to it using dir * (center - origin).
        // ? Next we solve origin + t*origin to find the closest point.
        // ? If the distance of that closest point to the center is less than or equal to the radius, we have an intersection.

        // determine the closest point and the distance to that point
        float t = ray.dir * (circle.c - ray.origin);

        // the circle is behind the ray
        if (t < 0) {
            dist = -1.0f;
            return 0;
        }

        ZMath::Vec2D close = ray.origin + ray.dir * t;

        float dSq = circle.c.distSq(close);
        float rSq = circle.r*circle.r;

        // no intersection
        if (dSq > rSq) {
            dist = -1.0f;
            return 0;
        }

        // lands on the circumference
        if (dSq == rSq) {
            dist = t;
            return 1;
        }

        // ray started in the circle
        if (t < circle.r) {
            dist = t + sqrtf(rSq - dSq);
            return 1;
        }

        // standard intersection
        dist = t - sqrtf(rSq - dSq);
        return 1;
    };

    // Determine if a ray intersects an AABB.
    // dist will be modified to equal the distance from the ray it hits the AABB.
    // dist is set to -1 if there is no intersection.
    inline bool raycast(Primitives::AABB const &aabb, Primitives::Ray2D const &ray, float &dist) {
        // ? We can determine the distance from the ray to a certain edge by dividing a select min or max vector component
        // ?  by the corresponding component from the unit directional vector.
        // ? We know if tMin > tMax, then we have no intersection and if tMax is negative the AABB is behind us and we do not have a hit.

        ZMath::Vec2D dirfrac(1.0f/ray.dir.x, 1.0f/ray.dir.y);
        ZMath::Vec2D min = aabb.getMin(), max = aabb.getMax();

        float t1 = (min.x - ray.origin.x)*dirfrac.x;
        float t2 = (max.x - ray.origin.x)*dirfrac.x;
        float t3 = (min.y - ray.origin.y)*dirfrac.y;
        float t4 = (max.y - ray.origin.y)*dirfrac.y;

        // tMin is the max of the mins and tMx is the min of the maxes
        float tMin = MAX(MIN(t1, t2), MIN(t3, t4));
        float tMax = MIN(MAX(t1, t2), MAX(t3, t4));

        // if tMax < 0 the ray is intersecting behind it. Therefore, we do not actually have a collision.
        if (tMax < 0) {
            dist = -1.0f;
            return 0;
        }

        // ray doesn't intersect the AABB.
        if (tMax < tMin) {
            dist = -1.0f;
            return 0;
        }

        // ray's origin is inside of the AABB.
        if (tMin < 0) {
            dist = tMax;
            return 1;
        }

        dist = tMin;
        return 1;
    };

    // Determine if a ray intersects a Box2D.
    // dist will be modified to equal the distance from the ray it hits the Box2D.
    // dist is set to -1 if there is no intersection.
    inline bool raycast(Primitives::Box2D const &box, Primitives::Ray2D const &ray, float &dist) {
        // ? Rotate and apply the AABB solution.

        ZMath::Vec2D Box2DMin = box.getLocalMin();
        ZMath::Vec2D Box2DMax = box.getLocalMax();

        ZMath::Vec2D rayOrigin = ray.origin - box.pos;
        ZMath::Vec2D rayDir = ray.dir;

        // todo unsure if this will work for angles outside the first quadrant; it should, though (I think)
        // ! test although im p sure it should work

        rayOrigin = box.rot * rayOrigin + box.pos;
        rayDir = box.rot * rayDir;

        Primitives::AABB newBox2D(box.getLocalMin(), box.getLocalMax());
        Primitives::Ray2D newRay(rayOrigin, rayDir);

        float d2 = 0; // ! simply for making it pass the unit tests

        return raycast(newBox2D, newRay, d2);
    };

    // * ===================================
    // * Circle vs Primitives
    // * ===================================

    // Determine if a circle intersects a point.
    inline bool CircleAndPoint(Primitives::Circle const &circle, ZMath::Vec2D const &point) { return PointAndCircle(point, circle); };

    // Determine if a circle intersects a line.
    inline bool CircleAndLine(Primitives::Circle const &circle, Primitives::Line2D const &line) { return LineAndCircle(line, circle); };

    // Determine if a circle intersects another circle.
    inline bool CircleAndCircle(Primitives::Circle const &circle1, Primitives::Circle const &circle2) {
        float r = circle1.r + circle2.r;
        return circle1.c.distSq(circle2.c) <= r*r;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool CircleAndCircle(Primitives::Circle const &circle1, Primitives::Circle const &circle2, ZMath::Vec2D &normal) {
        float r = circle1.r + circle2.r;
        ZMath::Vec2D circleDiff = circle2.c - circle1.c;

        if (circleDiff.magSq() > r*r) { return 0; }
        normal = circleDiff.normalize();

        return 1;
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

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool CircleAndAABB(Primitives::Circle const &circle, Primitives::AABB const & aabb, ZMath::Vec2D &normal) {
        ZMath::Vec2D closest = circle.c;
        ZMath::Vec2D min = aabb.getMin(), max = aabb.getMax();

        closest = ZMath::clamp(closest, min, max);
        ZMath::Vec2D diff = closest - circle.c;

        if (diff.magSq() > circle.r*circle.r) { return 0; }

        normal = diff.normalize();

        return 1;
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

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool CircleAndBox2D(Primitives::Circle const &circle, Primitives::Box2D const &box, ZMath::Vec2D &normal) {
        ZMath::Vec2D closest = circle.c - box.pos;
        ZMath::Vec2D min = box.getLocalMin(), max = box.getLocalMax();

        // rotate the center of the circle into the UV coordinates of our Box2D
        closest = box.rot * closest + box.pos;
        
        // perform the check as if it was an AABB vs circle
        closest = ZMath::clamp(closest, min, max);
        ZMath::Vec2D diff = closest - circle.c;

        if (diff.magSq() > circle.r*circle.r) { return 0; }

        // the closest point to the circle's center will be our contact point rotated back into global coordinates coordinates

        closest -= box.pos;
        closest = box.rot.transpose() * closest + box.pos;

        normal = diff.normalize();

        return 1;
    };

    // * ===================================
    // * AABB vs Primitives
    // * ===================================

    // Determine if an AABB intersects a point.
    inline bool AABBAndPoint(Primitives::AABB const &aabb, ZMath::Vec2D const &point) { return PointAndAABB(point, aabb); };

    // Determine if an AABB intersects a line.
    inline bool AABBAndLine(Primitives::AABB const &aabb, Primitives::Line2D const &line) { return LineAndAABB(line, aabb); };

    // Determine if an AABB intersects a circle.
    inline bool AABBAndCircle(Primitives::AABB const &aabb, Primitives::Circle const &circle) { return CircleAndAABB(circle, aabb); };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool AABBAndCircle(Primitives::AABB const &aabb, Primitives::Circle const &circle, ZMath::Vec2D &normal) {
        bool hit = CircleAndAABB(circle, aabb, normal);
        normal = -normal;
        return hit;
    };

    // Determine if an AABB intersects another AABB.
    inline bool AABBAndAABB(Primitives::AABB const &aabb1, Primitives::AABB const &aabb2) {
        // ? Check if there is overlap on all three axes.
        // ? If so, the AABBs intersect.

        ZMath::Vec2D min1 = aabb1.getMin(), max1 = aabb1.getMax();
        ZMath::Vec2D min2 = aabb2.getMin(), max2 = aabb2.getMax();

        return min1.x <= max2.x && min2.x <= max1.x && min1.y <= max2.y && min2.y <= max1.y;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool AABBAndAABB(Primitives::AABB const &aabb1, Primitives::AABB const &aabb2, ZMath::Vec2D &normal) {
        // half size of AABB a and b respectively
        ZMath::Vec2D hA = aabb1.getHalfsize(), hB = aabb2.getHalfsize();

        // * Check for intersections using the separating axis theorem.
        // because both are axis aligned, global space is the same as the local space of both AABBs.

        // distance between the two
        ZMath::Vec2D dP = aabb2.pos - aabb1.pos;
        ZMath::Vec2D absDP = ZMath::abs(dP);

        // penetration along A's (and B's) axes
        ZMath::Vec2D faceA = absDP - hA - hB;
        if (faceA.x > 0 || faceA.y > 0) { return 0; }

        // ? Since they are axis aligned, the penetration between the two will be the same on any given axis.
        // ?  Therefore, we only need to check for A.

        // * Find the best axis (i.e. the axis with the least amount of penetration).

        // Assume A's x-axis is the best axis first
        float separation = faceA.x;
        normal = dP.x > 0.0f ? ZMath::Vec2D(1, 0) : ZMath::Vec2D(-1, 0);

        // tolerance values
        float relativeTol = 0.95f;
        float absoluteTol = 0.01f;

        // ? check if there is another axis better than A's x axis by checking if the penetration along
        // ?  the current axis being checked is greater than that of the current penetration
        // ?  (as greater value = less negative = less penetration).

        // A's remaining axes
        if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
            separation = faceA.y;
            normal = dP.y > 0.0f ? ZMath::Vec2D(0, 1) : ZMath::Vec2D(0, -1);
        }

        return 1;
    };

    // Determine if an AABB intersects a Box2D.
    inline bool AABBAndBox2D(Primitives::AABB const &aabb, Primitives::Box2D const &box) {
        // ? Use the separating axis theorem to determine if there is an intersection bewteen the AABB and Box2D.

        // half the size of the AABB and Box2D (A = AABB, B = Box2D)
        ZMath::Vec2D hA = aabb.getHalfsize(), hB = box.getHalfsize();

        // rotate anything from global space to the Box2D's local space
        ZMath::Mat2D rotBT = box.rot.transpose();

        // determine the distance between the positions
        ZMath::Vec2D dA = box.pos - aabb.pos;
        ZMath::Vec2D dB = rotBT * dA;

        // * Check for intersection using the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec2D faceA = ZMath::abs(dA) - hA - hB;
        if (faceA.x > 0 || faceA.y > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec2D faceB = ZMath::abs(dB) - hB - rotBT * hA;
        return faceB.x <= 0 && faceB.y <= 0;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool AABBAndBox2D(Primitives::AABB const &aabb, Primitives::Box2D const &box, ZMath::Vec2D &normal) {
        // ? Use the separating axis theorem to determine if there is an intersection between the AABB and Box2D.

        // half size of the aabb and box respectively (A = AABB, B = box)
        ZMath::Vec2D hA = aabb.getHalfsize(), hB = box.getHalfsize();

        // rotate anything from global space to the box's local space
        ZMath::Mat2D rotBT = box.rot.transpose();

        // determine the distance between the positions
        ZMath::Vec2D dA = box.pos - aabb.pos; // global space is the AABB's local space
        ZMath::Vec2D dB = rotBT * dA;

        // * Check for intersection using the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec2D faceA = ZMath::abs(dA) - hA - hB;
        if (faceA.x > 0 || faceA.y > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec2D faceB = ZMath::abs(dB) - hB - rotBT * hA;
        if (faceB.x > 0 || faceB.y > 0) { return 0; }
        
        // * Find the best axis (i.e. the axis with the least amount of penetration).

        // Assume A's x-axis is the best axis first
        float separation = faceA.x;
        normal = dA.x > 0.0f ? ZMath::Vec2D(1, 0) : ZMath::Vec2D(-1, 0);

        // tolerance values
        float relativeTol = 0.95f;
        float absoluteTol = 0.01f;

        // ? check if there is another axis better than A's x axis by checking if the penetration along
        // ?  the current axis being checked is greater than that of the current penetration
        // ?  (as greater value = less negative = less penetration).

        // A's remaining axes
        if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
            separation = faceA.y;
            normal = dA.y > 0.0f ? ZMath::Vec2D(0, 1) : ZMath::Vec2D(0, -1);
        }

        // B's axes
        if (faceB.x > relativeTol * separation + absoluteTol * hB.x) {
            separation = faceB.x;
            normal = dB.x > 0.0f ? box.rot.c1 : -box.rot.c1;
        }

        if (faceB.y > relativeTol * separation + absoluteTol * hB.y) {
            separation = faceB.y;
            normal = dB.y > 0.0f ? box.rot.c2 : -box.rot.c2;
        }

        return 1;
    };

    // * ===================================
    // * Box2D vs Primitives
    // * ===================================

    // Determine if a Box2D intersects a point.
    inline bool Box2DAndPoint(Primitives::Box2D const &box, ZMath::Vec2D const &point) { return PointAndBox2D(point, box); };

    // Determine if a Box2D intersects a line.
    inline bool Box2DAndLine(Primitives::Box2D const &box, Primitives::Line2D const &line) { return LineAndBox2D(line, box); };

    // Determine if a Box2D intersects a circle.
    inline bool Box2DAndCircle(Primitives::Box2D const &box, Primitives::Circle const &circle) { return CircleAndBox2D(circle, box); };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool Box2DAndCircle(Primitives::Box2D const &box, Primitives::Circle const &circle, ZMath::Vec2D &normal) {
        bool hit = CircleAndBox2D(circle, box, normal);
        normal = -normal;
        return hit;
    };

    // Determine if a Box2D intersects an AABB.
    inline bool Box2DAndAABB(Primitives::Box2D const &box, Primitives::AABB const &aabb) { return AABBAndBox2D(aabb, box); };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool Box2DAndAABB(Primitives::Box2D const &box, Primitives::AABB const &aabb, ZMath::Vec2D &normal) {
        bool hit = AABBAndBox2D(aabb, box, normal);
        normal = -normal;
        return hit;
    };

    // Determine if a Box2D intersects another Box2D.
    inline bool Box2DAndBox2D(Primitives::Box2D const &box1, Primitives::Box2D const &box2) {
        // ? Use the separating axis theorem to determine if there is an intersection between the Box2Ds.

        // half size of Box2D a and b respectively
        ZMath::Vec2D hA = box1.getHalfsize(), hB = box2.getHalfsize();

        // rotate anything from global space to A's local space
        ZMath::Mat2D rotAT = box1.rot.transpose();

        // determine the difference between the positions
        ZMath::Vec2D dP = box2.pos - box1.pos;
        ZMath::Vec2D dA = rotAT * dP;
        ZMath::Vec2D dB = box2.rot.transpose() * dP;

        // * rotation matrices for switching between local spaces
        
        // ! When we have a proper scene to test, use that to check if the absolute value is necessary

        // Rotate anything from B's local space into A's
        ZMath::Mat2D C = ZMath::abs(rotAT * box2.rot);

        // Rotate anything from A's local space into B's
        ZMath::Mat2D CT = C.transpose();

        // * Check for intersections with the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec2D faceA = ZMath::abs(dA) - hA - C * hB;
        if (faceA.x > 0 || faceA.y > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec2D faceB = ZMath::abs(dB) - hB - CT * hA;
        return faceB.x <= 0 && faceB.y <= 0;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    bool Box2DAndBox2D(Primitives::Box2D const &box1, Primitives::Box2D const &box2, ZMath::Vec2D &normal) {
        // ? Use the separating axis theorem to determine if there is an intersection between the Box2Ds.

        // half size of Box2D a and b respectively
        ZMath::Vec2D hA = box1.getHalfsize(), hB = box2.getHalfsize();

        // rotate anything from global space to A's local space
        ZMath::Mat2D rotAT = box1.rot.transpose();

        // determine the difference between the positions
        ZMath::Vec2D dP = box2.pos - box1.pos;
        ZMath::Vec2D dA = rotAT * dP;
        ZMath::Vec2D dB = box2.rot.transpose() * dP;

        // * rotation matrices for switching between local spaces
        
        // ! When we have a proper scene to test, use that to check if the absolute value is necessary

        // Rotate anything from B's local space into A's
        ZMath::Mat2D C = ZMath::abs(rotAT * box2.rot);

        // Rotate anything from A's local space into B's
        ZMath::Mat2D CT = C.transpose();

        // * Check for intersections with the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec2D faceA = ZMath::abs(dA) - hA - C * hB;
        if (faceA.x > 0 || faceA.y > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec2D faceB = ZMath::abs(dB) - hB - CT * hA;
        if (faceB.x > 0 || faceB.y > 0) { return 0; }
        
        // * Find the best axis (i.e. the axis with the least penetration).

        // Assume A's x-axis is the best axis first.
        float separation = faceA.x;
        normal = dA.x > 0.0f ? box1.rot.c1 : -box1.rot.c1;

        // tolerance values
        float relativeTol = 0.95f;
        float absoluteTol = 0.01f;

        // ? check if there is another axis better than A's x axis by checking if the penetration along
        // ?  the current axis being checked is greater than that of the current penetration
        // ?  (as greater value = less negative = less penetration).

        // A's remaining axes
        if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
            separation = faceA.y;
            normal = dA.y > 0.0f ? box1.rot.c2 : -box1.rot.c2;
        }

        // B's axes
        if (faceB.x > relativeTol * separation + absoluteTol * hB.x) {
            separation = faceB.x;
            normal = dB.x > 0.0f ? box2.rot.c1 : -box2.rot.c1;
        }

        if (faceB.y > relativeTol * separation + absoluteTol * hB.y) {
            separation = faceB.y;
            normal = dB.y > 0.0f ? box2.rot.c2 : -box2.rot.c2;
        }

        return 1;
    };
}
