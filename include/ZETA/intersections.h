#pragma once

#include "bodies.h"

namespace Zeta {
    // * ===================================
    // * Point vs Primitives
    // * ===================================

    // Determine if a point lays on a line.
    extern bool PointAndLine(ZMath::Vec2D const &point, Line2D const &line);

    // Determine if a point lays within a circle.
    extern bool PointAndCircle(ZMath::Vec2D const &point, Circle const &circle);

    // Determine if a point lays within an AABB.
    extern bool PointAndAABB(ZMath::Vec2D const &point, AABB const &aabb);

    // Determine if a point lays within a Box2D.
    extern bool PointAndBox2D(ZMath::Vec2D const &point, Box2D const &box);

    // * ===================================
    // * Line2D vs Primitives
    // * ===================================

    // Determine if a line intersects a point.
    inline bool LineAndPoint(Line2D const &line, ZMath::Vec2D const &point) { return PointAndLine(point, line); };

    // Determine if a line intersects another line.
    extern bool LineAndLine(Line2D const &line1, Line2D const &line2);

    // Determine if a line intersects a circle.
    extern bool LineAndCircle(Line2D const &line, Circle const &circle);

    // Determine if a line intersects an AABB.
    // todo test
    extern bool LineAndAABB(Line2D const &line, AABB const &aabb);

    // Determine if a line intersects a Box2D.
    extern bool LineAndBox2D(Line2D const &line, Box2D const &box);

    // * =================
    // * Raycasting
    // * =================

    // Determine if a ray intersects a circle.
    // dist will be modified to equal the distance from the ray it hits the circle.
    // dist is set to -1 if there is no intersection.
    extern bool raycast(Circle const &circle, Ray2D const &ray, float &dist);

    // Determine if a ray intersects an AABB.
    // dist will be modified to equal the distance from the ray it hits the AABB.
    // dist is set to -1 if there is no intersection.
    extern bool raycast(AABB const &aabb, Ray2D const &ray, float &dist);

    // Determine if a ray intersects a Box2D.
    // dist will be modified to equal the distance from the ray it hits the Box2D.
    // dist is set to -1 if there is no intersection.
    extern bool raycast(Box2D const &box, Ray2D const &ray, float &dist);

    // * ===================================
    // * Circle vs Primitives
    // * ===================================

    // Determine if a circle intersects a point.
    inline bool CircleAndPoint(Circle const &circle, ZMath::Vec2D const &point) { return PointAndCircle(point, circle); };

    // Determine if a circle intersects a line.
    inline bool CircleAndLine(Circle const &circle, Line2D const &line) { return LineAndCircle(line, circle); };

    // Determine if a circle intersects another circle.
    extern bool CircleAndCircle(Circle const &circle1, Circle const &circle2);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool CircleAndCircle(Circle const &circle1, Circle const &circle2, ZMath::Vec2D &normal);

    // Determine if a circle intersects an AABB.
    extern bool CircleAndAABB(Circle const &circle, AABB const &aabb);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool CircleAndAABB(Circle const &circle, AABB const & aabb, ZMath::Vec2D &normal);

    // Determine if a circle intersects a Box2D.
    extern bool CircleAndBox2D(Circle const &circle, Box2D const &box);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool CircleAndBox2D(Circle const &circle, Box2D const &box, ZMath::Vec2D &normal);

    // * ===================================
    // * AABB vs Primitives
    // * ===================================

    // Determine if an AABB intersects a point.
    inline bool AABBAndPoint(AABB const &aabb, ZMath::Vec2D const &point) { return PointAndAABB(point, aabb); };

    // Determine if an AABB intersects a line.
    inline bool AABBAndLine(AABB const &aabb, Line2D const &line) { return LineAndAABB(line, aabb); };

    // Determine if an AABB intersects a circle.
    inline bool AABBAndCircle(AABB const &aabb, Circle const &circle) { return CircleAndAABB(circle, aabb); };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool AABBAndCircle(AABB const &aabb, Circle const &circle, ZMath::Vec2D &normal) {
        bool hit = CircleAndAABB(circle, aabb, normal);
        normal = -normal;
        return hit;
    };

    // Determine if an AABB intersects another AABB.
    extern bool AABBAndAABB(AABB const &aabb1, AABB const &aabb2);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool AABBAndAABB(AABB const &aabb1, AABB const &aabb2, ZMath::Vec2D &normal);

    // Determine if an AABB intersects a Box2D.
    extern bool AABBAndBox2D(AABB const &aabb, Box2D const &box);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool AABBAndBox2D(AABB const &aabb, Box2D const &box, ZMath::Vec2D &normal);

    // * ===================================
    // * Box2D vs Primitives
    // * ===================================

    // Determine if a Box2D intersects a point.
    inline bool Box2DAndPoint(Box2D const &box, ZMath::Vec2D const &point) { return PointAndBox2D(point, box); };

    // Determine if a Box2D intersects a line.
    inline bool Box2DAndLine(Box2D const &box, Line2D const &line) { return LineAndBox2D(line, box); };

    // Determine if a Box2D intersects a circle.
    inline bool Box2DAndCircle(Box2D const &box, Circle const &circle) { return CircleAndBox2D(circle, box); };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool Box2DAndCircle(Box2D const &box, Circle const &circle, ZMath::Vec2D &normal) {
        bool hit = CircleAndBox2D(circle, box, normal);
        normal = -normal;
        return hit;
    };

    // Determine if a Box2D intersects an AABB.
    inline bool Box2DAndAABB(Box2D const &box, AABB const &aabb) { return AABBAndBox2D(aabb, box); };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    inline bool Box2DAndAABB(Box2D const &box, AABB const &aabb, ZMath::Vec2D &normal);

    // Determine if a Box2D intersects another Box2D.
    extern bool Box2DAndBox2D(Box2D const &box1, Box2D const &box2);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool Box2DAndBox2D(Box2D const &box1, Box2D const &box2, ZMath::Vec2D &normal);
}
