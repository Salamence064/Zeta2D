#pragma once

#include "bodies.h"

namespace Collisions {
    // * ===================================
    // * Point vs Primitives
    // * ===================================

    // Determine if a point lays on a line.
    extern bool PointAndLine(ZMath::Vec2D const &point, Primitives::Line2D const &line);

    // Determine if a point lays within a circle.
    extern bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);

    // Determine if a point lays within an AABB.
    extern bool PointAndAABB(ZMath::Vec2D const &point, Primitives::AABB const &aabb);

    // Determine if a point lays within a Box2D.
    extern bool PointAndBox2D(ZMath::Vec2D const &point, Primitives::Box2D const &box);

    // * ===================================
    // * Line2D vs Primitives
    // * ===================================

    // Determine if a line intersects a point.
    inline bool LineAndPoint(Primitives::Line2D const &line, ZMath::Vec2D const &point) { return PointAndLine(point, line); };

    // Determine if a line intersects another line.
    extern bool LineAndLine(Primitives::Line2D const &line1, Primitives::Line2D const &line2);

    // Determine if a line intersects a circle.
    extern bool LineAndCircle(Primitives::Line2D const &line, Primitives::Circle const &circle);

    // Determine if a line intersects an AABB.
    // todo test
    extern bool LineAndAABB(Primitives::Line2D const &line, Primitives::AABB const &aabb);

    // Determine if a line intersects a Box2D.
    extern bool LineAndBox2D(Primitives::Line2D const &line, Primitives::Box2D const &box);

    // * =================
    // * Raycasting
    // * =================

    // Determine if a ray intersects a circle.
    // dist will be modified to equal the distance from the ray it hits the circle.
    // dist is set to -1 if there is no intersection.
    extern bool raycast(Primitives::Circle const &circle, Primitives::Ray2D const &ray, float &dist);

    // Determine if a ray intersects an AABB.
    // dist will be modified to equal the distance from the ray it hits the AABB.
    // dist is set to -1 if there is no intersection.
    extern bool raycast(Primitives::AABB const &aabb, Primitives::Ray2D const &ray, float &dist);

    // Determine if a ray intersects a Box2D.
    // dist will be modified to equal the distance from the ray it hits the Box2D.
    // dist is set to -1 if there is no intersection.
    extern bool raycast(Primitives::Box2D const &box, Primitives::Ray2D const &ray, float &dist);

    // * ===================================
    // * Circle vs Primitives
    // * ===================================

    // Determine if a circle intersects a point.
    inline bool CircleAndPoint(Primitives::Circle const &circle, ZMath::Vec2D const &point) { return PointAndCircle(point, circle); };

    // Determine if a circle intersects a line.
    inline bool CircleAndLine(Primitives::Circle const &circle, Primitives::Line2D const &line) { return LineAndCircle(line, circle); };

    // Determine if a circle intersects another circle.
    extern bool CircleAndCircle(Primitives::Circle const &circle1, Primitives::Circle const &circle2);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool CircleAndCircle(Primitives::Circle const &circle1, Primitives::Circle const &circle2, ZMath::Vec2D &normal);

    // Determine if a circle intersects an AABB.
    extern bool CircleAndAABB(Primitives::Circle const &circle, Primitives::AABB const &aabb);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool CircleAndAABB(Primitives::Circle const &circle, Primitives::AABB const & aabb, ZMath::Vec2D &normal);

    // Determine if a circle intersects a Box2D.
    extern bool CircleAndBox2D(Primitives::Circle const &circle, Primitives::Box2D const &box);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool CircleAndBox2D(Primitives::Circle const &circle, Primitives::Box2D const &box, ZMath::Vec2D &normal);

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
    extern bool AABBAndAABB(Primitives::AABB const &aabb1, Primitives::AABB const &aabb2);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool AABBAndAABB(Primitives::AABB const &aabb1, Primitives::AABB const &aabb2, ZMath::Vec2D &normal);

    // Determine if an AABB intersects a Box2D.
    extern bool AABBAndBox2D(Primitives::AABB const &aabb, Primitives::Box2D const &box);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool AABBAndBox2D(Primitives::AABB const &aabb, Primitives::Box2D const &box, ZMath::Vec2D &normal);

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
    inline bool Box2DAndAABB(Primitives::Box2D const &box, Primitives::AABB const &aabb, ZMath::Vec2D &normal);

    // Determine if a Box2D intersects another Box2D.
    extern bool Box2DAndBox2D(Primitives::Box2D const &box1, Primitives::Box2D const &box2);

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    extern bool Box2DAndBox2D(Primitives::Box2D const &box1, Primitives::Box2D const &box2, ZMath::Vec2D &normal);
}
