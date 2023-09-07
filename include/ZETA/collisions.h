#pragma once

#include "intersections.h"

namespace Collisions {
    // * =========================
    // * Collision Manifolds
    // * =========================

    // ? Note: if we have objects A and B colliding, the collison normal will point towards B and away from A.

    struct CollisionManifold {
        ZMath::Vec2D normal; // collision normal
        ZMath::Vec2D* contactPoints; // contact points of the collision
        float pDist; // penetration distance
        int numPoints; // number of contact points
        bool hit; // do they intersect
    };

    // * ==========================================
    // * Enum to Help with Collision Manifolds
    // * ==========================================

    // * Enums used for denotating the edges of the Box2Ds
    enum Axis {
        FACE_A_X,
        FACE_A_Y,
        FACE_B_X,
        FACE_B_Y
    };


    // * ===================================
    // * Collision Manifold Calculators
    // * ===================================

    extern CollisionManifold findCollisionFeatures(Primitives::Circle const &circle1, Primitives::Circle const &circle2);
    extern CollisionManifold findCollisionFeatures(Primitives::Circle const &circle, Primitives::AABB const &aabb);
    extern CollisionManifold findCollisionFeatures(Primitives::Circle const &circle, Primitives::Box2D const &box);


    // * ====================================================
    // * Helper Functions for 2D Box Collision Manifolds
    // * ====================================================

    /**
     * @brief Determine the 2 vertices comprising the incident face when the incident Box2D is an AABB.
     * 
     * @param v Array which gets filled with the 2 vertices making up the incident face.
     * @param h Halfsize of the incident AABB.
     * @param pos The position of the incident AABB.
     * @param normal The normal vector of the collision (points towards B away from A).
     */
    extern void computeIncidentFaceAABB(ZMath::Vec2D v[2], const ZMath::Vec2D& h, const ZMath::Vec2D& pos, const ZMath::Vec2D& normal);

    /**
     * @brief Determine the 2 vertices making up the incident face.
     * 
     * @param v Array which gets filled with the 2 vertices comprising the incident face.
     * @param h Halfsize of the incident Box2D.
     * @param pos The position of the incident Box2D.
     * @param rot The rotation matrix of the incident Box2D.
     * @param normal The normal vector of the collision.
     */
    extern void computeIncidentFace(ZMath::Vec2D v[2], const ZMath::Vec2D& h, const ZMath::Vec2D& pos, 
                                    const ZMath::Mat2D& rot, const ZMath::Vec2D& normal);

    /**
     * @brief Compute the clipping points given input points.
     * 
     * @param vOut Array which gets filled with the clipping points.
     * @param vIn Array containing the input points.
     * @param n Side normal.
     * @param offset Distance to the side corresponding with side normal.
     * @return (int) The number of clipping points. If this does not return 2, there is not an intersection on this axis.
     */
    extern int clipSegmentToLine(ZMath::Vec2D vOut[2], ZMath::Vec2D vIn[2], const ZMath::Vec2D &n, float offset);

    // ? Normal points towards B and away from A

    extern CollisionManifold findCollisionFeatures(Primitives::AABB const &aabb1, Primitives::AABB const &aabb2);

    // ? Normal points towards B and away from A

    extern CollisionManifold findCollisionFeatures(Primitives::AABB const &aabb, Primitives::Box2D const &box);

    // ? Normal points towards B and away from A

    extern CollisionManifold findCollisionFeatures(Primitives::Box2D const &box1, Primitives::Box2D const &box2);

    // Find the collision features and resolve the impulse between two arbitrary primitives.
    // The normal will point towards B and away from A.
    extern CollisionManifold findCollisionFeatures(Primitives::RigidBody2D* rb1, Primitives::RigidBody2D* rb2);

    // Find the collision features between a rigid and static body.
    // The normal will point away from the static body and towards the rigid body.
    extern CollisionManifold findCollisionFeatures(Primitives::RigidBody2D* rb, Primitives::StaticBody2D* sb);

    // Find the collision features between a rigid and kinematic body.
    // The normal will point away from the kinematic body and towards the rigid body.
    extern CollisionManifold findCollisionFeatures(Primitives::RigidBody2D* rb, Primitives::KinematicBody2D* kb);

    // Find the collision features between a kinematic and static body.
    // The normal will point away from the static body and towards the kinematic body.
    extern CollisionManifold findCollisionFeatures(Primitives::KinematicBody2D* kb, Primitives::StaticBody2D* sb);

    // Find the collision features between two kinematic bodies.
    // The normal points towards B and away from A.
    extern CollisionManifold findCollisionFeatures(Primitives::KinematicBody2D* kb1, Primitives::KinematicBody2D* kb2);
}
