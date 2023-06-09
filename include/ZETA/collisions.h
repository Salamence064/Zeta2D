#pragma once

#include "intersections.h"

namespace Collisions {
    // * =========================
    // * Collision Manifolds
    // * =========================

    // ? Note: if we have objects A and B colliding, the collison normal will point towards B and away from A.

    // todo maybe make the contactPoints list an array of 2 as that's the max number
    // this would allow for more efficiency in box2D v box2D manifolds

    struct CollisionManifold {
        ZMath::Vec2D normal; // collision normal
        ZMath::Vec2D* contactPoints; // contact points of the collision
        float pDist; // penetration distance
        int numPoints; // number of contact points
        bool hit; // do they intersect
    };
}
