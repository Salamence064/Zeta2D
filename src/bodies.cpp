#include <ZETA/bodies.h>

namespace Zeta {
    // * ===============
    // * RigidBody2D
    // * ===============

    void RigidBody2D::update(ZMath::Vec2D const &g, float dt) {
        // ? assuming g is gravity, and it is already negative
        netForce += g * mass;
        vel += (netForce * invMass) * dt;
        pos += vel * dt;

        vel *= linearDamping;
        netForce.zero();

        // Update the pos of the collider.
        // If statements are more readable than a switch here.
        if      (colliderType == RIGID_CIRCLE_COLLIDER) { collider.circle.c = pos; }
        else if (colliderType == RIGID_AABB_COLLIDER)   { collider.aabb.pos = pos; }
        else if (colliderType == RIGID_BOX2D_COLLIDER)  { collider.box.pos = pos;  }
    };
}
