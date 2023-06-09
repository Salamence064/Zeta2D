#pragma once

#include "primitives.h"

namespace Primitives {
    enum RigidBodyCollider {
        RIGID_CIRCLE_COLLIDER,
        RIGID_AABB_COLLIDER,
        RIGID_BOX2D_COLLIDER,
        RIGID_CUSTOM_COLLIDER,
        RIGID_NONE
    };

    enum StaticBodyCollider {
        STATIC_CIRCLE_COLLIDER,
        STATIC_AABB_COLLIDER,
        STATIC_BOX2D_COLLIDER,
        STATIC_CUSTOM_COLLIDER,
        STATIC_NONE
    };

    struct RigidBody2D {
        // Remember to specify the necessary fields before using the RigidBody2D if using the default constructor.
        // Consult documentation for those fields if needed.
        RigidBody2D() {}; // Default constructor to make the compiler happy (for efficiency).

        /**
         * @brief Create a 2D RigidBody.
         * 
         * @param pos Centerpoint of the rigidbody.
         * @param mass Mass of the rigidbody.
         * @param cor The coefficient of restituion of the rigidbody. This represents a loss of kinetic energy due to heat and should be
         *              between 0 and 1 inclusive with 1 being perfectly elastic.
         * @param colliderType The type of collider attached to the rigidbody. This should be set to RIGID_NONE if there will not be one attached.
         *                       Remember to manually assign a value to sphere, aabb, or cube depending on the collider type specified. DO NOT
         *                       assign a value to a collider other than the one corresponding to the type specified.
         */
        RigidBody2D(ZMath::Vec2D const &pos, float mass, float cor, float linearDamping, RigidBodyCollider colliderType) 
                : pos(pos), mass(mass), invMass(1.0f/mass), cor(cor), linearDamping(linearDamping), colliderType(colliderType) {};

        // * Handle and store the collider.

        RigidBodyCollider colliderType;
        union {
            Circle circle;
            AABB aabb;
            Box2D box;
            // * Add custom colliders here.
        };

        // * Handle and store the physics.

        float mass; // Must remain constant.
        float invMass; // 1/mass. Must remain constant.

        // Coefficient of Restitution.
        // Represents a loss of kinetic energy due to heat.
        // Between 0 and 1 for our purposes.
        // 1 = perfectly elastic.
        float cor;

        // Linear damping.
        // Acts as linear friction on the rigidbody.
        float linearDamping;

        ZMath::Vec2D pos; // centerpoint of the rigidbody.
        ZMath::Vec2D vel = ZMath::Vec2D(); // velocity of the rigidbody.
        ZMath::Vec2D netForce = ZMath::Vec2D(); // sum of all forces acting on the rigidbody.

        void update(ZMath::Vec2D const &g, float dt) {
            // ? assuming g is gravity, and it is already negative
            netForce += g * mass;
            vel += (netForce * invMass) * dt;
            pos += vel * dt;
            netForce = ZMath::Vec2D();

            // Update the pos of the collider.
            // If statements are more readable than a switch here.
            if      (colliderType == RIGID_CIRCLE_COLLIDER) { circle.c = pos; }
            else if (colliderType == RIGID_AABB_COLLIDER)   { aabb.pos = pos; }
            else if (colliderType == RIGID_BOX2D_COLLIDER)  { box.pos = pos;  }
        };
    };

    struct StaticBody3D {
        StaticBody3D() {}; // Default constructor to make the compiler happy (for efficiency).

        /**
         * @brief Create a 3D staticbody.
         * 
         * @param pos The centerpoint of the staticbody.
         * @param colliderType The type of collider attached to the staticbody. This should be set to STATIC_NONE if there will not be one attached.
         *                       Remember to manually assign a value to plane, sphere, aabb, or cube depending on the collider type specified.
         *                       DO NOT assign a value to a collider other than the one corresponding to the type specified.
         */
        StaticBody3D(ZMath::Vec2D const &pos, StaticBodyCollider colliderType) : pos(pos), colliderType(colliderType) {};

        // * Information related to the static body.

        ZMath::Vec2D pos; // centerpoint of the staticbody.

        // * Handle and store the collider.

        StaticBodyCollider colliderType;
        union {
            Circle circle;
            AABB aabb;
            Box2D box;
        };
    };
}
