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

    enum KinematicBodyCollider {
        KINEMATIC_CIRCLE_COLLIDER,
        KINEMATIC_AABB_COLLIDER,
        KINEMATIC_BOX2D_COLLIDER,
        KINEMATIC_CUSTOM_COLLIDER,
        KINEMATIC_NONE
    };


    class RigidBody2D {
        public:
            // Remember to specify the necessary fields before using the RigidBody2D if using the default constructor.
            // Consult documentation for those fields if needed.
            inline RigidBody2D() {}; // Default constructor to make the compiler happy (for efficiency).

            /**
             * @brief Create a 2D RigidBody.
             * 
             * @param pos Centerpoint of the rigidbody.
             * @param mass Mass of the rigidbody.
             * @param cor The coefficient of restituion of the rigidbody. This represents a loss of kinetic energy due to heat and should be
             *              between 0 and 1 inclusive with 1 being perfectly elastic.
             * @param linearDamping The linear damping of the rigid body. This should fall on (0, 1].
             * @param colliderType The type of collider attached to the rigidbody. This should be set to RIGID_NONE if there will not be one attached.
             * @param collider A pointer to the collider of the rigid body. If this does not match the colliderType specified, it will
             *                   cause undefined behvior to occur. If you specify RIGID_NONE, this should be set to nullptr. 
             */
            inline RigidBody2D(ZMath::Vec2D const &pos, float mass, float cor, float linearDamping, RigidBodyCollider colliderType, void* collider) 
                    : pos(pos), mass(mass), invMass(1.0f/mass), cor(cor), linearDamping(linearDamping), colliderType(colliderType)
            {
                switch(colliderType) {
                    case RIGID_CIRCLE_COLLIDER: { this->collider.circle = *((Circle*) collider); }
                    case RIGID_AABB_COLLIDER: { this->collider.aabb = *((AABB*) collider); }
                    case RIGID_BOX2D_COLLIDER: { this->collider.box = *((Box2D*) collider); }
                    // * User defined colliders go here.
                }
            };

            // * Handle and store the collider.

            RigidBodyCollider colliderType;
            union Collider {
                Collider() {}; // to make the compiler happy

                Circle circle;
                AABB aabb;
                Box2D box;
                // * Add custom colliders here.
            } collider;

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
            ZMath::Vec2D vel; // velocity of the rigidbody.
            ZMath::Vec2D netForce; // sum of all forces acting on the rigidbody.

            inline void update(ZMath::Vec2D const &g, float dt) {
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
    };


    class StaticBody2D {
        public:
            inline StaticBody2D() {}; // Default constructor to make the compiler happy (for efficiency).

            /**
             * @brief Create a 2D staticbody.
             * 
             * @param pos The centerpoint of the staticbody.
             * @param colliderType The type of collider attached to the staticbody. This should be set to STATIC_NONE if there will not be one attached.
             * @param collider A pointer to the collider of the static body. If this does not match the colliderType specified, it will
             *                   cause undefined behvior to occur. If you specify STATIC_NONE, this should be set to nullptr. 
             */
            inline StaticBody2D(ZMath::Vec2D const &pos, StaticBodyCollider colliderType, void* collider) : pos(pos), colliderType(colliderType) {
                switch(colliderType) {
                    case STATIC_CIRCLE_COLLIDER: { this->collider.circle = *((Circle*) collider); }
                    case STATIC_AABB_COLLIDER: { this->collider.aabb = *((AABB*) collider); }
                    case STATIC_BOX2D_COLLIDER: { this->collider.box = *((Box2D*) collider); }
                    // * User defined colliders go here.
                }
            };

            // * Information related to the static body.

            ZMath::Vec2D pos; // centerpoint of the staticbody.

            // * Handle and store the collider.

            StaticBodyCollider colliderType;
            union Collider {
                inline Collider() {}; // default constructor to make the compiler happy

                Circle circle;
                AABB aabb;
                Box2D box;
            } collider;
    };


    class KinematicBody2D {
        public:
            // Remember to specify necessary fields when using the default constructor.
            inline KinematicBody2D() {}; // Default constructor to make the compiler happy (for efficiency).

            inline KinematicBody2D(ZMath::Vec2D const &pos, KinematicBodyCollider colliderType, void* collider) : pos(pos), colliderType(colliderType) {
                switch(colliderType) {
                    case KINEMATIC_CIRCLE_COLLIDER: { this->collider.circle = *((Circle*) collider); }
                    case KINEMATIC_AABB_COLLIDER: { this->collider.aabb = *((AABB*) collider); }
                    case KINEMATIC_BOX2D_COLLIDER: { this->collider.box = *((Box2D*) collider); }
                    // * User defined colliders go here.
                }
            };

            // * Information related to the kinematic body.

            ZMath::Vec2D pos; // centerpoint of the kinematicbody.
            ZMath::Vec2D vel; // velocity of the kinematicbody.
            ZMath::Vec2D netForce; // sum of all forces acting upon the kinematicbody.

            // * Handle and store the collider.

            KinematicBodyCollider colliderType;
            union Collider {
                inline Collider() {};

                Circle circle;
                AABB aabb;
                Box2D box;
            } collider;
    };
}
