#pragma once

#include "collisions.h"
#include <stdexcept>

// todo maybe refactor so that everything is in a Zeta namespace (except for the ZMath stuff)
// todo or at least make the collisions stuff in there, too

// todo definitely refactor the namespace system. Currently it is kinda annoying and makes for verbose code

namespace Zeta {
    // * ====================================
    // * Common Framerates for Handler
    // * ====================================

    #define FPS_24 0.0417f
    #define FPS_30 0.0333f
    #define FPS_40 0.025f
    #define FPS_50 0.02f
    #define FPS_60 0.0167f

    // * =========================
    // * Impulse Resolution
    // * =========================

    // Resolve a collision between two rigidbodies.
    extern void applyImpulse(Primitives::RigidBody2D* rb1, Primitives::RigidBody2D* rb2, Collisions::CollisionManifold const &manifold);

    // todo technically could pass just purely the rigid and kinematic bodies for their impulse resolution vs static bodies
    // todo test to ensure these are sufficient for impulse resolution ------------------------------------------------------------------
    // Resolve a collision between a rigid and static body.
    extern void applyImpulse(Primitives::RigidBody2D* rb, Primitives::StaticBody2D* sb, Collisions::CollisionManifold const &manifold);

    // Resolve a collision between a rigid and kinematic body.
    extern void applyImpulse(Primitives::RigidBody2D* rb, Primitives::KinematicBody2D* kb, Collisions::CollisionManifold const &manifold);

    // Resolve a collision between a static and kinematic body.
    extern void applyImpulse(Primitives::KinematicBody2D* kb, Primitives::StaticBody2D* sb, Collisions::CollisionManifold const &manifold);

    // Resolve a collision between two kinematic bodies.
    extern void applyImpulse(Primitives::KinematicBody2D* kb1, Primitives::KinematicBody2D* kb2, Collisions::CollisionManifold const &manifold);

    // todo -----------------------------------------------------------------------------------------------------------------------------


    // * ==============
    // * Wrappers
    // * ==============

    // todo typedef the wrappers instead
    // these are also likely just placeholders until we can find a good value
    static const int startingSlots = 64;
    static const int halfStartingSlots = 32;
    static const int kStartingSlots = 4;
    static const int kHalfStartingSlots = 2;

    // ? For now, default to allocating 64 slots for Objects. Adjust once we start implementing more stuff.

    // * Body structs.

    struct RigidBodies {
        Primitives::RigidBody2D** rigidBodies = nullptr; // list of active rigid bodies
        int capacity; // current max capacity
        int count; // number of rigid bodies 
    };

    struct StaticBodies {
        Primitives::StaticBody2D** staticBodies = nullptr; // list of active static bodies
        int capacity; // current max capacity
        int count;  // number of static bodies
    };

    struct KinematicBodies {
        Primitives::KinematicBody2D** kinematicBodies = nullptr; // list of active kinematic bodies
        int capacity; // current max capacity
        int count; // number of kinematic bodies
    };


    // * CollisionWrapper Structs.

    struct CollisionWrapper {
        Primitives::RigidBody2D** bodies1 = nullptr; // list of colliding bodies (Object A)
        Primitives::RigidBody2D** bodies2 = nullptr; // list of colliding bodies (Object B)
        Collisions::CollisionManifold* manifolds = nullptr; // list of the collision manifolds between the objects

        int capacity; // current max capacity
        int count; // number of collisions
    };

    struct StaticCollisionWrapper {
        Primitives::StaticBody2D** sbs = nullptr; // list of colliding static bodies (Object A)
        Primitives::RigidBody2D** rbs = nullptr; // list of colliding rigid bodies (Object B)
        Collisions::CollisionManifold* manifolds = nullptr; // list of the collision manifolds between the objects

        int capacity; // current max capacity
        int count; // number of collisions
    };

    // Store data about collisions between rigid and kinematic bodies
    struct RkCollisionWrapper {
        Primitives::RigidBody2D** rbs = nullptr;
        Primitives::KinematicBody2D** kbs = nullptr;
        Collisions::CollisionManifold* manifolds = nullptr;

        int capacity;
        int count;
    };

    // Store data about collisions between kinematic and static bodies
    struct SkCollisionWrapper {
        Primitives::KinematicBody2D** kbs = nullptr;
        Primitives::StaticBody2D** sbs = nullptr;
        Collisions::CollisionManifold* manifolds = nullptr;

        int capacity;
        int count;
    };

    // Store data about collisions between two kinematic bodies
    struct KinematicCollisionWrapper {
        Primitives::KinematicBody2D** kb1s = nullptr;
        Primitives::KinematicBody2D** kb2s = nullptr;
        Collisions::CollisionManifold* manifolds = nullptr;

        int capacity;
        int count;
    };


    // * ========================
    // * Main Physics Handler
    // * ========================

    class Handler {
        private:
            // * =================
            // * Attributes
            // * =================

            RigidBodies rbs; // rigid bodies to update
            StaticBodies sbs; // static bodies to update
            KinematicBodies kbs; // kinematic bodies to update
            CollisionWrapper colWrapper; // collision information
            StaticCollisionWrapper staticColWrapper; // collision information involving static body collisions
            RkCollisionWrapper rkColWrapper; // collision information involving rigid and kinematic body collisions
            SkCollisionWrapper skColWrapper; // collision information involving static and kinematic body collisions
            KinematicCollisionWrapper kColWrapper; // collision information involving kinematic body collisions
            float updateStep; // amount of dt to update after
            static const int IMPULSE_ITERATIONS = 6; // number of times to apply the impulse update.


            // * ==============================
            // * Functions for Ease of Use
            // * ==============================

            void addCollision(Primitives::RigidBody2D* rb1, Primitives::RigidBody2D* rb2, Collisions::CollisionManifold const &manifold);
            void addCollision(Primitives::RigidBody2D* rb, Primitives::StaticBody2D* sb, Collisions::CollisionManifold const &manifold);
            void addCollision(Primitives::RigidBody2D* rb, Primitives::KinematicBody2D* kb, Collisions::CollisionManifold const &manifold);
            void addCollision(Primitives::StaticBody2D* sb, Primitives::KinematicBody2D* kb, Collisions::CollisionManifold const &manifold);
            void addCollision(Primitives::KinematicBody2D* kb1, Primitives::KinematicBody2D* kb2, Collisions::CollisionManifold const &manifold);
            void clearCollisions();

        public:
            // * =====================
            // * Public Attributes
            // * =====================

            ZMath::Vec2D g; // gravity


            // * ===================================
            // * Constructors, Destructors, Etc.
            // * ===================================

            // Make a physics handler with a default gravity of -9.8 and an update speed of 60FPS.

            /**
             * @brief Create a physics handler.
             * 
             * @param g (Vec3D) The force applied by gravity. Default of <0, 0, -9.8f>.
             * @param timeStep (float) The amount of time in seconds that must pass before the handler updates physics.
             *    Default speed of 60FPS. Anything above 60FPS is not recommended as it can cause lag in lower end hardware.
             */
            Handler(ZMath::Vec2D const &g = ZMath::Vec2D(0, -9.8f), float timeStep = FPS_60);

            // Do not allow for construction from an existing physics handler.
            Handler(Handler const &handler);
            Handler(Handler&& handler);

            // The physics handler cannot be reassigned.
            Handler& operator = (Handler const &handler);
            Handler& operator = (Handler&& handler);

            ~Handler();


            // * ============================
            // * RigidBody List Functions
            // * ============================

            // Add a rigid body to the list of rigid bodies to be updated.
            void addRigidBody(Primitives::RigidBody2D* rb);

            // Add a list of rigid bodies to the handler.
            void addRigidBodies(Primitives::RigidBody2D** rbs, int size);

            // Remove a rigid body from the handler.
            // 1 = rigid body was found and removed. 0 = It was not found.
            // rb will be deleted if the rigid body was found.
            bool removeRigidBody(Primitives::RigidBody2D* rb);


            // * ============================
            // * StaticBody List Functions
            // * ============================

            // Add a static body to the handler.
            void addStaticBody(Primitives::StaticBody2D* sb);

            // Add a list of static bodies to the handler.
            void addStaticBodies(Primitives::StaticBody2D** sbs, int size);

            // Remove a static body from the handler.
            // 1 = static body was found and removed. 0 = It was not found.
            // sb will be deleted if the static body was found.
            bool removeStaticBody(Primitives::StaticBody2D* sb);


            // * ================================
            // * KinematicBody List Functions
            // * ================================

            // Add a kinematic body to the handler.
            void addKinematicBody(Primitives::KinematicBody2D* kb);

            // Add a list of kinematic bodies to the handler.
            void addKinematicBodies(Primitives::KinematicBody2D** kbs, int size);

            // Remove a kinematic body from the handler.
            // 1 = kinematic body was found and removed. 0 = It was not found.
            // kb will be deleted if the kinematic body was found.
            bool removeKinematicBody(Primitives::KinematicBody2D* kb);


            // * ============================
            // * Main Physics Functions
            // * ============================

            // Update the physics.
            // dt will be updated to the appropriate value after the updates run for you so DO NOT modify it yourself.
            int update(float &dt);
    };
}
