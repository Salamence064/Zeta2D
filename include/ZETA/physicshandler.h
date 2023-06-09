#pragma once

#include "collisions.h"
#include <stdexcept>

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
    void applyImpulse(Primitives::RigidBody2D* rb1, Primitives::RigidBody2D* rb2, Collisions::CollisionManifold const &manifold) {
        // delta v = J/m
        // For this calculation we need to acocunt for the relative velocity between the two objects
        // v_r = v_1 - v_2
        // To determine the total velocity of the collision: v_j = -(1 + cor)v_r dot collisionNormal
        // Impulse then equals: J = v_j/(invMass_1 + invMass_2)
        // v_1' = v_1 + invMass_1 * J * collisionNormal
        // v_2' = v_2 - invMass_2 * J * collisionNormal. Note the - is to account for the direction which the normal is pointing.
        // It's opposite for one of the two objects.

        float J = ((ZMath::abs(rb1->vel - rb2->vel) * -(1 + rb1->cor * rb2->cor)) * manifold.normal)/(rb1->invMass + rb2->invMass);

        rb1->vel -= manifold.normal * (rb1->invMass * J);
        rb2->vel += manifold.normal * (rb2->invMass * J);
    };

    // todo test to ensure this is sufficient for impulse resolution
    // Resolve a collision between a rigid and static body.
    void applyImpulse(Primitives::RigidBody2D* rb, Primitives::StaticBody2D* sb, Collisions::CollisionManifold const &manifold) {
        float J = ((ZMath::abs(rb->vel) * -(1 + rb->cor)) * manifold.normal)/rb->invMass;
        rb->vel -= manifold.normal * (rb->invMass * J);
    };


    // * ==============
    // * Wrappers
    // * ==============

    namespace { // make this struct private to this file
        // these are also likely just placeholders until we can find a good value
        static const int startingSlots = 64;
        static const int halfStartingSlots = 32;

        // ? For now, default to allocating 64 slots for Objects. Probably up once we start implementing more stuff.

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
    }


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
            CollisionWrapper colWrapper; // collision information
            StaticCollisionWrapper staticColWrapper; // collision information involving static body collisions
            float updateStep; // amount of dt to update after
            static const int IMPULSE_ITERATIONS = 6; // number of times to apply the impulse update.


            // * ==============================
            // * Functions for Ease of Use
            // * ==============================

            inline void addCollision(Primitives::RigidBody2D* rb1, Primitives::RigidBody2D* rb2, Collisions::CollisionManifold const &manifold) {
                if (colWrapper.count == colWrapper.capacity) {
                    colWrapper.capacity *= 2;

                    Primitives::RigidBody2D** temp1 = new Primitives::RigidBody2D*[colWrapper.capacity];
                    Primitives::RigidBody2D** temp2 = new Primitives::RigidBody2D*[colWrapper.capacity];
                    Collisions::CollisionManifold* temp3 = new Collisions::CollisionManifold[colWrapper.capacity];

                    for (int i = 0; i < colWrapper.count; i++) {
                        temp1[i] = colWrapper.bodies1[i];
                        temp2[i] = colWrapper.bodies2[i];
                        temp3[i] = colWrapper.manifolds[i];
                    }

                    delete[] colWrapper.bodies1;
                    delete[] colWrapper.bodies2;
                    delete[] colWrapper.manifolds;

                    colWrapper.bodies1 = temp1;
                    colWrapper.bodies2 = temp2;
                    colWrapper.manifolds = temp3;
                }

                colWrapper.bodies1[colWrapper.count] = rb1;
                colWrapper.bodies2[colWrapper.count] = rb2;
                colWrapper.manifolds[colWrapper.count++] = manifold;
            };

            inline void addCollision(Primitives::RigidBody2D* rb, Primitives::StaticBody2D* sb, Collisions::CollisionManifold const &manifold) {
                if (staticColWrapper.count == staticColWrapper.capacity) {
                    staticColWrapper.capacity *= 2;

                    Primitives::StaticBody2D** temp1 = new Primitives::StaticBody2D*[staticColWrapper.capacity];
                    Primitives::RigidBody2D** temp2 = new Primitives::RigidBody2D*[staticColWrapper.capacity];
                    Collisions::CollisionManifold* temp3 = new Collisions::CollisionManifold[staticColWrapper.capacity];

                    for (int i = 0; i < staticColWrapper.count; ++i) {
                        temp1[i] = staticColWrapper.sbs[i];
                        temp2[i] = staticColWrapper.rbs[i];
                        temp3[i] = staticColWrapper.manifolds[i];
                    }

                    delete[] staticColWrapper.sbs;
                    delete[] staticColWrapper.rbs;
                    delete[] staticColWrapper.manifolds;

                    staticColWrapper.sbs = temp1;
                    staticColWrapper.rbs = temp2;
                    staticColWrapper.manifolds = temp3;
                }

                staticColWrapper.sbs[staticColWrapper.count] = sb;
                staticColWrapper.rbs[staticColWrapper.count] = rb;
                staticColWrapper.manifolds[staticColWrapper.count++] = manifold;
            };

            inline void clearCollisions() {
                // ? We do not need to check for nullptrs because if this function is reached it is guarenteed none of the pointers inside of here will be NULL

                int halfRbs = rbs.capacity/2;


                // * standard collision wrapper
                delete[] colWrapper.bodies1;
                delete[] colWrapper.bodies2;

                for (int i = 0; i < colWrapper.count; ++i) { delete[] colWrapper.manifolds[i].contactPoints; }
                delete[] colWrapper.manifolds;

                colWrapper.bodies1 = new Primitives::RigidBody2D*[halfRbs];
                colWrapper.bodies2 = new Primitives::RigidBody2D*[halfRbs];
                colWrapper.manifolds = new Collisions::CollisionManifold[halfRbs];

                colWrapper.capacity = halfRbs;
                colWrapper.count = 0;


                // * Static collision wrapper
                delete[] staticColWrapper.sbs;
                delete[] staticColWrapper.rbs;

                for (int i = 0; i < staticColWrapper.count; ++i) { delete[] staticColWrapper.manifolds[i].contactPoints; }
                delete[] staticColWrapper.manifolds;

                staticColWrapper.sbs = new Primitives::StaticBody2D*[halfRbs];
                staticColWrapper.rbs = new Primitives::RigidBody2D*[halfRbs];
                staticColWrapper.manifolds = new Collisions::CollisionManifold[halfRbs];

                staticColWrapper.capacity = halfRbs;
                staticColWrapper.count = 0;
            };

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
            Handler(ZMath::Vec2D const &g = ZMath::Vec2D(0, -9.8f), float timeStep = FPS_60) : g(g), updateStep(timeStep) {
                if (updateStep < FPS_60) { updateStep = FPS_60; } // hard cap at 60 FPS

                rbs.rigidBodies = new Primitives::RigidBody2D*[startingSlots];
                rbs.capacity = startingSlots;
                rbs.count = 0;

                sbs.staticBodies = new Primitives::StaticBody2D*[startingSlots];
                sbs.capacity = startingSlots;
                sbs.count = 0;

                colWrapper.bodies1 = new Primitives::RigidBody2D*[halfStartingSlots];
                colWrapper.bodies2 = new Primitives::RigidBody2D*[halfStartingSlots];
                colWrapper.manifolds = new Collisions::CollisionManifold[halfStartingSlots];
                colWrapper.capacity = halfStartingSlots;
                colWrapper.count = 0;

                staticColWrapper.sbs = new Primitives::StaticBody2D*[halfStartingSlots];
                staticColWrapper.rbs = new Primitives::RigidBody2D*[halfStartingSlots];
                staticColWrapper.manifolds = new Collisions::CollisionManifold[halfStartingSlots];
                staticColWrapper.capacity = halfStartingSlots;
                staticColWrapper.count = 0;
            };

            // Do not allow for construction from an existing physics handler.
            Handler(Handler const &handler) { throw std::runtime_error("PhysicsHandler object CANNOT be constructed from another PhysicsHandler."); };
            Handler(Handler&& handler) { throw std::runtime_error("PhysicsHandler object CANNOT be constructed from another PhysicsHandler."); };

            // The physics handler cannot be reassigned.
            Handler& operator = (Handler const &handler) { throw std::runtime_error("PhysicsHandler object CANNOT be reassigned to another PhysicsHandler."); };
            Handler& operator = (Handler&& handler) { throw std::runtime_error("PhysicsHandler object CANNOT be reassigned to another PhysicsHandler."); };

            ~Handler() {
                // If one of the pointers is not NULL, none of them are.
                if (rbs.rigidBodies) {
                    for (int i = 0; i < rbs.count; ++i) { delete rbs.rigidBodies[i]; }
                    delete[] rbs.rigidBodies;

                    for (int i = 0; i < sbs.count; ++i) { delete sbs.staticBodies[i]; }
                    delete[] sbs.staticBodies;

                    for (int i = 0; i < colWrapper.count; ++i) {
                        delete colWrapper.bodies1[i];
                        delete colWrapper.bodies2[i];
                    }

                    // ? Note: we do not need to delete each RigidBody pointer in bodies1 and bodies2 as the main rigidBodies list
                    // ?       is guarenteed to contain those same pointers.
                    delete[] colWrapper.bodies1;
                    delete[] colWrapper.bodies2;

                    // ? We do not need to check for nullptr for contactPoints as if colWrapper.count > 0, it is guarenteed manifolds[i].contactPoints != nullptr.
                    for (int i = 0; i < colWrapper.count; ++i) { delete[] colWrapper.manifolds[i].contactPoints; }
                    delete[] colWrapper.manifolds;

                    // * Same thing but for the static collisions
                    delete[] staticColWrapper.sbs;
                    delete[] staticColWrapper.rbs;

                    for (int i = 0; i < staticColWrapper.count; ++i) { delete[] staticColWrapper.manifolds[i].contactPoints; }
                    delete[] colWrapper.manifolds;
                }
            };


            // * ============================
            // * RigidBody List Functions
            // * ============================

            // Add a rigid body to the list of rigid bodies to be updated.
            inline void addRigidBody(Primitives::RigidBody2D* rb) {
                if (rbs.count == rbs.capacity) {
                    rbs.capacity *= 2;
                    Primitives::RigidBody2D** temp = new Primitives::RigidBody2D*[rbs.capacity];

                    for (int i = 0; i < rbs.count; ++i) { temp[i] = rbs.rigidBodies[i]; }

                    delete[] rbs.rigidBodies;
                    rbs.rigidBodies = temp;
                }

                rbs.rigidBodies[rbs.count++] = rb;
            };

            // Add a list of rigid bodies to the handler.
            inline void addRigidBodies(Primitives::RigidBody2D** rbs, int size) {
                if (this->rbs.count + size > this->rbs.capacity) {
                    do { this->rbs.capacity *= 2; } while (this->rbs.count + size > this->rbs.capacity);
                    Primitives::RigidBody2D** temp = new Primitives::RigidBody2D*[this->rbs.capacity];
                
                    for (int i = 0; i < this->rbs.count; ++i) { temp[i] = this->rbs.rigidBodies[i]; }

                    delete[] this->rbs.rigidBodies;
                    this->rbs.rigidBodies = temp;
                }


                for (int i = 0; i < size; ++i) { this->rbs.rigidBodies[this->rbs.count++] = rbs[i]; }
            };

            // Remove a rigid body from the handler.
            // 1 = rigid body was found and removed. 0 = It was not found.
            // rb will be deleted if the rigid body was found.
            inline bool removeRigidBody(Primitives::RigidBody2D* rb) {
                for (int i = rbs.count - 1; i >= 0; --i) {
                    if (rbs.rigidBodies[i] == rb) {
                        delete rb;
                        for (int j = i; i < rbs.count - 1; ++j) { rbs.rigidBodies[j] = rbs.rigidBodies[j + 1]; }
                        rbs.count--;
                        return 1;
                    }
                }

                return 0;
            };


            // * ============================
            // * StaticBody List Functions
            // * ============================

            // Add a static body to the handler.
            inline void addStaticBody(Primitives::StaticBody2D* sb) {
                if (sbs.count == sbs.capacity) {
                    sbs.capacity *= 2;
                    Primitives::StaticBody2D** temp = new Primitives::StaticBody2D*[sbs.capacity];

                    for (int i = 0; i < sbs.count; ++i) { temp[i] = sbs.staticBodies[i]; }

                    delete[] sbs.staticBodies;
                    sbs.staticBodies = temp;
                }

                sbs.staticBodies[sbs.count++] = sb;
            };

            // Add a list of static bodies to the handler.
            inline void addStaticBodies(Primitives::StaticBody2D** sbs, int size) {
                if (this->sbs.count + size > this->sbs.capacity) {
                    do { this->sbs.capacity *= 2; } while(this->sbs.count + size > this->sbs.capacity);
                    Primitives::StaticBody2D** temp = new Primitives::StaticBody2D*[this->sbs.capacity];

                    for (int i = 0; i < this->sbs.count; ++i) { temp[i] = this->sbs.staticBodies[i]; }

                    delete[] this->sbs.staticBodies;
                    this->sbs.staticBodies = temp;
                }

                for (int i = 0; i < size; ++i) { this->sbs.staticBodies[this->sbs.count++] = sbs[i]; }
            };

            // Remove a static body from the handler.
            // 1 = static body was found and removed. 0 = It was not found.
            // sb will be deleted if the static body was found.
            inline bool removeStaticBody(Primitives::StaticBody2D* sb) {
                for (int i = sbs.count - 1; i >= 0; --i) {
                    if (sbs.staticBodies[i] == sb) {
                        delete sb;
                        for (int j = i; j < sbs.count - 1; ++j) { sbs.staticBodies[j] = sbs.staticBodies[j + 1]; }
                        sbs.count--;
                        return 1;
                    }
                }

                return 0;
            };


            // * ============================
            // * Main Physics Functions
            // * ============================

            // Update the physics.
            // dt will be updated to the appropriate value after the updates run for you so DO NOT modify it yourself.
            int update(float &dt) {
                int count = 0;

                while (dt >= updateStep) {
                    // Broad phase: collision detection
                    for (int i = 0; i < rbs.count - 1; ++i) {
                        for (int j = i + 1; j < rbs.count; ++j) {
                            Collisions::CollisionManifold result = Collisions::findCollisionFeatures(rbs.rigidBodies[i], rbs.rigidBodies[j]);
                            if (result.hit) { addCollision(rbs.rigidBodies[i], rbs.rigidBodies[j], result); }
                        }

                        for (int j = 0; j < sbs.count; ++j) {
                            Collisions::CollisionManifold result = Collisions::findCollisionFeatures(rbs.rigidBodies[i], sbs.staticBodies[j]);
                            if (result.hit) { addCollision(rbs.rigidBodies[i], sbs.staticBodies[j], result); }
                        }
                    }

                    // todo update to not be through iterative deepening -- look into this in the future
                    // todo use spacial partitioning
                    // Narrow phase: Impulse resolution
                    for (int k = 0; k < IMPULSE_ITERATIONS; ++k) {
                        if (colWrapper.count > staticColWrapper.count) { // staticColWrapper is the shorter of the two.
                            for (int i = 0; i < staticColWrapper.count; ++i) {
                                applyImpulse(colWrapper.bodies1[i], colWrapper.bodies2[i], colWrapper.manifolds[i]);
                                applyImpulse(staticColWrapper.rbs[i], staticColWrapper.sbs[i], staticColWrapper.manifolds[i]);
                            }

                            for (int i = staticColWrapper.count; i < colWrapper.count; ++i) {
                                applyImpulse(colWrapper.bodies1[i], colWrapper.bodies2[i], colWrapper.manifolds[i]);
                            }

                        } else { // colWrapper is the shorter or the two or they are equal.
                            for (int i = 0; i < colWrapper.count; ++i) {
                                applyImpulse(colWrapper.bodies1[i], colWrapper.bodies2[i], colWrapper.manifolds[i]);
                                applyImpulse(staticColWrapper.rbs[i], staticColWrapper.sbs[i], staticColWrapper.manifolds[i]);
                            }

                            for (int i = colWrapper.count; i < staticColWrapper.count; ++i) {
                                applyImpulse(staticColWrapper.rbs[i], staticColWrapper.sbs[i], staticColWrapper.manifolds[i]);
                            }
                        }
                    }

                    clearCollisions();

                    // Update our rigidbodies
                    for (int i = 0; i < rbs.count; ++i) { rbs.rigidBodies[i]->update(g, updateStep); }

                    dt -= updateStep;
                    ++count;
                }

                return count;
            };
    };
}
