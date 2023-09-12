#include <ZETA/physicshandler.h>

namespace Zeta {
    // * =========================
    // * Impulse Resolution
    // * =========================

    // Resolve a collision between two rigidbodies.
    void applyImpulse(RigidBody2D* rb1, RigidBody2D* rb2, CollisionManifold const &manifold) {
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

    // todo technically could pass just purely the rigid and kinematic bodies for their impulse resolution vs static bodies
    // todo test to ensure these are sufficient for impulse resolution ------------------------------------------------------------------
    // Resolve a collision between a rigid and static body.
    void applyImpulse(RigidBody2D* rb, StaticBody2D* sb, CollisionManifold const &manifold) {
        float J = ((ZMath::abs(rb->vel) * -(1 + rb->cor)) * manifold.normal) * rb->mass;
        rb->vel += manifold.normal * (rb->invMass * J);
    };

    // Resolve a collision between a rigid and kinematic body.
    void applyImpulse(RigidBody2D* rb, KinematicBody2D* kb, CollisionManifold const &manifold) {
        float J = ((ZMath::abs(rb->vel) * -(1 + rb->cor)) * manifold.normal) * rb->mass;
        rb->vel += manifold.normal * (rb->invMass * J);
        kb->pos -= manifold.normal * manifold.pDist;
    };

    // Resolve a collision between a static and kinematic body.
    void applyImpulse(KinematicBody2D* kb, StaticBody2D* sb, CollisionManifold const &manifold) {
        kb->pos += manifold.normal * (2 * manifold.pDist);
    };

    // Resolve a collision between two kinematic bodies.
    void applyImpulse(KinematicBody2D* kb1, KinematicBody2D* kb2, CollisionManifold const &manifold) {
        kb1->pos -= manifold.normal * manifold.pDist;
        kb2->pos += manifold.normal * manifold.pDist;
    };

    // todo -----------------------------------------------------------------------------------------------------------------------------


    // * ========================
    // * Main Physics Handler
    // * ========================

    // * ==============================
    // * Functions for Ease of Use
    // * ==============================

    void Handler::addCollision(RigidBody2D* rb1, RigidBody2D* rb2, CollisionManifold const &manifold) {
        if (colWrapper.count == colWrapper.capacity) {
            colWrapper.capacity *= 2;

            RigidBody2D** temp1 = new RigidBody2D*[colWrapper.capacity];
            RigidBody2D** temp2 = new RigidBody2D*[colWrapper.capacity];
            CollisionManifold* temp3 = new CollisionManifold[colWrapper.capacity];

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

    void Handler::addCollision(RigidBody2D* rb, StaticBody2D* sb, CollisionManifold const &manifold) {
        if (staticColWrapper.count == staticColWrapper.capacity) {
            staticColWrapper.capacity *= 2;

            StaticBody2D** temp1 = new StaticBody2D*[staticColWrapper.capacity];
            RigidBody2D** temp2 = new RigidBody2D*[staticColWrapper.capacity];
            CollisionManifold* temp3 = new CollisionManifold[staticColWrapper.capacity];

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

    void Handler::addCollision(RigidBody2D* rb, KinematicBody2D* kb, CollisionManifold const &manifold) {
        if (rkColWrapper.count == rkColWrapper.capacity) {
            rkColWrapper.capacity *= 2;

            RigidBody2D** temp1 = new RigidBody2D*[rkColWrapper.capacity];
            KinematicBody2D** temp2 = new KinematicBody2D*[rkColWrapper.capacity];
            CollisionManifold* temp3 = new CollisionManifold[rkColWrapper.capacity];

            for (int i = 0; i < rkColWrapper.count; ++i) {
                temp1[i] = rkColWrapper.rbs[i];
                temp2[i] = rkColWrapper.kbs[i];
                temp3[i] = rkColWrapper.manifolds[i];
            }

            delete[] rkColWrapper.rbs;
            delete[] rkColWrapper.kbs;
            delete[] rkColWrapper.manifolds;

            rkColWrapper.rbs = temp1;
            rkColWrapper.kbs = temp2;
            rkColWrapper.manifolds = temp3;
        }

        rkColWrapper.rbs[rkColWrapper.count] = rb;
        rkColWrapper.kbs[rkColWrapper.count] = kb;
        rkColWrapper.manifolds[rkColWrapper.count++] = manifold;
    };

    void Handler::addCollision(StaticBody2D* sb, KinematicBody2D* kb, CollisionManifold const &manifold) {
        if (skColWrapper.count == skColWrapper.capacity) {
            skColWrapper.capacity *= 2;

            StaticBody2D** temp1 = new StaticBody2D*[skColWrapper.capacity];
            KinematicBody2D** temp2 = new KinematicBody2D*[skColWrapper.capacity];
            CollisionManifold* temp3 = new CollisionManifold[skColWrapper.capacity];

            for (int i = 0; i < skColWrapper.count; ++i) {
                temp1[i] = skColWrapper.sbs[i];
                temp2[i] = skColWrapper.kbs[i];
                temp3[i] = skColWrapper.manifolds[i];
            }

            delete[] skColWrapper.sbs;
            delete[] skColWrapper.kbs;
            delete[] skColWrapper.manifolds;

            skColWrapper.sbs = temp1;
            skColWrapper.kbs = temp2;
            skColWrapper.manifolds = temp3;
        }

        skColWrapper.sbs[skColWrapper.count] = sb;
        skColWrapper.kbs[skColWrapper.count] = kb;
        skColWrapper.manifolds[skColWrapper.count++] = manifold;
    };

    void Handler::addCollision(KinematicBody2D* kb1, KinematicBody2D* kb2, CollisionManifold const &manifold) {
        if (kColWrapper.count == kColWrapper.capacity) {
            kColWrapper.capacity *= 2;

            KinematicBody2D** temp1 = new KinematicBody2D*[kColWrapper.capacity];
            KinematicBody2D** temp2 = new KinematicBody2D*[kColWrapper.capacity];
            CollisionManifold* temp3 = new CollisionManifold[kColWrapper.capacity];

            for (int i = 0; i < kColWrapper.count; ++i) {
                temp1[i] = kColWrapper.kb1s[i];
                temp2[i] = kColWrapper.kb2s[i];
                temp3[i] = kColWrapper.manifolds[i];
            }

            delete[] kColWrapper.kb1s;
            delete[] kColWrapper.kb2s;
            delete[] kColWrapper.manifolds;

            kColWrapper.kb1s = temp1;
            kColWrapper.kb2s = temp2;
            kColWrapper.manifolds = temp3;
        }

        kColWrapper.kb1s[kColWrapper.count] = kb1;
        kColWrapper.kb2s[kColWrapper.count] = kb2;
        kColWrapper.manifolds[kColWrapper.count++] = manifold;
    };

    void Handler::clearCollisions() {
        // ? We do not need to check for nullptrs because if this function is reached it is guarenteed none of the pointers inside of here will be NULL

        int halfRbs = rbs.capacity/2;
        int halfKbs = kbs.capacity/2;


        // * standard collision wrapper
        delete[] colWrapper.bodies1;
        delete[] colWrapper.bodies2;

        for (int i = 0; i < colWrapper.count; ++i) { delete[] colWrapper.manifolds[i].contactPoints; }
        delete[] colWrapper.manifolds;

        colWrapper.bodies1 = new RigidBody2D*[halfRbs];
        colWrapper.bodies2 = new RigidBody2D*[halfRbs];
        colWrapper.manifolds = new CollisionManifold[halfRbs];

        colWrapper.capacity = halfRbs;
        colWrapper.count = 0;


        // * Static collision wrapper
        delete[] staticColWrapper.sbs;
        delete[] staticColWrapper.rbs;

        for (int i = 0; i < staticColWrapper.count; ++i) { delete[] staticColWrapper.manifolds[i].contactPoints; }
        delete[] staticColWrapper.manifolds;

        staticColWrapper.sbs = new StaticBody2D*[halfRbs];
        staticColWrapper.rbs = new RigidBody2D*[halfRbs];
        staticColWrapper.manifolds = new CollisionManifold[halfRbs];

        staticColWrapper.capacity = halfRbs;
        staticColWrapper.count = 0;


        // * Kinematic body collision wrappers
        delete[] rkColWrapper.rbs;
        delete[] rkColWrapper.kbs;
        
        for (int i = 0; i < rkColWrapper.count; ++i) { delete[] rkColWrapper.manifolds[i].contactPoints; }
        delete[] rkColWrapper.manifolds;

        rkColWrapper.rbs = new RigidBody2D*[halfKbs];
        rkColWrapper.kbs = new KinematicBody2D*[halfKbs];
        rkColWrapper.manifolds = new CollisionManifold[halfKbs];

        rkColWrapper.capacity = halfKbs;
        rkColWrapper.count = 0;


        delete[] skColWrapper.sbs;
        delete[] skColWrapper.kbs;
        
        for (int i = 0; i < skColWrapper.count; ++i) { delete[] skColWrapper.manifolds[i].contactPoints; }
        delete[] skColWrapper.manifolds;

        skColWrapper.sbs = new StaticBody2D*[halfKbs];
        skColWrapper.kbs = new KinematicBody2D*[halfKbs];
        skColWrapper.manifolds = new CollisionManifold[halfKbs];

        skColWrapper.capacity = halfKbs;
        skColWrapper.count = 0;


        delete[] kColWrapper.kb1s;
        delete[] kColWrapper.kb2s;
        
        for (int i = 0; i < kColWrapper.count; ++i) { delete[] kColWrapper.manifolds[i].contactPoints; }
        delete[] kColWrapper.manifolds;

        kColWrapper.kb1s = new KinematicBody2D*[halfKbs];
        kColWrapper.kb2s = new KinematicBody2D*[halfKbs];
        kColWrapper.manifolds = new CollisionManifold[halfKbs];

        kColWrapper.capacity = halfKbs;
        kColWrapper.count = 0;
    };

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
    Handler::Handler(ZMath::Vec2D const &g, float timeStep) : g(g), updateStep(timeStep) {
        if (updateStep < FPS_60) { updateStep = FPS_60; } // hard cap at 60 FPS

        // * Bodies
        rbs.rigidBodies = new RigidBody2D*[startingSlots];
        rbs.capacity = startingSlots;
        rbs.count = 0;

        sbs.staticBodies = new StaticBody2D*[startingSlots];
        sbs.capacity = startingSlots;
        sbs.count = 0;

        kbs.kinematicBodies = new KinematicBody2D*[kStartingSlots];
        kbs.capacity = kStartingSlots;
        kbs.count = 0;


        // * Collisions
        colWrapper.bodies1 = new RigidBody2D*[halfStartingSlots];
        colWrapper.bodies2 = new RigidBody2D*[halfStartingSlots];
        colWrapper.manifolds = new CollisionManifold[halfStartingSlots];
        colWrapper.capacity = halfStartingSlots;
        colWrapper.count = 0;

        staticColWrapper.sbs = new StaticBody2D*[halfStartingSlots];
        staticColWrapper.rbs = new RigidBody2D*[halfStartingSlots];
        staticColWrapper.manifolds = new CollisionManifold[halfStartingSlots];
        staticColWrapper.capacity = halfStartingSlots;
        staticColWrapper.count = 0;

        rkColWrapper.rbs = new RigidBody2D*[kHalfStartingSlots];
        rkColWrapper.kbs = new KinematicBody2D*[kHalfStartingSlots];
        rkColWrapper.manifolds = new CollisionManifold[kHalfStartingSlots];
        rkColWrapper.capacity = kHalfStartingSlots;
        rkColWrapper.count = 0;

        skColWrapper.sbs = new StaticBody2D*[kHalfStartingSlots];
        skColWrapper.kbs = new KinematicBody2D*[kHalfStartingSlots];
        skColWrapper.manifolds = new CollisionManifold[kHalfStartingSlots];
        skColWrapper.capacity = kHalfStartingSlots;
        skColWrapper.count = 0;

        kColWrapper.kb1s = new KinematicBody2D*[kHalfStartingSlots];
        kColWrapper.kb2s = new KinematicBody2D*[kHalfStartingSlots];
        kColWrapper.manifolds = new CollisionManifold[kHalfStartingSlots];
        kColWrapper.capacity = kHalfStartingSlots;
        kColWrapper.count = 0;
    };

    // Do not allow for construction from an existing physics handler.
    Handler::Handler(Handler const &handler) { throw std::runtime_error("PhysicsHandler object CANNOT be constructed from another PhysicsHandler."); };
    Handler::Handler(Handler&& handler) { throw std::runtime_error("PhysicsHandler object CANNOT be constructed from another PhysicsHandler."); };

    // The physics handler cannot be reassigned.
    Handler& Handler::operator = (Handler const &handler) { throw std::runtime_error("PhysicsHandler object CANNOT be reassigned to another PhysicsHandler."); };
    Handler& Handler::operator = (Handler&& handler) { throw std::runtime_error("PhysicsHandler object CANNOT be reassigned to another PhysicsHandler."); };

    Handler::~Handler() {
        // If one of the pointers is not NULL, none of them are.
        if (rbs.rigidBodies) {
            // * Bodies

            for (int i = 0; i < rbs.count; ++i) { delete rbs.rigidBodies[i]; }
            delete[] rbs.rigidBodies;

            for (int i = 0; i < sbs.count; ++i) { delete sbs.staticBodies[i]; }
            delete[] sbs.staticBodies;

            for (int i = 0; i < kbs.count; ++i) { delete kbs.kinematicBodies[i]; }
            delete[] kbs.kinematicBodies;


            // * Collisions

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

            // * Same stuff but for kinematic body collisions
            delete[] rkColWrapper.kbs;
            delete[] rkColWrapper.rbs;

            for (int i = 0; i < rkColWrapper.count; ++i) { delete[] rkColWrapper.manifolds[i].contactPoints; }
            delete[] rkColWrapper.manifolds;

            delete[] skColWrapper.kbs;
            delete[] skColWrapper.sbs;

            for (int i = 0; i < skColWrapper.count; ++i) { delete[] skColWrapper.manifolds[i].contactPoints; }
            delete[] skColWrapper.manifolds;

            delete[] kColWrapper.kb1s;
            delete[] kColWrapper.kb2s;

            for (int i = 0; i < kColWrapper.count; ++i) { delete[] kColWrapper.manifolds[i].contactPoints; }
            delete[] kColWrapper.manifolds;
        }
    };


    // * ============================
    // * RigidBody List Functions
    // * ============================

    // Add a rigid body to the list of rigid bodies to be updated.
    void Handler::addRigidBody(RigidBody2D* rb) {
        if (rbs.count == rbs.capacity) {
            rbs.capacity *= 2;
            RigidBody2D** temp = new RigidBody2D*[rbs.capacity];

            for (int i = 0; i < rbs.count; ++i) { temp[i] = rbs.rigidBodies[i]; }

            delete[] rbs.rigidBodies;
            rbs.rigidBodies = temp;
        }

        rbs.rigidBodies[rbs.count++] = rb;
    };

    // Add a list of rigid bodies to the handler.
    void Handler::addRigidBodies(RigidBody2D** rbs, int size) {
        if (this->rbs.count + size > this->rbs.capacity) {
            do { this->rbs.capacity *= 2; } while (this->rbs.count + size > this->rbs.capacity);
            RigidBody2D** temp = new RigidBody2D*[this->rbs.capacity];
        
            for (int i = 0; i < this->rbs.count; ++i) { temp[i] = this->rbs.rigidBodies[i]; }

            delete[] this->rbs.rigidBodies;
            this->rbs.rigidBodies = temp;
        }


        for (int i = 0; i < size; ++i) { this->rbs.rigidBodies[this->rbs.count++] = rbs[i]; }
    };

    // Remove a rigid body from the handler.
    // 1 = rigid body was found and removed. 0 = It was not found.
    // rb will be deleted if the rigid body was found.
    bool Handler::removeRigidBody(RigidBody2D* rb) {
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
    void Handler::addStaticBody(StaticBody2D* sb) {
        if (sbs.count == sbs.capacity) {
            sbs.capacity *= 2;
            StaticBody2D** temp = new StaticBody2D*[sbs.capacity];

            for (int i = 0; i < sbs.count; ++i) { temp[i] = sbs.staticBodies[i]; }

            delete[] sbs.staticBodies;
            sbs.staticBodies = temp;
        }

        sbs.staticBodies[sbs.count++] = sb;
    };

    // Add a list of static bodies to the handler.
    void Handler::addStaticBodies(StaticBody2D** sbs, int size) {
        if (this->sbs.count + size > this->sbs.capacity) {
            do { this->sbs.capacity *= 2; } while(this->sbs.count + size > this->sbs.capacity);
            StaticBody2D** temp = new StaticBody2D*[this->sbs.capacity];

            for (int i = 0; i < this->sbs.count; ++i) { temp[i] = this->sbs.staticBodies[i]; }

            delete[] this->sbs.staticBodies;
            this->sbs.staticBodies = temp;
        }

        for (int i = 0; i < size; ++i) { this->sbs.staticBodies[this->sbs.count++] = sbs[i]; }
    };

    // Remove a static body from the handler.
    // 1 = static body was found and removed. 0 = It was not found.
    // sb will be deleted if the static body was found.
    bool Handler::removeStaticBody(StaticBody2D* sb) {
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


    // * ================================
    // * KinematicBody List Functions
    // * ================================

    // Add a kinematic body to the handler.
    void Handler::addKinematicBody(KinematicBody2D* kb) {
        if (kbs.count == kbs.capacity) {
            kbs.capacity *= 2;
            KinematicBody2D** temp = new KinematicBody2D*[kbs.capacity];

            for (int i = 0; i < kbs.count; ++i) { temp[i] = kbs.kinematicBodies[i]; }

            delete[] kbs.kinematicBodies;
            kbs.kinematicBodies = temp;
        }

        kbs.kinematicBodies[sbs.count++] = kb;
    };

    // Add a list of kinematic bodies to the handler.
    void Handler::addKinematicBodies(KinematicBody2D** kbs, int size) {
        if (this->kbs.count + size > this->kbs.capacity) {
            do { this->kbs.capacity *= 2; } while(this->kbs.count + size > this->kbs.capacity);
            KinematicBody2D** temp = new KinematicBody2D*[this->kbs.capacity];

            for (int i = 0; i < this->kbs.count; ++i) { temp[i] = this->kbs.kinematicBodies[i]; }

            delete[] this->kbs.kinematicBodies;
            this->kbs.kinematicBodies = temp;
        }

        for (int i = 0; i < size; ++i) { this->kbs.kinematicBodies[this->kbs.count++] = kbs[i]; }
    };

    // Remove a kinematic body from the handler.
    // 1 = kinematic body was found and removed. 0 = It was not found.
    // kb will be deleted if the kinematic body was found.
    bool Handler::removeKinematicBody(KinematicBody2D* kb) {
        for (int i = kbs.count - 1; i >= 0; --i) {
            if (kbs.kinematicBodies[i] == kb) {
                delete kb;
                for (int j = i; j < kbs.count - 1; ++j) { kbs.kinematicBodies[j] = kbs.kinematicBodies[j + 1]; }
                kbs.count--;
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
    int Handler::update(float &dt) {
        int count = 0;

        while (dt >= updateStep) {
            // Broad phase: collision detection
            // There will, on average, be too few kinematic bodies for it to be worth combining the loops
            for (int i = 0; i < rbs.count - 1; ++i) {
                for (int j = i + 1; j < rbs.count; ++j) {
                    CollisionManifold result = findCollisionFeatures(rbs.rigidBodies[i], rbs.rigidBodies[j]);
                    if (result.hit) { addCollision(rbs.rigidBodies[i], rbs.rigidBodies[j], result); }
                }

                for (int j = 0; j < sbs.count; ++j) {
                    CollisionManifold result = findCollisionFeatures(rbs.rigidBodies[i], sbs.staticBodies[j]);
                    if (result.hit) { addCollision(rbs.rigidBodies[i], sbs.staticBodies[j], result); }
                }

                for (int j = 0; j < kbs.count; ++j) {
                    CollisionManifold result = findCollisionFeatures(rbs.rigidBodies[i], kbs.kinematicBodies[j]);
                    if (result.hit) { addCollision(rbs.rigidBodies[i], kbs.kinematicBodies[j], result); }
                }
            }

            // check for kinematic body collisions
            for (int i = 0; i < kbs.count; ++i) {
                for (int j = i + 1; j < kbs.count; ++j) {
                    CollisionManifold result = findCollisionFeatures(kbs.kinematicBodies[i], kbs.kinematicBodies[j]);
                    if (result.hit) { addCollision(kbs.kinematicBodies[i], kbs.kinematicBodies[j], result); }
                }

                for (int j = 0; j < sbs.count; ++j) {
                    CollisionManifold result = findCollisionFeatures(kbs.kinematicBodies[i], sbs.staticBodies[j]);
                    if (result.hit) { addCollision(sbs.staticBodies[j], kbs.kinematicBodies[i], result); }
                }
            }

            // todo update to not be through iterative deepening -- look into this in the future
            // todo use spatial partitioning
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

                // resolve kinematic body collisions
                // There will, on average, be too few kinematic bodies for it to be worth combining the loops
                for (int i = 0; i < rkColWrapper.count; ++i) {
                    applyImpulse(rkColWrapper.rbs[i], rkColWrapper.kbs[i], rkColWrapper.manifolds[i]);
                }

                for (int i = 0; i < skColWrapper.count; ++i) {
                    applyImpulse(skColWrapper.kbs[i], skColWrapper.sbs[i], skColWrapper.manifolds[i]);
                }

                for (int i = 0; i < kColWrapper.count; ++i) {
                    applyImpulse(kColWrapper.kb1s[i], kColWrapper.kb2s[i], kColWrapper.manifolds[i]);
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
}
