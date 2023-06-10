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

    namespace {
        // * ===================================
        // * Collision Manifold Calculators
        // * ===================================

        CollisionManifold findCollisionFeatures(Primitives::Circle const &circle1, Primitives::Circle const &circle2) {
            CollisionManifold result;

            float r = circle1.r + circle2.r;
            ZMath::Vec2D sphereDiff = circle2.c - circle1.c;

            result.hit = sphereDiff.magSq() <= r*r;

            if (!result.hit) { return result; }
            
            float d = sphereDiff.mag(); // allows us to only take the sqrt once

            result.pDist = (circle1.r + circle2.r - d) * 0.5f;
            result.normal = sphereDiff * (1.0f/d);

            // determine the contact point
            result.numPoints = 1;
            result.contactPoints = new ZMath::Vec2D[result.numPoints];
            result.contactPoints[0] = circle1.c + (result.normal * (circle1.r - result.pDist));

            return result;
        };

        CollisionManifold findCollisionFeatures(Primitives::Circle const &circle, Primitives::AABB const &aabb) {
            CollisionManifold result;

            // ? We know a sphere and AABB would intersect if the distance from the closest point to the center on the AABB
            // ?  from the center is less than or equal to the radius of the sphere.
            // ? We can determine the closet point by clamping the value of the sphere's center between the min and max of the AABB.
            // ? From here, we can check the distance from this point to the sphere's center.

            ZMath::Vec2D closest = circle.c;
            ZMath::Vec2D min = aabb.getMin(), max = aabb.getMax();

            closest = ZMath::clamp(closest, min, max);
            result.hit = closest.distSq(circle.c) <= circle.r*circle.r;

            if (!result.hit) { return result; }

            // The closest point to the sphere's center will be our contact point.
            // Therefore, we just set our contact point to closest.

            result.numPoints = 1;
            result.contactPoints = new ZMath::Vec2D[1];
            result.contactPoints[0] = closest;

            // determine the penetration distance and collision normal

            ZMath::Vec2D diff = closest - circle.c;
            float d = diff.mag(); // allows us to only take the sqrt once
            result.pDist = circle.r - d;
            result.normal = diff * (1.0f/d);

            return result;
        };

        CollisionManifold findCollisionFeatures(Primitives::Circle const &circle, Primitives::Box2D const &box) {
            CollisionManifold result;

            ZMath::Vec2D closest = circle.c - box.pos;
            ZMath::Vec2D min = box.getLocalMin(), max = box.getLocalMax();

            // rotate the center of the sphere into the UVW coordinates of our cube
            closest = box.rot * closest + box.pos;
            
            // perform the check as if it was an AABB vs Sphere
            closest = ZMath::clamp(closest, min, max);
            result.hit = closest.distSq(circle.c) <= circle.r*circle.r;

            if (!result.hit) { return result; }

            // the closest point to the sphere's center will be our contact point rotated back into global coordinates coordinates

            closest -= box.pos;
            closest = box.rot.transpose() * closest + box.pos;

            result.numPoints = 1;
            result.contactPoints = new ZMath::Vec2D[1];
            result.contactPoints[0] = closest;

            // determine the penetration distance and the collision normal

            ZMath::Vec2D diff = closest - circle.c;
            float d = diff.mag(); // allows us to only take the sqrt once
            result.pDist = circle.r - d;
            result.normal = diff * (1.0f/d);

            return result;
        };

        // * ====================================================
        // * Helper Functions for 3D Box Collision Manifolds
        // * ====================================================

        // * Enums used for denotating the edges of the cubes
        enum Axis {
            FACE_A_X,
            FACE_A_Y,
            FACE_B_X,
            FACE_B_Y
        };

        /**
         * @brief Determine the 2 vertices comprising the incident face when the incident cube is an AABB.
         * 
         * @param v Array which gets filled with the 2 vertices making up the incident face.
         * @param h Halfsize of the incident AABB.
         * @param pos The position of the incident AABB.
         * @param normal The normal vector of the collision (points towards B away from A).
         */
        static void computeIncidentFaceAABB(ZMath::Vec2D v[2], const ZMath::Vec2D& h, const ZMath::Vec2D& pos, const ZMath::Vec2D& normal) {
            // Take the absolute value of the normal for comparisons.
            ZMath::Vec2D nAbs = ZMath::abs(normal);

            // Determine the vertices.
            // Vertex array starts in the bottom left corner when considering the face as a 2D box and goes around counterclockwise.
            if (nAbs.x > nAbs.y) { // x > y
                if (normal.x > 0.0f) { // incident cube is intersecting on its -x side
                    v[0] = pos - h;
                    v[1] = ZMath::Vec2D(pos.x - h.x, pos.y + h.y);

                } else { // incident cube is intersecting on its +x side
                    v[0] = ZMath::Vec2D(pos.x + h.x, pos.y - h.y);
                    v[1] = pos + h;
                }

            } else { // y >= x
                if (normal.y > 0.0f) { // incident cube is intersecting on its -y side
                    v[0] = pos - h;
                    v[1] = ZMath::Vec2D(pos.x + h.x, pos.y - h.y);

                } else { // incident cube is intersecting on its +y side
                    v[0] = ZMath::Vec2D(pos.x - h.x, pos.y + h.y);
                    v[1] = pos + h;
                }
            }
        };

        /**
         * @brief Determine the 2 vertices making up the incident face.
         * 
         * @param v Array which gets filled with the 2 vertices comprising the incident face.
         * @param h Halfsize of the incident cube.
         * @param pos The position of the incident cube.
         * @param rot The rotation matrix of the incident cube.
         * @param normal The normal vector of the collision.
         */
        static void computeIncidentFace(ZMath::Vec2D v[2], const ZMath::Vec2D& h, const ZMath::Vec2D& pos, 
                                        const ZMath::Mat2D& rot, const ZMath::Vec2D& normal) {

            // Rotate the normal to the incident cube's local space.
            ZMath::Vec2D n = rot.transpose() * normal;
            ZMath::Vec2D nAbs = ZMath::abs(n);

            // Determine the vertices in terms of halfsize.
            // Vertex array starts in bottom left corner when considering the face as a 2D box and goes around counterclockwise.
            if (nAbs.x > nAbs.y) { // x > y
                if (n.x > 0.0f) { // incident cube is intersecting on its -x side
                    v[0] = ZMath::Vec2D(-h.x, -h.y);
                    v[1] = ZMath::Vec2D(-h.x, h.y);

                } else { // incident cube is intersecting on its +x side
                    v[0] = ZMath::Vec2D(h.x, -h.y);
                    v[1] = ZMath::Vec2D(h.x, h.y);
                }

            } else { // y >= x
                if (n.y > 0.0f) { // incident cube is intersecting on its -y side
                    v[0] = ZMath::Vec2D(-h.x, -h.y);
                    v[1] = ZMath::Vec2D(h.x, -h.y);

                } else { // incident cube is intersecting on its +y side
                    v[0] = ZMath::Vec2D(-h.x, h.y);
                    v[1] = ZMath::Vec2D(h.x, h.y);
                }
            }

            // rotate vertices back into global coordinates and translate them to their proper positions
            v[0] = pos + rot * v[0];
            v[1] = pos + rot * v[1];
        };

        /**
         * @brief Compute the clipping points given input points.
         * 
         * @param vOut Array which gets filled with the clipping points.
         * @param vIn Array containing the input points.
         * @param n1 Side normal 1.
         * @param n2 Side normal 2.
         * @param offset1 Distance to the side corresponding with side normal 1.
         * @param offset2 Distance to the side corresponding with side normal 2.
         * @return (int) The number of clipping points. If this does not return 2, there is not an intersection on this axis.
         */
        int clipSegmentToLine(ZMath::Vec3D vOut[4], ZMath::Vec3D vIn[4], const ZMath::Vec3D &n1, const ZMath::Vec3D &n2, float offset1, float offset2) {
            // begin with 0 output points
            int np = 0;

            // calculate the distance
            // first set of distances
            float d0 = n1 * vIn[0] - offset1;
            float d1 = n1 * vIn[1] - offset1;

            // second set of distances
            float d2 = n2 * vIn[0] - offset2;
            float d3 = n2 * vIn[3] - offset2;

            // * Compute the clipping points.
            // ? If the points are outside the reference cube's clipping plane (more or less inside the cube), add them as clipping points.
            // ? Otherwise, check if the vertices are separated by the edge of the reference cube used for this clipping plane.
            
            // first input point
            if (d0 <= 0.0f && d2 <= 0.0f) { vOut[np++] = vIn[0]; }
            else if (d0 * d1 < 0.0f && d2 * d3 < 0.0f) { vOut[np++] = vIn[0] + (vIn[1] - vIn[0]) * (d0/(d1 + d0)) + (vIn[3] - vIn[0]) * (d2/(d3 + d2)); }
            else if (d0 * d1 < 0.0f) { vOut[np++] = vIn[0] + (vIn[1] - vIn[0]) * (d0/(d1 + d0)); } 
            else if (d2 * d3 < 0.0f) { vOut[np++] = vIn[0] + (vIn[3] - vIn[0]) * (d2/(d3 + d2)); }

            // second input point
            if (d1 <= 0.0f && d2 <= 0.0f) { vOut[np++] = vIn[1]; }
            else if (d0 * d1 < 0.0f && d2 * d3 < 0.0f) { vOut[np++] = vIn[1] + (vIn[0] - vIn[1]) * (d1/(d1 + d0)) + (vIn[2] - vIn[1]) * (d2/(d2 + d3)); }
            else if (d0 * d1 < 0.0f) { vOut[np++] = vIn[1] + (vIn[0] - vIn[1]) * (d1/(d1 + d0)); }
            else if (d2 * d3 < 0.0f) { vOut[np++] = vIn[1] + (vIn[2] - vIn[1]) * (d2/(d3 + d2)); }

            // third input point
            if (d1 <= 0.0f && d3 <= 0.0f) { vOut[np++] = vIn[2]; }
            else if (d0 * d1 < 0.0f && d2 * d3 < 0.0f) { vOut[np++] = vIn[2] + (vIn[3] - vIn[2]) * (d1/(d1 + d0)) + (vIn[1] - vIn[2]) * (d3/(d2 + d3)); }
            else if (d0 * d1 < 0.0f) { vOut[np++] = vIn[2] + (vIn[3] - vIn[2]) * (d1/(d1 + d0)); }
            else if (d2 * d3 < 0.0f) { vOut[np++] = vIn[2] + (vIn[1] - vIn[2]) * (d3/(d3 + d2)); }

            // fourth input point
            if (d0 <= 0.0f && d3 <= 0.0f) { vOut[np++] = vIn[3]; }
            else if (d0 * d1 < 0.0f && d2 * d3 < 0.0f) { vOut[np++] = vIn[3] + (vIn[2] - vIn[3]) * (d0/(d1 + d0)) + (vIn[0] - vIn[3]) * (d3/(d3 + d2)); }
            else if (d0 * d1 < 0.0f) { vOut[np++] = vIn[3] + (vIn[2] - vIn[3]) * (d0/(d1 + d0)); }
            else if (d2 * d3 < 0.0f) { vOut[np++] = vIn[3] + (vIn[0] - vIn[3]) * (d3/(d3 + d2)); }

            return np;
        };

        // ? Normal points towards B and away from A

        CollisionManifold findCollisionFeatures(Primitives::AABB const &aabb1, Primitives::AABB const &aabb2) {
            CollisionManifold result;

            // half size of AABB a and b respectively
            ZMath::Vec2D hA = aabb1.getHalfsize(), hB = aabb2.getHalfsize();

            // * Check for intersections using the separating axis theorem.
            // because both are axis aligned, global space is the same as the local space of both AABBs.

            // distance between the two
            ZMath::Vec2D dP = aabb2.pos - aabb1.pos;
            ZMath::Vec2D absDP = ZMath::abs(dP);

            // penetration along A's (and B's) axes
            ZMath::Vec2D faceA = absDP - hA - hB;
            if (faceA.x > 0 || faceA.y > 0) {
                result.hit = 0;
                return result;
            }

            // ? Since they are axis aligned, the penetration between the two will be the same on any given axis.
            // ?  Therefore, we only need to check for A.

            // * Find the best axis (i.e. the axis with the least amount of penetration).

            // Assume A's x-axis is the best axis first
            Axis axis = FACE_A_X;
            float separation = faceA.x;
            result.normal = dP.x > 0.0f ? ZMath::Vec2D(1, 0) : ZMath::Vec2D(-1, 0);

            // tolerance values
            float relativeTol = 0.95f;
            float absoluteTol = 0.01f;

            // ? check if there is another axis better than A's x axis by checking if the penetration along
            // ?  the current axis being checked is greater than that of the current penetration
            // ?  (as greater value = less negative = less penetration).

            // A's remaining axes
            if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
                axis = FACE_A_Y;
                separation = faceA.y;
                result.normal = dP.y > 0.0f ? ZMath::Vec2D(0, 1) : ZMath::Vec2D(0, -1);
            }

            // * Setup clipping plane data based on the best axis

            ZMath::Vec2D sideNormal1, sideNormal2;
            ZMath::Vec2D incidentFace[2]; // 4 vertices for the collision in 3D
            float front, negSide1, negSide2, posSide1, posSide2;

            // * Compute the clipping lines and line segment to be clipped

            switch(axis) {
                case FACE_A_X: {
                    front = aabb1.pos * result.normal + hA.x;

                    negSide1 = aabb1.pos.y - hA.y; // negSideY
                    posSide1 = aabb1.pos.y + hA.y; // posSideY

                    computeIncidentFaceAABB(incidentFace, hB, aabb2.pos, result.normal);
                    break;
                }

                case FACE_A_Y: {
                    front = aabb1.pos * result.normal + hA.y;

                    negSide1 = aabb1.pos.x - hA.x; // negSideX
                    posSide1 = aabb1.pos.x + hA.x; // posSideX

                    computeIncidentFaceAABB(incidentFace, hB, aabb2.pos, result.normal);
                    break;
                }
            }

            // * Clip the incident edge with box planes.

            ZMath::Vec2D clipPoints1[2];
            ZMath::Vec2D clipPoints2[2];

            // Clip to side 1
            int np = clipSegmentToLine(clipPoints1, incidentFace, -sideNormal1, -sideNormal2, negSide1, negSide2);

            if (np < 2) {
                result.hit = 0;
                return result;
            }

            // Clip to the negative side 1
            np = clipSegmentToLine(clipPoints2, clipPoints1, sideNormal1, sideNormal2, posSide1, posSide2);

            if (np < 2) {
                result.hit = 0;
                return result;
            }

            // * ClipPoints2 now contains the clipping points.
            // * Compute the contact points.
            
            // store the conatct points in here and add them to the dynamic array after they are determined
            ZMath::Vec2D contactPoints[2];
            np = 0;
            result.pDist = 0.0f;

            for (int i = 0; i < 2; ++i) {
                separation = result.normal * clipPoints2[i] - front;

                if (separation <= 0) {
                    contactPoints[np++] = clipPoints2[i] - result.normal * separation;
                    if (result.pDist < separation) { result.pDist = separation; }
                }
            }

            // * update the manifold to contain the results.

            result.pDist = -result.pDist;
            result.hit = 1;
            result.numPoints = np;
            result.contactPoints = new ZMath::Vec2D[np];

            for (int i = 0; i < np; ++i) { result.contactPoints[i] = contactPoints[i]; }
            
            return result;
        };

        // ? Normal points towards B and away from A

        CollisionManifold findCollisionFeatures(Primitives::AABB const &aabb, Primitives::Box2D const &box) {
            CollisionManifold result;

            // half size of a and b respectively
            ZMath::Vec2D hA = aabb.getHalfsize(), hB = box.getHalfsize();

            // * determine the rotation matrices of A and B
            // * global space is A's local space so we do not need a rotation matrix for it

            // rotate anything from gobal space to B's local space
            ZMath::Mat2D rotBT = box.rot.transpose();

            // determine the difference between the positions
            ZMath::Vec2D dA = box.pos - aabb.pos;
            ZMath::Vec2D dB = rotBT * dA;

            // * Check for intersections with the separating axis theorem

            // amount of penetration along A's axes
            ZMath::Vec2D faceA = ZMath::abs(dA) - hA - hB;
            if (faceA.x > 0 || faceA.y > 0) {
                result.hit = 0;
                return result;
            }

            // amount of penetration along B's axes
            ZMath::Vec2D faceB = ZMath::abs(dB) - hB - rotBT * hA;
            if (faceB.x > 0 || faceB.y > 0) {
                result.hit = 0;
                return result;
            }

            // * Find the best axis (i.e. the axis with the least amount of penetration).

            // Assume A's x-axis is the best axis first
            Axis axis = FACE_A_X;
            float separation = faceA.x;
            result.normal = dA.x > 0.0f ? ZMath::Vec2D(1, 0) : ZMath::Vec2D(-1, 0);

            // tolerance values
            float relativeTol = 0.95f;
            float absoluteTol = 0.01f;

            // ? check if there is another axis better than A's x axis by checking if the penetration along
            // ?  the current axis being checked is greater than that of the current penetration
            // ?  (as greater value = less negative = less penetration).

            // A's remaining axes
            if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
                axis = FACE_A_Y;
                separation = faceA.y;
                result.normal = dA.y > 0.0f ? ZMath::Vec2D(0, 1) : ZMath::Vec2D(0, -1);
            }

            // B's axes
            if (faceB.x > relativeTol * separation + absoluteTol * hB.x) {
                axis = FACE_B_X;
                separation = faceB.x;
                result.normal = dB.x > 0.0f ? box.rot.c1 : -box.rot.c1;
            }

            if (faceB.y > relativeTol * separation + absoluteTol * hB.y) {
                axis = FACE_B_Y;
                separation = faceB.y;
                result.normal = dB.y > 0.0f ? box.rot.c2 : -box.rot.c2;
            }

            // * Setup clipping plane data based on the best axis

            ZMath::Vec2D sideNormal;
            ZMath::Vec2D incidentFace[2]; // 4 vertices for the collision in 3D
            float front, negSide, posSide;

            // * Compute the clipping lines and line segment to be clipped

            switch(axis) {
                case FACE_A_X: {
                    front = aabb.pos * result.normal + hA.x;

                    negSide = aabb.pos.y - hA.y; // negSideY
                    posSide = aabb.pos.y + hA.y; // posSideY

                    computeIncidentFace(incidentFace, hB, box.pos, box.rot, result.normal);
                    break;
                }

                case FACE_A_Y: {
                    front = aabb.pos * result.normal + hA.y;

                    negSide = aabb.pos.x - hA.x; // negSideX
                    posSide = aabb.pos.x + hA.x; // posSideX

                    computeIncidentFace(incidentFace, hB, box.pos, box.rot, result.normal);
                    break;
                }

                case FACE_B_X: {
                    front = box.pos * result.normal + hB.x;
                    sideNormal = box.rot.c2; // yNormal
                    float ySide = box.pos * sideNormal;

                    negSide = -ySide + hB.y; // negSideY
                    posSide = ySide + hB.y; // posSideY

                    computeIncidentFaceAABB(incidentFace, hA, aabb.pos, result.normal);
                    break;
                }

                case FACE_B_Y: {
                    front = box.pos * result.normal + hB.y;
                    sideNormal = box.rot.c1; // xNormal
                    float xSide = box.pos * sideNormal;

                    negSide = -xSide + hB.x; // negSideX
                    posSide = xSide + hB.x; // posSideX

                    computeIncidentFaceAABB(incidentFace, hA, aabb.pos, result.normal);
                    break;
                }
            }

            // * Clip the incident edge with box planes.

            ZMath::Vec2D clipPoints1[2];
            ZMath::Vec2D clipPoints2[2];

            // Clip to side 1
            int np = clipSegmentToLine(clipPoints1, incidentFace, -sideNormal, negSide);

            if (np < 2) {
                result.hit = 0;
                return result;
            }

            // Clip to the negative side 1
            np = clipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide);

            if (np < 2) {
                result.hit = 0;
                return result;
            }

            // * ClipPoints2 now contains the clipping points.
            // * Compute the contact points.
            
            // store the conatct points in here and add them to the dynamic array after they are determined
            ZMath::Vec2D contactPoints[2];
            np = 0;
            result.pDist = 0.0f;

            for (int i = 0; i < 2; ++i) {
                separation = result.normal * clipPoints2[i] - front;

                if (separation <= 0) {
                    contactPoints[np++] = clipPoints2[i] - result.normal * separation;
                    if (result.pDist < separation) { result.pDist = separation; }
                }
            }

            // * update the manifold to contain the results.

            result.pDist = -result.pDist;
            result.hit = 1;
            result.numPoints = np;
            result.contactPoints = new ZMath::Vec2D[np];

            for (int i = 0; i < np; ++i) { result.contactPoints[i] = contactPoints[i]; }
            
            return result;
        };

        // ? Normal points towards B and away from A

        CollisionManifold findCollisionFeatures(Primitives::Box2D const &box1, Primitives::Box2D const &box2) {
            CollisionManifold result;

            // half size of cube a and b respectively
            ZMath::Vec2D hA = box1.getHalfsize(), hB = box2.getHalfsize();

            // * determine the rotation matrices of A and B

            // rotate anything from global space to A's local space
            ZMath::Mat2D rotAT = box1.rot.transpose();

            // rotate anything from gobal space to B's local space
            ZMath::Mat2D rotBT = box2.rot.transpose();

            // determine the difference between the positions
            ZMath::Vec2D dP = box2.pos - box1.pos;
            ZMath::Vec2D dA = rotAT * dP;
            ZMath::Vec2D dB = rotBT * dP;

            // * rotation matrices for switching between local spaces

            // ! When scenes are developed test if we actually need the absolute value

            // Rotate anything from B's local space into A's
            ZMath::Mat2D C = ZMath::abs(rotAT * box2.rot);

            // Rotate anything from A's local space into B's
            ZMath::Mat2D CT = C.transpose();

            // * Check for intersections with the separating axis theorem

            // amount of penetration along A's axes
            ZMath::Vec2D faceA = ZMath::abs(dA) - hA - C * hB;
            if (faceA.x > 0 || faceA.y > 0) {
                result.hit = 0;
                return result;
            }

            // amount of penetration along B's axes
            ZMath::Vec2D faceB = ZMath::abs(dB) - hB - CT * hA;
            if (faceB.x > 0 || faceB.y > 0) {
                result.hit = 0;
                return result;
            }

            // * Find the best axis (i.e. the axis with the least amount of penetration).

            // Assume A's x-axis is the best axis first.
            Axis axis = FACE_A_X;
            float separation = faceA.x;
            result.normal = dA.x > 0.0f ? box1.rot.c1 : -box1.rot.c1;

            // tolerance values
            float relativeTol = 0.95f;
            float absoluteTol = 0.01f;

            // ? check if there is another axis better than A's x axis by checking if the penetration along
            // ?  the current axis being checked is greater than that of the current penetration
            // ?  (as greater value = less negative = less penetration).

            // A's remaining axes
            if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
                axis = FACE_A_Y;
                separation = faceA.y;
                result.normal = dA.y > 0.0f ? box1.rot.c2 : -box1.rot.c2;
            }

            // B's axes
            if (faceB.x > relativeTol * separation + absoluteTol * hB.x) {
                axis = FACE_B_X;
                separation = faceB.x;
                result.normal = dB.x > 0.0f ? box2.rot.c1 : -box2.rot.c1;
            }

            if (faceB.y > relativeTol * separation + absoluteTol * hB.y) {
                axis = FACE_B_Y;
                separation = faceB.y;
                result.normal = dB.y > 0.0f ? box2.rot.c2 : -box2.rot.c2;
            }

            // * Setup clipping plane data based on the best axis

            ZMath::Vec2D sideNormal;
            ZMath::Vec2D incidentFace[2]; // 4 vertices for the collision in 3D
            float front, negSide, posSide;

            // * Compute the clipping lines and line segment to be clipped

            switch(axis) {
                case FACE_A_X: {
                    front = box1.pos * result.normal + hA.x;
                    sideNormal = box1.rot.c2; // yNormal
                    float ySide = box1.pos * sideNormal;

                    negSide = -ySide + hA.y; // negSideY
                    posSide = ySide + hA.y; // posSideY

                    computeIncidentFace(incidentFace, hB, box2.pos, box2.rot, result.normal);
                    break;
                }

                case FACE_A_Y: {
                    front = box1.pos * result.normal + hA.y;
                    sideNormal = box1.rot.c1; // xNormal
                    float xSide = box1.pos * sideNormal;

                    negSide = -xSide + hA.x; // negSideX
                    posSide = xSide + hA.x; // posSideX

                    computeIncidentFace(incidentFace, hB, box2.pos, box2.rot, result.normal);
                    break;
                }

                case FACE_B_X: {
                    front = box2.pos * result.normal + hB.x;
                    sideNormal = box2.rot.c2; // yNormal
                    float ySide = box2.pos * sideNormal;

                    negSide = -ySide + hB.y; // negSideY
                    posSide = ySide + hB.y; // posSideY

                    computeIncidentFace(incidentFace, hA, box1.pos, box1.rot, result.normal);
                    break;
                }

                case FACE_B_Y: {
                    front = box2.pos * result.normal + hB.y;
                    sideNormal = box2.rot.c1; // xNormal
                    float xSide = box2.pos * sideNormal;

                    negSide = -xSide + hB.x; // negSideX
                    posSide = xSide + hB.x; // posSideX

                    computeIncidentFace(incidentFace, hA, box1.pos, box1.rot, result.normal);
                    break;
                }
            }

            // * Clip the incident edge with box planes.

            ZMath::Vec2D clipPoints1[2];
            ZMath::Vec2D clipPoints2[2];

            // Clip to side 1
            int np = clipSegmentToLine(clipPoints1, incidentFace, -sideNormal, negSide);

            if (np < 2) {
                result.hit = 0;
                return result;
            }

            // Clip to the negative side 1
            np = clipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide);

            if (np < 2) {
                result.hit = 0;
                return result;
            }

            // * ClipPoints2 now contains the clipping points.
            // * Compute the contact points.
            
            // store the conatct points in here and add them to the dynamic array after they are determined
            ZMath::Vec2D contactPoints[2];
            np = 0;
            result.pDist = 0.0f;

            for (int i = 0; i < 2; ++i) {
                separation = result.normal * clipPoints2[i] - front;

                if (separation <= 0) {
                    contactPoints[np++] = clipPoints2[i] - result.normal * separation;
                    if (result.pDist < separation) { result.pDist = separation; }
                }
            }

            // * update the manifold to contain the results.

            result.pDist = -result.pDist;
            result.hit = 1;
            result.numPoints = np;
            result.contactPoints = new ZMath::Vec2D[np];

            for (int i = 0; i < np; ++i) { result.contactPoints[i] = contactPoints[i]; }
            
            return result;
        };
    }

    // // Find the collision features and resolve the impulse between two arbitrary primitives.
    // CollisionManifold findCollisionFeatures(Primitives::RigidBody3D* rb1, Primitives::RigidBody3D* rb2) {
    //     switch (rb1->colliderType) {
    //         case Primitives::RIGID_SPHERE_COLLIDER: {
    //             if (rb2->colliderType == Primitives::RIGID_SPHERE_COLLIDER) { return findCollisionFeatures(rb1->sphere, rb2->sphere); }
    //             if (rb2->colliderType == Primitives::RIGID_AABB_COLLIDER) { return findCollisionFeatures(rb1->sphere, rb2->aabb); }
    //             if (rb2->colliderType == Primitives::RIGID_CUBE_COLLIDER) { return findCollisionFeatures(rb1->sphere, rb2->cube); }

    //             break;
    //         }

    //         case Primitives::RIGID_AABB_COLLIDER: {
    //             if (rb2->colliderType == Primitives::RIGID_SPHERE_COLLIDER) {
    //                 CollisionManifold manifold = findCollisionFeatures(rb2->sphere, rb1->aabb);
    //                 manifold.normal = -manifold.normal; // flip the direction as the original order passed in was reversed
    //                 return manifold;
    //             }

    //             if (rb2->colliderType == Primitives::RIGID_AABB_COLLIDER) { return findCollisionFeatures(rb1->aabb, rb2->aabb); }
    //             if (rb2->colliderType == Primitives::RIGID_CUBE_COLLIDER) { return findCollisionFeatures(rb1->aabb, rb2->cube); }

    //             break;
    //         }

    //         case Primitives::RIGID_CUBE_COLLIDER: {
    //             if (rb2->colliderType == Primitives::RIGID_SPHERE_COLLIDER) {
    //                 CollisionManifold manifold = findCollisionFeatures(rb2->sphere, rb1->cube);
    //                 manifold.normal = -manifold.normal; // flip the direction as the original order passed in was reversed
    //                 return manifold;
    //             }

    //             if (rb2->colliderType == Primitives::RIGID_AABB_COLLIDER) {
    //                 CollisionManifold manifold = findCollisionFeatures(rb2->aabb, rb1->cube);
    //                 manifold.normal = -manifold.normal; // flip the direction as the original order passed in was reversed
    //                 return manifold;
    //             }

    //             if (rb2->colliderType == Primitives::RIGID_CUBE_COLLIDER) { return findCollisionFeatures(rb1->cube, rb2->cube); }

    //             break;
    //         }

    //         default: {
    //             // * User defined types go here.
    //             break;
    //         }
    //     }

    //     return {ZMath::Vec3D(), nullptr, -1.0f, 0, 0};
    // };
}
