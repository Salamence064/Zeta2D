#pragma once

#include "zmath2D.h"

namespace Primitives {
    class Ray2D {
        public:
            ZMath::Vec2D origin;
            ZMath::Vec2D dir; // normalized direction of the ray

            /**
             * @brief Create a 2D ray.
             * 
             * @param origin The origin of the ray.
             * @param dir The direction of the ray as a normalized vector.
             */
            Ray2D(ZMath::Vec2D const &origin, ZMath::Vec2D const &dir) : origin(origin), dir(dir) {};
    };

    class Line2D {
        public:
            ZMath::Vec2D start, end;

            /**
             * @brief Create a 2D line.
             * 
             * @param start Starting point of the line.
             * @param end Ending point of the line.
             */
            Line2D(ZMath::Vec2D const &start, ZMath::Vec2D const &end) : start(start), end(end) {};

            // A vector with the lowest value of x and y the line segment reaches.
            inline ZMath::Vec2D getMin() const { return ZMath::Vec2D(MIN(start.x, end.x), MIN(start.y, end.y)); };

            // A vector with the greatest value of x and y the line segment reaches.
            inline ZMath::Vec2D getMax() const { return ZMath::Vec2D(MAX(start.x, end.x), MAX(start.y, end.y)); };
    };

    class Circle {
        public:
            ZMath::Vec2D c; // centerpoint
            float r; // radius

            /**
             * @brief Create a Circle with a given center and radius.
             * 
             * @param c Centerpoint of the circle.
             * @param r Radius of the circle.
             */
            Circle(ZMath::Vec2D const &c, float r) : c(c), r(r) {};
    };

    class AABB {
        private:
            ZMath::Vec2D halfsize;

        public:
            ZMath::Vec2D pos; // Centerpoint of the AABB.

            /**
             * @brief Create an unrotated 2D rectangle.
             * 
             * @param min Min vertex of the AABB.
             * @param max Max vertex of the AABB.
             */
            AABB(ZMath::Vec2D const &min, ZMath::Vec2D const &max) {
                halfsize = (max - min) * 0.5f;
                pos = min + halfsize;
            };

            inline ZMath::Vec2D getMin() const { return pos - halfsize; };
            inline ZMath::Vec2D getMax() const { return pos + halfsize; };

            // Get half the distance between the AABB's min and max vertices.
            inline ZMath::Vec2D getHalfsize() const { return halfsize; };

            // Get the vertices of the AABB.
            // Remember to call delete[] on what you assign this to afterwards to free the memory.
            ZMath::Vec2D* getVertices() const {
                ZMath::Vec2D* v = new ZMath::Vec2D[4];

                // todo ensure the order is the same as the order OpenGL likes

                v[0] = pos - halfsize;
                v[1] = ZMath::Vec2D(pos.x - halfsize.x, pos.y + halfsize.y);
                v[2] = ZMath::Vec2D(pos.x + halfsize.x, pos.y - halfsize.y);
                v[3] = pos + halfsize;

                return v;
            };
    };

    class Box2D {
        private:
            ZMath::Vec2D halfsize;

        public:
            ZMath::Vec2D pos; // Centerpoint of the Box2D.
            ZMath::Mat2D rot; // Rotate anything from this Box2D's local space to global space. Cached for efficiency.
            float theta; // Rotation of the Box2D in degrees.

            /**
             * @brief Create a rotated 2D rectangle.
             * 
             * @param min Min vertex of the Box2D as if it was not rotated.
             * @param max Max vertex of the Box2D as if it was not rotated.
             * @param theta Angle the Box2D is rotated by in degrees.
             */
            Box2D(ZMath::Vec2D const &min, ZMath::Vec2D const &max, float theta) {
                halfsize = (max - min) * 0.5f;
                pos = min + halfsize;
                this->theta = theta;
                rot = ZMath::Mat2D::rotationMat(theta);
            };

             // Get the min vertex in the Box2D's UV coordinates.
            inline ZMath::Vec2D getLocalMin() const { return pos - halfsize; };

             // Get the max vertex in the Box2D's UV coordinates.
            inline ZMath::Vec2D getLocalMax() const { return pos + halfsize; };

            // Get half the distance between the Box2D's min and max vertices.
            inline ZMath::Vec2D getHalfsize() const { return halfsize; };

            // Get the vertices of the Box2D in terms of global coordinates.
            // Remeber to use delete[] on the variable you assign this after use to free the memory.
            ZMath::Vec2D* getVertices() const {
                ZMath::Vec2D* v = new ZMath::Vec2D[4];

                // todo reorder to match OpenGL bindings

                v[0] = -halfsize;
                v[1] = ZMath::Vec2D(-halfsize.x, halfsize.y);
                v[2] = ZMath::Vec2D(halfsize.x, -halfsize.y);
                v[3] = halfsize;

                // rotate the vertices
                for (int i = 0; i < 4; ++i) { v[i] = rot * v[i] + pos; }
                
                return v;
            };
    };
}
