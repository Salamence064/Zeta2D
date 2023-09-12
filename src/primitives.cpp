#include <ZETA/primitives.h>

namespace Zeta {
    // * ==========
    // * Line2D
    // * ==========

    ZMath::Vec2D Line2D::getMin() const { return ZMath::Vec2D(MIN(start.x, end.x), MIN(start.y, end.y)); };
    ZMath::Vec2D Line2D::getMax() const { return ZMath::Vec2D(MAX(start.x, end.x), MAX(start.y, end.y)); };


    // * =======
    // * AABB
    // * =======

    AABB::AABB(ZMath::Vec2D const &min, ZMath::Vec2D const &max) {
        halfsize = (max - min) * 0.5f;
        pos = min + halfsize;
    };

    // Get the vertices of the AABB.
    // Remember to call delete[] on what you assign this to afterwards to free the memory.
    ZMath::Vec2D* AABB::getVertices() const {
        ZMath::Vec2D* v = new ZMath::Vec2D[4];

        // todo ensure the order is the same as the order OpenGL likes

        v[0] = pos - halfsize;
        v[1] = ZMath::Vec2D(pos.x - halfsize.x, pos.y + halfsize.y);
        v[2] = ZMath::Vec2D(pos.x + halfsize.x, pos.y - halfsize.y);
        v[3] = pos + halfsize;

        return v;
    };


    // * =========
    // * Box2D
    // * =========

    Box2D::Box2D(ZMath::Vec2D const &min, ZMath::Vec2D const &max, float theta) {
        halfsize = (max - min) * 0.5f;
        pos = min + halfsize;
        this->theta = theta;
        rot = ZMath::Mat2D::rotationMat(theta);
    };

    // Get the vertices of the Box2D in terms of global coordinates.
    // Remeber to use delete[] on the variable you assign this after use to free the memory.
    ZMath::Vec2D* Box2D::getVertices() const {
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
}
