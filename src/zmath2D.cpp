#include <ZETA/zmath2D.h>

namespace ZMath {
    // * ===============
    // * Vec2D Stuff
    // * ===============

    void Vec2D::zero() {
        x = 0;
        y = 0;
    };

    void Vec2D::set (Vec2D const &vec) {
        this->x = vec.x;
        this->y = vec.y;
    };

    void Vec2D::set (float d) {
        x = d;
        y = d;
    };

    // * Set each component of this vector.
    // * Less expensive than creating a new Vec3D object.
    void Vec2D::set (float i, float j) {
        x = i;
        y = j;
    };

    Vec2D Vec2D::operator + (Vec2D const &vec) const { return Vec2D(x + vec.x, y + vec.y); };
    Vec2D Vec2D::operator - (Vec2D const &vec) const { return Vec2D(x - vec.x, y - vec.y); };
    Vec2D Vec2D::operator * (float c) const { return Vec2D(c*x, c*y); };
    float Vec2D::operator * (Vec2D const &vec) const { return x * vec.x + y * vec.y; };

    // * Add a constant to each vector component.
    Vec2D Vec2D::operator + (float c) const { return Vec2D(x + c, y + c); };

    bool Vec2D::operator != (Vec2D const &vec) const { return x != vec.x || y != vec.y; };
    bool Vec2D::operator == (Vec2D const &vec) const { return x == vec.x && y == vec.y; };

    Vec2D& Vec2D::operator += (Vec2D const &vec) {
        x += vec.x;
        y += vec.y;

        return (*this);
    };

    Vec2D& Vec2D::operator += (float c) {
        x += c;
        y += c;

        return (*this);
    };

    Vec2D& Vec2D::operator -= (Vec2D const &vec) {
        x -= vec.x;
        y -= vec.y;

        return (*this);
    };

    Vec2D& Vec2D::operator -= (float c) {
        x -= c;
        y -= c;

        return (*this);
    };

    Vec2D& Vec2D::operator *= (float c) {
        x *= c;
        y *= c;

        return (*this);
    };

    Vec2D Vec2D::operator - () const { return Vec2D(-x, -y); };

    // * Get the cross product of this and another vector.
    float Vec2D::cross (Vec2D const &vec) const { return x*vec.y - y*vec.x; };

    // * Get the magnitude.
    float Vec2D::mag() const { return sqrtf(x*x + y*y); };

    // * Get the magnitude squared.
    // * This should be used over mag() when possible as it is less expensive.
    float Vec2D::magSq() const { return x*x + y*y; };

    // * Get the vector projection of another vector onto this vector (Parameter onto this).
    Vec2D Vec2D::proj (Vec2D const &vec) const { return (*this) * ((x*vec.x + y*vec.y)/(x*x + y*y)); };

    // * Get the distance between this and another vector.
    float Vec2D::dist (Vec2D const &vec) const { return sqrtf((x - vec.x) * (x - vec.x) + (y - vec.y) * (y - vec.y)); };

    // * Get the distance squared between this and another vector.
    // * This should be used over dist() when possible as it is less expensive.
    float Vec2D::distSq (Vec2D const &vec) const { return (x - vec.x) * (x - vec.x) + (y - vec.y) * (y - vec.y); };

    // * Get the normal vector. This is used to determine the direction a vector is pointing in.
    Vec2D Vec2D::normalize() const { return (*this) * (1.0f/sqrtf(x*x + y*y)); };

    // * Get the angle between the vectors in radians.
    // * Keep in mind range restrictions for arccos.
    // * This function is very expensive. Only call if absolutely needed.
    float Vec2D::angle (Vec2D const &vec) const { return acos((x*vec.x + y*vec.y)/(sqrtf(x*x + y*y) * sqrtf(vec.x*vec.x + vec.y*vec.y))); };

    // * Get the sign of each entry.
    Vec2D Vec2D::getSigns() const { return Vec2D(SIGNOF(x), SIGNOF(y)); };


    // * =====================
    // * Utility Functions
    // * =====================

    // * Get the absolute value of all components in a 2D vector.
    Vec2D abs(const Vec2D &vec) { return Vec2D(std::fabs(vec.x), std::fabs(vec.y)); };

    /**
     * @brief Rotate a point in 2D space about an origin.
     * 
     * @param point The point to rotate.
     * @param origin The origin the point will be rotated about.
     * @param angle The angle, in degrees, to rotate the point by.
     */
    void rotate(Vec2D &point, Vec2D const &origin, float angle) {
        float x = point.x - origin.x, y = point.y - origin.y;

        float c = cosf(TORADIANS(angle));
        float s = sinf(TORADIANS(angle));

        // compute the new point
        point.x = x*c - y*s + origin.x;
        point.y = x*s + y*c + origin.y;
    };

    // * Handle tolerance for floating point numbers.
    // * If no epsilon is specified, the default of 5 * 10^-4 will be used.
    bool compare(float a, float b, float epsilon) { return std::fabs(a - b) <= epsilon; };

    // * Handle tolerance for 2D vectors of floatinf point numbers.
    // * If no epsilon is specified, the defauly of 5 * 10^-4 will be used.
    bool compare(Vec2D const &u, Vec2D const &v, float epsilon) { return std::fabs(u.x - v.x) <= epsilon && std::fabs(u.y - v.y) <= epsilon; };

    // * Clamp a float between a min and max.
    float clamp(float n, float min, float max) { return MAX(MIN(n, max), min); };

    // * Clamp a Vec2D between a min and max vector.
    Vec2D clamp(const Vec2D &n, const Vec2D &min, const Vec2D &max) { return Vec2D(MAX(MIN(n.x, max.x), min.x), MAX(MIN(n.y, max.y), min.y)); };


    // * ===============
    // * Mat2D Stuff
    // * ===============
    
    Mat2D::Mat2D() {
        c1 = Vec2D(1, 0);
        c2 = Vec2D(0, 1);
    };

    // Create a 2D matrix from another 2D matrix.
    Mat2D::Mat2D (const Mat2D &mat) {
        c1.x = mat.c1.x;
        c1.y = mat.c1.y;
        c2.x = mat.c2.x;
        c2.y = mat.c2.y;
    };

    // Create a 2D matrix from 2 column vectors.
    Mat2D::Mat2D (const Vec2D &col1, const Vec2D &col2) {
        c1.x = col1.x;
        c1.y = col1.y;
        c2.x = col2.x;
        c2.y = col2.y;
    };

    // Create a 2D matrix from 4 scalars.
    Mat2D::Mat2D (float a11, float a12, float a21, float a22) {
        c1.x = a11;
        c1.y = a21;
        c2.x = a12;
        c2.y = a22;
    };

    // Set this matrix's components equal to that of another.
    void Mat2D::set (const Mat2D &mat) {
        c1.x = mat.c1.x;
        c1.y = mat.c1.y;
        c2.x = mat.c2.x;
        c2.y = mat.c2.y;
    };

    // Set this matrix's columns equal to those passed in.
    void Mat2D::set (const Vec2D &col1, const Vec2D &col2) {
        c1.x = col1.x;
        c1.y = col1.y;
        c2.x = col2.x;
        c2.y = col2.y;
    };

    // Set this matrix's elements equal to those passed in.
    void Mat2D::set(float a11, float a12, float a21, float a22) {
        c1.x = a11;
        c1.y = a21;
        c2.x = a12;
        c2.y = a22;
    };

    // Set all elements equal to 0.
    void Mat2D::zero() {
        c1.x = 0;
        c1.y = 0;
        c2.x = 0;
        c2.y = 0;
    };

    Mat2D Mat2D::operator + (const Mat2D &mat) const { return Mat2D(c1 + mat.c1, c2 + mat.c2); };
    Mat2D Mat2D::operator - (const Mat2D &mat) const { return Mat2D(c1 - mat.c1, c2 - mat.c2); };
    Mat2D Mat2D::operator * (const Mat2D &mat) const {
        return Mat2D(
            c1.x*mat.c1.x + c1.x*mat.c1.y, c1.x*mat.c2.x + c2.x*mat.c2.y,
            c1.y*mat.c1.x + c2.y*mat.c1.y, c1.y*mat.c2.x + c2.y*mat.c2.y
        );
    };

    Mat2D Mat2D::operator * (float c) const { return Mat2D(c1*c, c2*c); };
    Vec2D Mat2D::operator * (const Vec2D &vec) const { return Vec2D(c1.x*vec.x + c2.x*vec.y, c1.y*vec.x + c2.y*vec.y); };
    Mat2D Mat2D::operator + (float c) const { return Mat2D(c1 + c, c2 + c); };
    Mat2D Mat2D::operator - (float c) const { return Mat2D(c1 - c, c2 - c); };

    Mat2D& Mat2D::operator += (const Mat2D &mat) {
        c1.x += mat.c1.x;
        c1.y += mat.c1.y;
        c2.x += mat.c2.x;
        c2.y += mat.c2.y;

        return (*this);
    };

    Mat2D& Mat2D::operator += (float c) {
        c1.x += c;
        c1.y += c;
        c2.x += c;
        c2.y += c;

        return (*this);
    };

    Mat2D& Mat2D::operator -= (const Mat2D &mat) {
        c1.x -= mat.c1.x;
        c1.y -= mat.c1.y;
        c2.x -= mat.c2.x;
        c2.y -= mat.c2.y;

        return (*this);
    };

    Mat2D& Mat2D::operator -= (float c) {
        c1.x -= c;
        c1.y -= c;
        c2.x -= c;
        c2.y -= c;

        return (*this);
    };

    Mat2D& Mat2D::operator *= (const Mat2D &mat) {
        c1.x = c1.x * mat.c1.x + c2.x * mat.c1.y;
        c1.y = c1.x * mat.c2.x + c2.x * mat.c2.y;
        c2.x = c1.y * mat.c1.x + c2.y * mat.c1.y;
        c2.y = c1.y * mat.c2.x + c2.y * mat.c2.y;

        return (*this);
    };

    Mat2D& Mat2D::operator *= (float c) {
        c1.x *= c;
        c1.y *= c;
        c2.x *= c;
        c2.y *= c;

        return (*this);
    };

    Mat2D Mat2D::operator - () const { return Mat2D(-c1, -c2); };

    bool Mat2D::operator == (const Mat2D &mat) const { return c1.x == mat.c1.x && c1.y == mat.c1.y && c2.x == mat.c2.x && c2.y == mat.c2.y; };
    bool Mat2D::operator != (const Mat2D &mat) const { return c1.x != mat.c1.x || c1.y != mat.c1.y || c2.x != mat.c2.x || c2.y != mat.c2.y; };

    // Return the inverse of this matrix.
    // Note this may experience issues for matrices with determinants nearly equal to 0.
    Mat2D Mat2D::inverse() const {
        float det = c1.x * c2.y - c2.x * c1.y;
        Mat2D mat(*this);

        if (compare(det, 0)) { return mat; }; // singular matrix -- doesn't have an inverse.

        det = 1.0f/det;

        mat.set(c2.y * det, c2.x * -det, c1.x * -det, c1.x * det);
        return mat;
    };

    // Return the transpose of this matrix.
    Mat2D Mat2D::transpose() const { return Mat2D(c1.x, c1.y, c2.x, c2.y); };

    // Return a matrix with the sign of each entry.
    Mat2D Mat2D::getSigns() const { return Mat2D(c1.getSigns(), c2.getSigns()); };


    // * ========================
    // * Matrix Util Function
    // * ========================

    Mat2D abs(Mat2D const &mat) { return Mat2D(std::fabs(mat.c1.x), std::fabs(mat.c2.x), std::fabs(mat.c1.y), std::fabs(mat.c2.y)); };
}
