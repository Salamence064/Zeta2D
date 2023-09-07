#pragma once

#include <cmath>

namespace ZMath {
    // * ============================================
    // * ZMath Constants
    // * ============================================

    // * Pi constant
    #define PI 3.1415926535897932L

    // * Default tolerance value for a floating point comparison
    #define EPSILON 0.0005


    // * ============================================
    // * ZMath Macro Functions
    // * ============================================

    // * Macro to get the sign of a number.
    #define SIGNOF(num)( num < 0 ? -1 : (num ? 1 : 0) )

    // * Macro to convert to radians.
    #define TORADIANS(degrees)( (degrees/180) * PI )

    // * Macro to get the min of two numbers.
    #define MIN(a, b)( a < b ? a : b )

    // * Macro to get the max of two numbers.
    #define MAX(a, b)( a > b ? a : b )


    // * Class modeling a 2D Vector.
    class Vec2D {
        public:
            // * ===========================
            // * Vector Components
            // * ===========================

            // * x and y components.
            float x, y;

            // * ============================
            // * Constructors
            // * ============================

            // * Instantiate a Vec2D object with all components set to the same value.
            inline Vec2D(float d = 0) : x(d), y(d) {};

            // * Instantiate a Vec3D object with each component assigned.
            inline Vec2D(float i, float j) : x(i), y(j) {};

            // * Instantiate a copy of the Vec3D object passed in.
            inline Vec2D(const Vec2D &vec) : x(vec.x), y(vec.y) {};

            // * ============================
            // * Functions
            // * ============================

            // * Zero this vector.
            void zero();

            // * Set this vector's components equal to another.
            void set (Vec2D const &vec);

            // * Set all components of this vector to the same value.
            void set (float d);

            // * Set each component of this vector.
            // * Less expensive than creating a new Vec2D object.
            void set (float i, float j);

            Vec2D operator + (Vec2D const &vec) const ;
            Vec2D operator - (Vec2D const &vec) const;
            Vec2D operator * (float c) const;
            float operator * (Vec2D const &vec) const;

            // * Add a constant to each vector component.
            Vec2D operator + (float c) const;

            bool operator != (Vec2D const &vec) const;
            bool operator == (Vec2D const &vec) const;

            Vec2D& operator += (Vec2D const &vec);
            Vec2D& operator += (float c);
            Vec2D& operator -= (Vec2D const &vec);
            Vec2D& operator -= (float c);
            Vec2D& operator *= (float c);

            Vec2D operator - () const;

            // * Get the cross product of this and another vector.
            float cross (Vec2D const &vec) const;

            // * Get the magnitude.
            float mag() const;

            // * Get the magnitude squared.
            // * This should be used over mag() when possible as it is less expensive.
            float magSq() const;

            // * Get the vector projection of another vector onto this vector (Parameter onto this).
            Vec2D proj (Vec2D const &vec) const;

            // * Get the distance between this and another vector.
            float dist (Vec2D const &vec) const;

            // * Get the distance squared between this and another vector.
            // * This should be used over dist() when possible as it is less expensive.
            float distSq (Vec2D const &vec) const;

            // * Get the normal vector. This is used to determine the direction a vector is pointing in.
            Vec2D normalize() const;

            // * Get the angle between the vectors in radians.
            // * Keep in mind range restrictions for arccos.
            // * This function is very expensive. Only call if absolutely needed.
            float angle (Vec2D const &vec) const;

            // * Get the sign of each entry.
            Vec2D getSigns() const;
    };


    // * ============================================
    // * Utility Functions
    // * ============================================

    // * Get the absolute value of all components in a 2D vector.
    extern Vec2D abs(const Vec2D &vec);

    /**
     * @brief Rotate a point in 2D space about an origin.
     * 
     * @param point The point to rotate.
     * @param origin The origin the point will be rotated about.
     * @param angle The angle, in degrees, to rotate the point by.
     */
    extern void rotate(Vec2D &point, Vec2D const &origin, float angle);

    // * Handle tolerance for floating point numbers.
    // * If no epsilon is specified, the default of 5 * 10^-4 will be used.
    extern bool compare(float a, float b, float epsilon = EPSILON);

    // * Handle tolerance for 2D vectors of floatinf point numbers.
    // * If no epsilon is specified, the defauly of 5 * 10^-4 will be used.
    extern bool compare(Vec2D const &u, Vec2D const &v, float epsilon = EPSILON);

    // * Clamp a float between a min and max.
    extern float clamp(float n, float min, float max);

    // * Clamp a Vec2D between a min and max vector.
    extern Vec2D clamp(const Vec2D &n, const Vec2D &min, const Vec2D &max);


    // * Class modeling a 2x2 Matrix stored in column major order.
    class Mat2D {
        public:
            // Matrix columns.
            Vec2D c1, c2;

            // Initialize as the identity matrix.
            Mat2D();

            // Create a 2D matrix from another 2D matrix.
            Mat2D (const Mat2D &mat);

            // Create a 2D matrix from 2 column vectors.
            Mat2D (const Vec2D &col1, const Vec2D &col2);

            // Create a 2D matrix from 4 scalars.
            Mat2D (float a11, float a12, float a21, float a22);

            // Set this matrix's components equal to that of another.
            void set (const Mat2D &mat);

            // Set this matrix's columns equal to those passed in.
            void set (const Vec2D &col1, const Vec2D &col2);

            // Set this matrix's elements equal to those passed in.
            void set(float a11, float a12, float a21, float a22);

            // Set all elements equal to 0.
            void zero();

            Mat2D operator + (const Mat2D &mat) const;
            Mat2D operator - (const Mat2D &mat) const;
            Mat2D operator * (const Mat2D &mat) const;
            Mat2D operator * (float c) const;
            Vec2D operator * (const Vec2D &vec) const;
            Mat2D operator + (float c) const;
            Mat2D operator - (float c) const;

            Mat2D& operator += (const Mat2D &mat);
            Mat2D& operator += (float c);
            Mat2D& operator -= (const Mat2D &mat);
            Mat2D& operator -= (float c);
            Mat2D& operator *= (const Mat2D &mat);
            Mat2D& operator *= (float c);

            Mat2D operator - () const;

            bool operator == (const Mat2D &mat) const;
            bool operator != (const Mat2D &mat) const;

            // Return the inverse of this matrix.
            // Note this may experience issues for matrices with determinants nearly equal to 0.
            Mat2D inverse() const;

            // Return the transpose of this matrix.
            Mat2D transpose() const;

            // Return a matrix with the sign of each entry.
            Mat2D getSigns() const;


            // * ===============================
            // * Utility Matrix Functions
            // * ===============================

            // * Get the 2x2 identity matrix.
            static Mat2D identity() { return Mat2D(1, 0, 0, 1); };

            // * Generate the 2D rotation matrix given a specified angle.
            // * The angle should be in degrees.
            static Mat2D rotationMat(float theta) {
                float s = sinf(TORADIANS(theta));
                float c = cosf(TORADIANS(theta));

                return Mat2D(c, -s, s, c);
            };
    };

    // * ===================================
    // * Additional Utility Functions
    // * ===================================

    // * Take the absolute value of each element of a 2x2 matrix.
    extern Mat2D abs(Mat2D const &mat);
}
