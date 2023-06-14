# ZMath

This section covers the ZMath namespace. ZMath contains all of Zeta's custom math library's code and many helpful math functions. zmath2D.h contains all the code contained in the ZMath namespace.

___

## <span style="color:fuchsia">Classes</span>

This subsection touches on the classes found in the ZMath namespace. There are two classes used for Zeta's math library: Vec2D and Mat2D.

### <span style="color:darkolivegreen">Vec2D</span>
This class models a 2D vector. It's used extensibly throughout Zeta and will be how you pass information about positions, velocities forces, etc. to the physics engine. Any function described as being "expensive" is expensive in the computational sense. Below are the core functions and operators.

#### <span style="color:steelblue">Fields</span>

| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">int</span> | <span style="color:seagreen">x</span> | The vector's x component. |
| <span style="color:hotpink">int</span> | <span style="color:seagreen">y</span> | The vector's y component. |

#### <span style="color:steelblue">Constructors</span>
```c++
Vec2D(float d = 0);      // Set both components to the same value. Default of 0.
Vec2D(float i, float j); // X is initialized to i and y to j.
Vec2D(Vec2D const &vec); // Initialize the components to those of another vector.
```

#### <span style="color:steelblue">Operators</span>

```c++
Vec2D operator + (Vec2D const &vec) const; // Add the vectors component-wise.
Vec2D operator + (float c) const;          // Add a float to both vector components.
Vec2D operator - (Vec2D const &vec) const; // Subtract the vectors component-wise.
Vec2D operator * (float c) const;          // Multiply both components by a float.
float operator * (Vec2D const &vec) const; // Calculate the dot product of the two vectors.

Vec2D operator - () const;                 // Negate both components.

bool operator == (Vec2D const &vec) const; // Bool representing if the two vectors are equal.
bool operator != (Vec2D const &vec) const; // Bool representing if the two vectors are not equal.

Vec2D& operator += (Vec2D const& vec);     // Make this vector the sum of the vectors component-wise.
Vec2D& operator += (float c);              // Modify this vector by adding a float to both components.
Vec2D& operator -= (Vec2D const& vec);     // Make this vector the difference of the vectors component-wise.
Vec2D& operator *= (float c);              // Modify this vector by multiplying both components by a float.
```

#### <span style="color:steelblue">Functions</span>

```c++
void zero();                            // Set both components to 0.
void set(float d);                      // Set both components to the same float.
void set(float i, float j);             // Set x to i and y to j.
void set(Vec2D const &vec);             // Set the components to those of another vector.

float cross(Vec2D const &vec) const;    // Take the cross product of the two vectors.
float mag() const;                      // Get the magnitude of the vector. This requires a sqrt.
float magSq() const;                    // Get the magnitude squared of the vector.
float dist(Vec2D const &vec) const;     // Get the distance between two vectors. This requires a sqrt.
float distSq(Vec2D const& vec) const;   // Get the distance between two vectors squared.
float getAngle(Vec2D const& vec) const; // Get the angle between two vectors. This function is expensive.

Vec2D proj(Vec2D const &vec) const;     // Take the vector projection of vec (parameter) onto this vector.
Vec2D normalize() const;                // Get the normalized vector in the same direction as this one.
Vec2D getSigns() const;                 // Get a vector with the signs of each component.
```

### <span style="color:darkolivegreen">Mat2D</span>
This class models a 2x2 matrix. It's used primarily to store rotation matrices to rotate points in the physics engine. Any function described as being "expensive" is expensive in the computational sense. Below are the core functions and operators.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">c1</span> | The leftmost column vector of the matrix. |
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">c2</span> | The rightmost column vector of the matrix. |

#### <span style="color:steelblue">Constructors</span>

```c++
// Initialize the matrix as follows:
// |a11, a12|
// |a21, a22|
Mat2D(float a11, float a12, float a21, float a22); // Check the above comment.
Mat2D();                                           // Initialize this matrix as the identity matrix.
Mat2D(Vec2D const &col1, Vec2D const &col2);       // Initialize c1 to col1 and c2 to col2.
Mat2D(Mat2D const &mat);                           // Initialize the column vectors to those of another matrix.
```

#### <span style="color:steelblue">Operators</span>
```c++
Mat2D operator + (Mat2D const &mat) const; // Add the matrices element-wise.
Mat2D operator + (float c) const;          // Add a float to each element of the matrix.
Mat2D operator - (Mat2D const &mat) const; // Subtract the matrices element-wise.
Mat2D operator - (float c) const;          // Subtract a float from each element.
Mat2D operator * (Mat2D const &mat) const; // Matrix multiplication of the two matrices.
Mat2D operator * (Vec2D const &vec) const; // Matrix multiplication with vec treated as a 2x1 column vector.
Mat2D operator * (float c) const;          // Multiply each element by a float.

Mat2D operator - () const;                 // Negate each element.

bool operator == (Mat2D const &mat) const; // Bool representing if the matrices are equal.
bool operator != (Mat2D const &mat) const; // Bool representing if the matrices are not equal.

Mat2D& operator += (Mat2D const &mat);     // Make this matrix the sum of the matrices element-wise.
Mat2D& operator += (float c);              // Modify this matrix by adding a float to each element.
Mat2D& operator -= (Mat2D const &mat);     // Make this matrix the difference of the matrices element-wise.
Mat2D& operator -= (float c);              // Modify this matrix by subtracting a float from each element.
Mat2D& operator *= (Mat2D const &mat);     // Make this matrix the product from matrix multiplication.
Mat2D& operator *= (float c);              // Modify this matrix by multiplying each element by a float.
```

#### <span style="color:steelblue">Functions</span>
```c++
// Set the matrix's elements as follows:
// |a11, a12|
// |a21, a22|
void set(float a11, float a12, float a21, float a22); // Check the above comment.
void set(Vec2D const &col1, Vec2D const &col2);       // Set c1 to col1 and c2 to col2.
void set(Mat2D const &mat);                           // Set the elements to those of another matrix.
void zero();                                          // Set all elements to 0.

Mat2D inverse() const;                                // Get the inverse of this matrix.
Mat2D transpose() const;                              // Get the transpose of this matrix.
Mat2D getSigns() const;                               // Get the signs of each element of this matrix.

// =====================
// Static functions
// =====================

static Mat2D identity();               // Get the identity matrix.
static Mat2D rotationMat(float theta); // Get the rotation matrix for an angle. Theta should be in degrees.
```
___

## <span style="color:fuchsia">Functions</span>
This subsection highlights essential math functions used throughout the engine. Many of these are simple utility functions, making them convenient to use in your program. Below are all of the standalone functions provided in ZMath.

### <span style="color:darkolivegreen">SIGNOF</span>
Macro function to get the sign of a number. This returns -1 for a negative number and 1 for a positive number. 0 is treated as positive. The parameter can be any numeric data type.
```c++
SIGNOF(num);
```

### <span style="color:darkolivegreen">TORADIANS</span>
Macro function to convert an angle in degrees to radians. This allows for storage of angles in degrees while still using cmath trig functions. The parameter can be any numeric data type and should be an angle in degrees.
```c++
TORADIANS(degrees);
```

### <span style="color:darkolivegreen">MIN</span>
Macro function that returns the minimum of two numbers. The parameters can be any numeric data types.
```c++
MIN(a, b);
```

### <span style="color:darkolivegreen">MAX</span>
Macro function that returns the maximum of two numbers. The parameters can be any numeric data types.
```c++
MAX(a, b);
```

### <span style="color:darkolivegreen">abs</span>
Function returning the absolute value of a vector component-wise.
```c++
Vec2D abs(Vec2D const &vec);
```

Function returning the absolute value of a matrix element-wise.
```c++
Mat2D abs(Mat2D const &mat);
```

### <span style="color:darkolivegreen">rotate</span>
Function that rotates a point in 2D space about an origin. The point's components will be updated directly. The angle to rotate by should be in degrees.
```c++
void rotate(Vec2D &point, Vec2D const &origin, float angle);
```

### <span style="color:darkolivegreen">compare</span>
Function that returns if a float is close enough to another float to be considered approximately the same. This should be used to handle tolerance for floating point values. Epsilon is the tolerance value and is 5 * 10^-4 by default.
```c++
bool compare(float a, float b, float epsilon = EPSILON);
```

Function that returns if a Vec2D is close enough to another Vec2D to be considered approximately the same. This should be used to handle tolerance for vectors of floats. Epsilon is the tolerance value and is 5 * 10^-4 by default.
```c++
bool compare(Vec2D const &u, Vec2D const &v, float epsilon = EPSILON);
```

### <span style="color:darkolivegreen">clamp</span>
Function that returns a float clamped between a min and max. If the float is greater than max, this will return max; if the float is less than min, this will return min; otherwise, the float will be returned.
```c++
float clamp(float n, float min, float max);
```

Function that returns a Vec2D clamped component-wise between a min vector and a max vector. The process described for single float clamping is applied to each element of the Vec2D in this function.
```c++
Vec2D clamp(Vec2D const &n, Vec2D const &min, Vec2D const &max);
```

___

## <span style="color:fuchsia">Constants</span>
ZMath offers 2 constants defined through preprocessor directives for ease of use: &#960; and &#949;.

### <span style="color:darkolivegreen">PI</span>
Constant representing &#960;. This is set to 3.1415926535897932.

### <span style="color:darkolivegreen">EPSILON</span>
Constant representing the default tolerance value. This is set to 0.0005 (5 * 10^-4).
