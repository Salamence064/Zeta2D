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
Vec2D(Vec2D const &vec); // Initialize the components to that of another Vec2D.
```

#### <span style="color:steelblue">Operators</span>

```c++
Vec2D operator + (Vec2D const &vec) const; // Add the vectors component-wise.
Vec2D operator + (float c) const;          // Add a float to both vector components.
Vec2D operator - (Vec2D const &vec) const; // Subtract vec from this component-wise.
Vec2D operator * (float c) const;          // Multiply both components by a float.
float operator * (Vec2D const &vec) const; // Calculate the dot product of the two vectors.

Vec2D operator - () const;                 // Multiply both components by -1.

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
void set(Vec2D const &vec);             // Set the components to the components of the other vector.

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

___

## <span style="color:fuchsia">Functions</span>

### <span style="color:darkolivegreen">SIGNOF</span>

### <span style="color:darkolivegreen">TORADIANS</span>

### <span style="color:darkolivegreen">MIN</span>

### <span style="color:darkolivegreen">MAX</span>


___

## <span style="color:fuchsia">Constants</span>
