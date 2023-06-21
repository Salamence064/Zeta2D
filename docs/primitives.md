# Primitives

This section covers the Primitives namespace. Primitives contains all of the basic shapes defined in Zeta and rigid and static bodies. Both primitives.h and bodies.h contain code in the Primitives namespace.

___

## <span style="color:fuchsia">Classes</span>
This subsection touches on the classes found in the Primitives namespace. Each of these classes represent a different simple shape that will be recognized for the engine. The ones that can be used as a collider will be marked as such.

### <span style="color:darkolivegreen">Ray2D</span>
This class models a 2D ray. Rays are most notably used to simulate light, but can also be used to model the path an object is taking. Ray2D does not contain any functions or operators. Below are the fields and the constructor for Ray2D.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">origin</span> | The ray's origin point. |
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">dir</span> | The ray's direction as a normalized vector. |

#### <span style="color:steelblue">Constructor</span>
<span style="color:slategrey">Description:</span>  

* Create a 2D ray from an origin point and normalized direction vector.  
  
<span style="color:slategrey">Parameters:</span>

* origin (Vec2D) - the ray's origin point
* dir (Vec2D) - the ray's normalized direction vector  

```c++
Ray2D(ZMath::Vec2D const &origin, ZMath::Vec2D const &dir);
```

### <span style="color:darkolivegreen">Line2D</span>
This class models a 2D line segment, which will be referred to as a "line" for convenience. Lines are not commonly used for physics, but it is included in case you need one for a niche purpose. Below are the fields, constructor, and functions comprising the class.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">start</span> | The line's starting point. |
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">end</span> | The line's ending point. |

#### <span style="color:steelblue">Constructor</span>
<span style="color:slategrey">Description:</span>  

* Create a 2D line from a starting and ending point.  
  
<span style="color:slategrey">Parameters:</span>

* start (Vec2D) - the line's starting point
* end (Vec2D) - the line's ending point

```c++
Line2D(ZMath::Vec2D const &start, ZMath::Vec2D const &end);
```

#### <span style="color:steelblue">Functions</span>
```c++
ZMath::Vec2D getMin() const; // Returns the lowest value of x and y the line reaches.
ZMath::Vec2D getMax() const; // Returns the greatest value of x and y the line reaches.
```

### <span style="color:darkolivegreen">Circle</span>
This class models a circle. Circles are a common choice for colliders, especially to model balls, projectiles, and sometimes even players and enemies. This class does not contain any functions. Below are the fields and constructor.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">c</span> | The circle's centerpoint. |
| <span style="color:hotpink">float</span> | <span style="color:seagreen">r</span> | The circle's radius. |

#### <span style="color:steelblue">Constructor</span>
<span style="color:slategrey">Description:</span>  

* Create a circle from a centerpoint and a radius.  
  
<span style="color:slategrey">Parameters:</span>

* c (Vec2D) - the circle's centerpoint
* r (float) - the circle's radius

```c++
Circle(ZMath::Vec2D const &c, float r);
```

### <span style="color:darkolivegreen">AABB</span>
This class models an unrotated rectangle. This is used over a Box2D for the unrotated case as it allows for less computationally expensive functions to be ran. AABBs are a common choice for colliders, especially for walls and box-like entities. Below are the fields, constructor, and functions.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">pos</span> | The AABB's centerpoint. |
| <span style="color:deeppink">private</span> <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">halfsize</span> | Half the dimensions of the AABB. |

#### <span style="color:steelblue">Constructor</span>
<span style="color:slategrey">Description:</span>  

* Create an AABB from a min vertex and a max vertex.  
  
<span style="color:slategrey">Parameters:</span>

* min (Vec2D) - the AABB's vertex with the lowest values of x and y contained in the AABB (known as the min vertex)
* max (Vec2D) - the AABB's vertex with the greatest values of x and y contained in the AABB (known as the max vertex)

```c++
AABB(ZMath::Vec2D const &min, ZMath::Vec2D const &max);
```

#### <span style="color:steelblue">Functions</span>
```c++
ZMath::Vec2D getMin() const;       // Returns the AABB's min vertex.
ZMath::Vec2D getMax() const;       // Returns the AABB's max vertex.
ZMath::Vec2D getHalfsize() const;  // Returns the AABB's halfsize.
ZMath::Vec2D* getVertices() const; // Returns a pointer array of size 4 with the vertices of the AABB.
```

### <span style="color:darkolivegreen">Box2D</span>
This class models a rotated rectangle. This should only be used for non-zero cases of rotation as it is more computationally expensive to conduct Box2D collision checks than AABB checks. Box2Ds are a common choice for colliders, especially for non-aligned walls or box-like entities. Below are the fields, functions, and constructor.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">pos</span> | The Box2D's centerpoint. |
| <span style="color:hotpink">Mat2D</span> | <span style="color:seagreen">rot</span> | The Box2D's rotation matrix. This will rotate anything from the Box2D's local space to global space. |
| <span style="color:hotpink">float</span> | <span style="color:seagreen">theta</span> | The angle the Box2D is rotated by in degrees. |
| <span style="color:deeppink">private</span> <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">halfsize</span> | Half the dimensions of the Box2D. |

#### <span style="color:steelblue">Constructor</span>
<span style="color:slategrey">Description:</span>  

* Create a Box2D from a min vertex, a max vertex, and an angle in degrees.  
  
<span style="color:slategrey">Parameters:</span>

* min (Vec2D) - the Box2D's vertex with the lowest values of x and y contained in the Box2D if it was not rotated (known as the min vertex)
* max (Vec2D) - the Box2D's vertex with the greatest values of x and y contained in the Box2D if it was not rotated (known as the max vertex)
* theta (float) - the angle the Box2D is rotated by in degrees.

```c++
Box2D(ZMath::Vec2D const &min, ZMath::Vec2D const &max, float theta);
```

#### <span style="color:steelblue">Functions</span>
```c++
ZMath::Vec2D getLocalMin() const;  // Returns the Box2D's min vertex in the Box2D's local space.
ZMath::Vec2D getLocalMax() const;  // Returns the Box2D's max vertex in the Box2D's local space.
ZMath::Vec2D getHalfsize() const;  // Returns the Box2D's halfsize.
ZMath::Vec2D* getVertices() const; // Returns a pointer array of size 4 with the vertices of the Box2D in global space.
```

### <span style="color:darkolivegreen">RigidBody2D</span>
This class models a 2D rigid body. A rigid body is an object that's affected by physics. It has various fields storing information to use for physics updates. Below are the core fields and functions.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">pos</span> | The rigid body's centerpoint. |
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">vel</span> | The rigid body's velocity. It is initialized to the 0 vector by default. |
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">netForce</span> | The rigid body's net force. It is initialized to the 0 vector by default. |
| <span style="color:hotpink">float</span> | <span style="color:seagreen">mass</span> | The mass of the rigid body in grams. |
| <span style="color:hotpink">float</span> | <span style="color:seagreen">invMass</span> | 1 over the mass of the rigid body in grams. |
| <span style="color:hotpink">float</span> | <span style="color:seagreen">cor</span> | The coefficient of restitution of the rigid body.This represents a loss of kinetic energy due<br>to heat and should be between 0 and 1. 1 is perfectly elastic and 0 is perfectly inelastic. |
| <span style="color:hotpink">float</span> | <span style="color:seagreen">linearDamping</span> | Controls how much the rigid body resists translation and should be on the interval (0, 1].<br>1 = no resistance to translation. |
| <span style="color:hotpink">RigidBodyCollider</span> | <span style="color:seagreen">colliderType</span> | The collider type attached to the rigid body. |
| <span style="color:hotpink"><br>Union</span> | <span style="color:seagreen"><br>collider</span> | A union containing a circle, AABB, and Box2D referenced by .circle, .aabb, and .box respectively.<br>Only use the collider associated with the collider type attached. You **must manually assign this**<br>and assigning the wrong collider will break the physics engine. |

#### <span style="color:steelblue">Constructors</span>
<span style="color:slategrey">Description:</span>  

* Create a 2D rigid body from a position, mass, coefficient of restitution, linear damping value, colliderType, and a collider.
  
<span style="color:slategrey">Parameters:</span>

* pos (Vec2D) - The centerpoint of the rigid body. This should be equal to the centerpoint of the collider.
* mass (float) - The mass of the rigid body in grams.
* cor (float) - The coefficient of restitution of the rigid body. Should be between 0 and 1 inclusive.
* linearDamping (float) - The linear damping of the rigid body. Should be on the interval (0, 1].
* colliderType (RigidBodyCollider) - Enum value informing the engine which type of collider is attached to the rigid body.
* collider (void*) - Pointer to the collider of the rigid body. If this does not match the collider type specified, undefined behavior will occur. If you specify the RIGID_NONE collider type, you should set this to nullptr. Delete will not be called on this pointer.

```c++
RigidBody2D(
    ZMath::Vec2D const &pos,
    float mass,
    float cor,
    float linearDamping, 
    RigidBodyCollider colliderType,
    void* collider
);
```

RigidBody2D also offers a default constructor that does nothing. If you use the default constructor, you must manually assign **every field** or the rigid body will cause undefined behavior.


#### <span style="color:steelblue">Functions</span>
<span style="color:slategrey">Function Signature:</span>

```c++
void update(ZMath::Vec2D const &g, float dt);
```

<span style="color:slategrey">Description:</span>  

* Updates the rigid body based on its current physics attributes. The physics handler will run this for it so it is **not recommended** to call this.

<span style="color:slategrey">Parameters:</span>

* g (Vec2D) - The acceleration due to gravity.
* dt (float) - Amount of time passed since the last update.


### <span style="color:darkolivegreen">StaticBody2D</span>
This class models a 2D static body. A static body is an object unaffected by physics. Static bodies are commonly used to model walls, goals, death zones, etc. as those objects should be unaffected by physics. Static bodies still contain a collider, allowing you to check and resolve static body collisions how you see fit. Below are the core fields.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">pos</span> | The static body's centerpoint. |
| <span style="color:hotpink">StaticBodyCollider</span> | <span style="color:seagreen">colliderType</span> | The collider type attached to the static body. |
| <span style="color:hotpink">Union</span> | <span style="color:seagreen">collider</span> | A union containing a circle, AABB, and Box2D referenced by .circle, .aabb, and .box respectively.<br>Only use the collider associated with the collider type attached. |


#### <span style="color:steelblue">Constructors</span>
<span style="color:slategrey">Description:</span>  

* Create a 2D static body from a position, colliderType, and collider.
  
<span style="color:slategrey">Parameters:</span>

* pos (Vec2D) - The centerpoint of the static body. This should be equal to the centerpoint of the collider.
* colliderType (StaticBodyCollider) - Enum value informing the engine which type of collider is attached to the static body.
* collider (void*) - Pointer to the collider of the static body. If this does not match the collider type specified, undefined behavior will occur. If you specify the STATIC_NONE collider type, you should set this to nullptr. Delete will not be called on this pointer.

```c++
StaticBody2D(ZMath::Vec2D const &pos, StaticBodyCollider colliderType, void* collider);
```

StaticBody2D also offers a default constructor that does nothing. If you use the default constructor, you must manually assign **every field** or the static body will cause undefined behavior.

___

## <span style="color:fuchsia">Enums</span>
The primitives namespace has two enums. Both are used to specify the collider types for rigid and static bodies.

### <span style="color:darkolivegreen">RigidBodyCollider</span>
This enum is used to indicate the type of collider attached to a rigid body.

| <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----------:|:-----------:|
| <span style="color:hotpink">RIGID_CIRCLE_COLLIDER</span> | Indicates a circle collider is attached to the rigid body. |
| <span style="color:hotpink">RIGID_AABB_COLLIDER</span> | Indicates an AABB collider is attached to the rigid body. |
| <span style="color:hotpink">RIGID_BOX2D_COLLIDER</span> | Indicates a Box2D collider is attached to the rigid body. |
| <span style="color:hotpink">RIGID_CUSTOM_COLLIDER</span> | Indicates a custom collider is attached to the rigid body. |
| <span style="color:hotpink">RIGID_NONE</span> | Indicates no collider is attached to the rigid body. |


### <span style="color:darkolivegreen">StaticBodyCollider</span>
This enum is used to indicate the type of collider attached to a static body.

| <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----------:|:-----------:|
| <span style="color:hotpink">STATIC_CIRCLE_COLLIDER</span> | Indicates a circle collider is attached to the static body. |
| <span style="color:hotpink">STATIC_AABB_COLLIDER</span> | Indicates an AABB collider is attached to the static body. |
| <span style="color:hotpink">STATIC_BOX2D_COLLIDER</span> | Indicates a Box2D collider is attached to the static body. |
| <span style="color:hotpink">STATIC_CUSTOM_COLLIDER</span> | Indicates a custom collider is attached to the static body. |
| <span style="color:hotpink">STATIC_NONE</span> | Indicates no collider is attached to the static body. |