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
&ensp; &ensp; Create a 2D ray from an origin point and normalized direction vector.  
  
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
&ensp; &ensp; Create a 2D line from a starting and ending point.  
  
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
&ensp; &ensp; Create a circle from a centerpoint and a radius.  
  
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
&ensp; &ensp; Create an AABB from a min vertex and a max vertex.  
  
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
&ensp; &ensp; Create a Box2D from a min vertex, a max vertex, and an angle in degrees.  
  
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

___

## <span style="color:fuchsia">Structs</span>
This subsection talks about the structs contained in the Primitives namespace. Both of these structs &#8212; RigidBody2D and StaticBody2D &#8212; store vital information about a rigid or static body respectively. 

___

## <span style="color:fuchsia">Enums</span>
The primitives namespace has two enums. Both are used to specify the collider types for rigid and static bodies.
