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

#### <span style="color:steelblue">Fields</span>

### <span style="color:darkolivegreen">AABB</span>

#### <span style="color:steelblue">Fields</span>

### <span style="color:darkolivegreen">Box2D</span>

#### <span style="color:steelblue">Fields</span>

___

## <span style="color:fuchsia">Structs</span>
This subsection talks about the structs contained in the Primitives namespace. Both of these structs &#8212; RigidBody2D and StaticBody2D &#8212; store vital information about a rigid or static body respectively. 

___

## <span style="color:fuchsia">Enums</span>
The primitives namespace has two enums. Both are used to specify the collider types for rigid and static bodies.
