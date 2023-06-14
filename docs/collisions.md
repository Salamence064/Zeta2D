# Collisions

This section covers the Collisions namespace. Collisions contains functions relating to intersection detection and collision manifolds. Both intersections.h and collisions.h contain the code in the Collisions namespace.

___

## <span style="color:fuchsia">Structs</span>
This subsection touches on the structs included in Collisions. There is only only one struct in this namespace and details about it can be found below.

### <span style="color:darkolivegreen">Collision Manifold</span>
This struct stores data related to a collision. These are used by the physics handler for impulse resolution. It is not recommended for you to create and use your own collision manifolds; however, they are provided here on the offchance you need to. Note: this struct does not have any rule of 5 functions so the memory from the pointer in here must be managed by you if you do choose to make your own manifolds.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:hotpink">bool</span> | <span style="color:seagreen">hit</span> | Bool representing if there's a collision. If this is false, all of the other fields will be junk values. |
| <span style="color:hotpink">float</span> | <span style="color:seagreen">pDist</span> | The penetration distance of the collision. |
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">normal</span> | The collision normal. |
| <span style="color:hotpink">Vec2D*</span> | <span style="color:seagreen">contactPoints</span> | The point(s) at which the colliders overlap. |
| <span style="color:hotpink">int</span> | <span style="color:seagreen">numPoints</span> | The number of contact points. |

___

## <span style="color:fuchsia">Functions</span>
This subsection showcases the many functions in this namespace. A majority of the Collisions namespace is comprised of intersection detection functions and details on those are provided here.

### <span style="color:darkolivegreen">PointAndLine</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndLine(ZMath::Vec2D const &point, Primitives::Line2D const &line);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies on a line.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* line (Line2D) - A line in 2D space.


### <span style="color:darkolivegreen">PointAndCircle</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* circle (Circle) - A circle.


### <span style="color:darkolivegreen">PointAndAABB</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndAABB(ZMath::Vec2D const &point, Primitives::AABB const &aabb);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in an AABB.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* aabb (AABB) - An unrotated rectangle.


### <span style="color:darkolivegreen">PointAndBox2D</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndBox2D(ZMath::Vec2D const &point, Primitives::Box2D const &box);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a Box2D.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* box (Box2D) - A rotated rectangle.


### <span style="color:darkolivegreen">LineAndPoint</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool LineAndPoint(Primitives::Line2D const &line, ZMath::Vec2D const &point);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies on a line.

<span style="color:slategrey">Parameters:</span>

* line (Line2D) - A line in 2D space.
* point (Vec2D) - A point in 2D space.


### <span style="color:darkolivegreen">LineAndLine</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool LineAndLine(Primitives::Line2D const &line1, Primitives::Line2D const &line2);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if two line segments intersect.

<span style="color:slategrey">Parameters:</span>

* line1 (Line2D) - A line in 2D space.
* line2 (Line2D) - A second line in 2D space.


### <span style="color:darkolivegreen">LineAndCircle</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool LineAndCircle(Primitives::Line2D const &line, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a line intersects a circle.

<span style="color:slategrey">Parameters:</span>

* line (Line2D) - A line in 2D space.
* circle (Circle) - A circle.


### <span style="color:darkolivegreen">LineAndAABB</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool LineAndAABB(Primitives::Line2D const &line, Primitives::AABB const &aabb);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a line intersects an AABB.

<span style="color:slategrey">Parameters:</span>

* line (Line2D) - A line in 2D space.
* aabb (AABB) - An unrotated rectangle.


### <span style="color:darkolivegreen">LineAndBox2D</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool LineAndBox2D(Primitives::Line2D const &line, Primitives::Box2D const &box);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a line intersects a Box2D.

<span style="color:slategrey">Parameters:</span>

* point (Line2D) - A line in 2D space.
* box (Box2D) - A rotated rectangle.


### <span style="color:darkolivegreen">raycast</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool raycast(Primitives::Ray2D const &ray, Primitives::Circle const &circle, float &dist);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a ray intersects a circle and determine the distance to the point of intersection.

<span style="color:slategrey">Parameters:</span>

* ray (Ray2D) - A ray in 2D space.
* circle (Circle) - A circle.
* dist (float) - A float updated to be the distance to the point of intersection. If this returns false, this will be a junk value.


<span style="color:slategrey">Function Signature:</span>

```c++
bool raycast(Primitives::Ray2D const &ray, Primitives::AABB const &aabb, float &dist);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a ray intersects an AABB and determine the distance to the point of intersection..

<span style="color:slategrey">Parameters:</span>

* ray (Ray2D) - A ray in 2D space.
* aabb (AABB) - An unrotated rectangle.
* dist (float) - A float updated to be the distance to the point of intersection. If this returns false, this will be a junk value.


<span style="color:slategrey">Function Signature:</span>

```c++
bool raycast(Primitives::Ray2D const &ray, Primitives::Box2D const &box, float &dist);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a ray intersects a Box2D and determine the distance to the point of intersection.

<span style="color:slategrey">Parameters:</span>

* ray (Ray2D) - A ray in 2D space.
* box (Box2D) - A rotated rectangle.
* dist (float) - A float updated to be the distance to the point of intersection. If this returns false, this will be a junk value.


### <span style="color:darkolivegreen">CircleAndPoint</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool CircleAndPoint(Primitives::Circle const &circle, ZMath::Vec2D const &point);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* circle (Circle) - A circle.
* point (Vec2D) - A point in 2D space.


### <span style="color:darkolivegreen">CircleAndLine</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool CircleAndLine(Primitives::Circle const &circle, Primitives::Line2D const &line);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a line intersects a circle.

<span style="color:slategrey">Parameters:</span>

* circle (Circle) - A circle.
* line (Vec2D) - A line in 2D space.


### <span style="color:darkolivegreen">CircleAndCircle</span>
`
<span style="color:slategrey">Function Signature:</span>

```c++
bool CircleAndCircle(Primitives::Circle const &circle1, Primitives::Circle const &circle2);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if two circles intersect.

<span style="color:slategrey">Parameters:</span>

* circle1 (Circle) - A circle.
* circle2 (Circle) - A second circle.

<span style="color:slategrey">Function Signature:</span>

```c++
bool CircleAndCircle(Primitives::Circle const &circle1, Primitives::Circle const &circle2, ZMath::Vec2D &normal);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if two circles intersect and calculate the collision normal.

<span style="color:slategrey">Parameters:</span>

* circle1 (Circle) - A circle.
* circle2 (Circle) - A second circle.
* normal (Vec2D) - Updated to be the collision normal. If this returns false, this will be a junk value.


### <span style="color:darkolivegreen">CircleAndAABB</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool CircleAndAABB(Primitives::Circle const &circle, Primitives::AABB const &aabb);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a circle intersects an AABB.

<span style="color:slategrey">Parameters:</span>

* circle (Circle) - A circle.
* aabb (AABB) - An unrotated rectangle.


<span style="color:slategrey">Function Signature:</span>

```c++
bool CircleAndAABB(Primitives::Circle const &circle, Primitives::AABB const &aabb, ZMath::Vec2D &normal);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a circle intersects an AABB and calculate the collision normal.

<span style="color:slategrey">Parameters:</span>

* circle (Circle) - A circle.
* aabb (AABB) - An unrotated rectangle.
* normal (Vec2D) - Updated to be the collision normal. If this returns false, this will be a junk value.


### <span style="color:darkolivegreen">CircleAndBox2D</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool CircleAndBox2D(Primitives::Circle const &circle, Primitives::Box2D const &box);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a circle intersects a Box2D.

<span style="color:slategrey">Parameters:</span>

* circle (Circle) - A circle.
* box (Box2D) - A rotated rectangle.


<span style="color:slategrey">Function Signature:</span>

```c++
bool CircleAndBox2D(Primitives::Circle const &circle, Primitives::Box2D const &box, ZMath::Vec2D &normal);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a circle intersects a Box2D and calculate the collision normal.

<span style="color:slategrey">Parameters:</span>

* circle (Circle) - A circle.
* box (Box2D) - A rotated rectangle.
* normal (Vec2D) - Updated to be the collision normal. If this returns false, this will be a junk value.


### <span style="color:darkolivegreen">AABBAndPoint</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool AABBAndPoint(Primitives::AABB const &aabb, ZMath::Vec2D const &point);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in an AABB.

<span style="color:slategrey">Parameters:</span>

* aabb (AABB) - An unrotated rectangle.
* point (Vec2D) - A point in 2D space.


### <span style="color:darkolivegreen">AABBAndLine</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool AABBAndLine(Primitives::AABB const &aabb, Primitives::Line2D const &line);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a line intersects an AABB.

<span style="color:slategrey">Parameters:</span>

* aabb (AABB) - An unrotated rectangle
* line (Line2D) - A line in 2D space.


### <span style="color:darkolivegreen">AABBAndCircle</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* Circle (Circle) - A circle.


### <span style="color:darkolivegreen">AABBAndAABB</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* Circle (Circle) - A circle.


### <span style="color:darkolivegreen">AABBAndBox2D</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* Circle (Circle) - A circle.


### <span style="color:darkolivegreen">Box2DAndPoint</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* Circle (Circle) - A circle.


### <span style="color:darkolivegreen">Box2DAndLine</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* Circle (Circle) - A circle.


### <span style="color:darkolivegreen">Box2DAndCircle</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* Circle (Circle) - A circle.


### <span style="color:darkolivegreen">Box2DAndAABB</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* Circle (Circle) - A circle.


### <span style="color:darkolivegreen">Box2DAndBox2D</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* Circle (Circle) - A circle.


### <span style="color:darkolivegreen">findCollisionFeatures</span>

<span style="color:slategrey">Function Signature:</span>

```c++
bool PointAndCircle(ZMath::Vec2D const &point, Primitives::Circle const &circle);
```

<span style="color:slategrey">Description:</span>  
&ensp; &ensp; Return a bool representing if a point lies in a circle.

<span style="color:slategrey">Parameters:</span>

* point (Vec2D) - A point in 2D space.
* Circle (Circle) - A circle.
