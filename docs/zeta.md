# Zeta

This section covers the Zeta namespace. Zeta contains all the code related to the physics handler you'll use to simulate physics using Zeta2D. All the code inside this namespace can be found in physicshandler.h.

___

## <span style="color:fuchsia">Classes</span>
This subsection discusses the classes contained in Zeta. There's only a single class declared in the namespace and details on it are provided here.

### <span style="color:darkolivegreen">Handler</span>
This class will simulate physics for your program. In other words, for the rigid bodies you pass to this handler, collisions will be resolved and their physics attributes will be updated. You should store the pointers to the rigid bodies in a list in your main program, too, so you can use them for graphical purposes. Below are the core fields, constructors, and functions.

#### <span style="color:steelblue">Fields</span>
| <span style="color:slategrey">Type</span> | <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----:|:----------:|:-----------:|
| <span style="color:deeppink">private</span> <span style="color:hotpink">struct</span> | <span style="color:seagreen">rbs</span> | A struct storing the list of rigid bodies the engine will update. |
| <span style="color:deeppink">private</span> <span style="color:hotpink">struct</span> | <span style="color:seagreen">colWrapper</span> | A struct storing colliding rigid bodies and their collision manifolds. This is used for impulse resolution. |
| <span style="color:deeppink">private</span> <span style="color:hotpink">float</span> | <span style="color:seagreen">updateStep</span> | The amount of time, in seconds, to perform a physics update after. |
| <span style="color:hotpink">Vec2D</span> | <span style="color:seagreen">g</span> | The acceleration due to gravity as a vector. |

#### <span style="color:steelblue">Constructor</span>
<span style="color:slategrey">Description:</span>  

* Create a physics handler with a specific gravity and update time step.  
  
<span style="color:slategrey">Parameters:</span>

* g (Vec2D) - The acceleration due to gravity. Default of <0, -9.8f>.
* timeStep (float) - The amount of time, in seconds, to update physics after. Default of 0.0167f (equivalent to 60FPS).

```c++
Handler(ZMath::Vec2D const &g = ZMath::Vec2D(0, -9.8f), float timeStep = FPS_60);
```


#### <span style="color:steelblue">Functions</span>
<span style="color:slategrey">Function Signature:</span>

```c++
int update(float &dt);
```

<span style="color:slategrey">Description:</span>  

* Update the rigid bodies managed by this physics handler according to a dt value. The dt value passed in will be subtracted from by the handler. This returns an int equal to the number of times the physics was updated this update call. This way, the user can update anything that depends on the number of times the physics were updated properly. 

<span style="color:slategrey">Parameters:</span>

* dt (float) - The time ellapsed since the last time this was called. Dt should be added to in your main program instead of replaced for determinism purposes. Zeta2D will subtract from dt for you after each time it runs.


<span style="color:slategrey">Function Signature:</span>

```c++
void addRigidBody(Primitives::RigidBody2D* rb);
```

<span style="color:slategrey">Description:</span>  

* Add a rigid body to the handler. The rigid body is passed as a pointer so your graphics program can store and draw the rigid bodies, too, without any extra function calls.

<span style="color:slategrey">Parameters:</span>

* rb (RigidBody2D*) - A pointer to the rigid body being added to the handler.


<span style="color:slategrey">Function Signature:</span>

```c++
bool removeRigidBody(Primitives::RigidBody2D* rb);
```

<span style="color:slategrey">Description:</span>  

* Remove a rigid body from the handler based on its pointer. The data pointed to will be deleted by this function. A bool is returned indicating if the rigidbody was found and removed or not.

<span style="color:slategrey">Parameters:</span>

* rb (RigidBody2D*) - A pointer to the rigid body being removed from the handler.

___

## <span style="color:fuchsia">Functions</span>
This subsection touches on the functions available in the Zeta namespace. There's only a single function defined in this namespace and it is not recommended you call it in your main program. Details on it can still be found below on the offchance you need to use it.

### <span style="color:darkolivegreen">Handler</span>
<span style="color:slategrey">Function Signature:</span>

```c++
void applyImpulse(Primitives::RigidBody2D* rb1, Primitives::RigidBody2D* rb2, Collisions::CollisionManifold const &manifold);
```

<span style="color:slategrey">Description:</span>

* Apply impulse to two colliding rigid bodies. The physics handler will run this function for you. It is not recommended for you to call this in your main program.

<span style="color:slategrey">Parameters:</span>

* rb1 (RigidBody2D*) - A pointer to a colliding rigid body.
* rb2 (RigidBody2D*) - A pointer to the other colliding rigid body.
* manifold (CollisionManifold) - A struct containing the data about the collision.

___

## <span style="color:fuchsia">Constants</span>
This subsection talks about useful constants declared in the physicshandler.h header. These constants are all related to common framerates to make it easier to set the handler to a chosen framerate. The intended use for all of them is to pass as the timeStep argument for the physics handler's constructor. The name of each indicates the physics framerate it will set the handler to.

| <span style="color:slategrey">Identifier</span> | <span style="color:slategrey">Description</span> |
|:----------:|:-----------:|
| <span style="color:hotpink">FPS_24</span> | Tells the handler to update 24 times per second. |
| <span style="color:hotpink">FPS_30</span> | Tells the handler to update 30 times per second. |
| <span style="color:hotpink">FPS_40</span> | Tells the handler to update 40 times per second. |
| <span style="color:hotpink">FPS_50</span> | Tells the handler to update 50 times per second. |
| <span style="color:hotpink">FPS_60</span> | Tells the handler to update 60 times per second. |
