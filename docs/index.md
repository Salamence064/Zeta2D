# Introduction

Welcome to the Zeta2D Physics Engine! Zeta provides a simple solution for modeling 2D physics for anything you'd need. This page gives an overview of the organization of this engine and how to get started. For the purposes of this documentation assume a field is public unless otherwise specified.

___

##<span style="color:fuchsia">Organization</span>
The engine and documentation are split into sections by namespace with each namespace providing a select core functionality. There are 4 namespaces used throughout the engine:  

1. [**ZMath**](https://salamence064.github.io/Zeta2D/zmath/): contains Zeta's custom math library, including many vital math utility functions.
2. [**Primitives**](https://salamence064.github.io/Zeta2D/primitives/): contains all of the primitive shapes supported by Zeta and rigid and static bodies.
3. [**Collisions**](https://salamence064.github.io/Zeta2D/collisions/): contains intersection detection, collision manifolds, and collision manifold calculators.
4. [**Zeta**](https://salamence064.github.io/Zeta2D/zeta/): contains the physics handler used to model physics in your program.
  
Zeta also makes use of preprocessor directives for certain constants and macros. If any of these are already defined, Zeta will fail to compile. Zeta defines the following keywords:

* PI (zmath2D.h)
* EPSILON (zmath2D.h)
* SIGNOF (zmath2D.h)
* TORADIANS (zmath2D.h)
* MIN (zmath2D.h)
* MAX (zmath2D.h)
* FPS_24 (physicshandler.h)
* FPS_30 (physicshandler.h)
* FPS_40 (physicshandler.h)
* FPS_50 (physicshandler.h)
* FPS_60 (physicshandler.h)

If your graphics library defines any of these, go into the corresponding header file and surround it with an ifndef guard. For example:
```c++
#ifndef PI
    #define PI 3.1415926535897932L
#endif
```

___

##<span style="color:fuchsia">Getting Started</span>

Here is a quick snippet to get Zeta2D up and running in your project. There are many more functions and classes than those used here. Information on those will be available throughout this documentation.

```c++
#include <ZETA/physicshandler.h>

int main() {
    // Create your physics handler with the default settings.
    Zeta::handler handler = Zeta::handler();

    // Create the colliders.
    Primitives::Circle* c1 = new Primitives::Circle(ZMath::Vec2D(100.0f, 120.0f), 25.0f);
    Primitives::Circle* c2 = new Primitives::Circle(ZMath::Vec2D(200.0f, 240.0f), 12.0f);

    // Create some rigid bodies to pass to the handler.
    Primitives::RigidBody2D rb1(
        c1->c,                             // centerpoint
        50.0f,                             // mass
        0.9f,                              // coefficient of restitution
        0.975f,                            // linear damping
        Primitives::RIGID_CIRCLE_COLLIDER, // collider type
        c1                                 // collider (gets deleted by the constructor)
    );

    Primitives::RigidBody2D rb2(
        c2->c,                           // centerpoint
        20.0f,                             // mass
        0.95f,                             // coefficient of restitution
        0.8f,                              // linear damping
        Primitives::RIGID_CIRCLE_COLLIDER, // collider type
        c2                                 // collider (gets deleted by the constructor)
    );

    // Add the rigid bodies to the handler.
    handler.addRigidBody(&rb1);
    handler.addRigidBody(&rb2);

    // Program's dt loop.
    float dt = 0.0f;

    // Note: windowShouldNotClose should be replaced with the exit window condition in your graphics library.
    while (windowShouldNotClose) {
        /* Rendering/Drawing code should go here */

        handler.update(&dt); // The handler will subtract from dt for you.
        // Note: getEllapsedTime() should be replaced with the equivalent function in your graphics library.
        dt += getEllapsedTime(); 
    }

    return 0;
};
```

___

##<span style="color:fuchsia">Contributing and Bugs</span>
If you encounter a bug while using the engine, open it as an issue on [Zeta's github page](https://github.com/Salamence064/Zeta2D), and it will be fixed as soon as possible.  
If you wish to contribute to this physics engine, check out our contribution guidlines to learn more.
