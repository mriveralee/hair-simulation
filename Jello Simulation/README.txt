-------README.TXT---------------

Name: Michael Rivera
PennKey: rimic
HW: Jello Simulation (HW-1)

---------------------------------
*******Question Responses********

1.) What is the effect of the Ks and Kd parameters on the jello? 
The Ks and Kd parameters affect the spring forces and thereby the behavior of the jello. The Ks value refers to the strength of the spring. The higher a value of Ks, the more difficult it is to stretch that particular spring. The lower the value of Ks, the easier is it to stretch the spring. In the jello, this values are particularly meaningful because it effects the deformation of the jello cube. Higher values made the jello less jiggly and more stable. If you decrease the value of Ks for the springs the jello because very jiggly.

The Kd value is used to apply a force to a spring to damp its motion.  This constant serves the purpose of demonstrating the loss of energy that a spring will have in its motion. Higher damping constants reduce the oscillation of the spring, meaning it moves less.


2.) What are the benefits and the drawbacks of the collision system used here? What are some different ways in which it could be improved?

The benefits of this collision system are 

The drawbacks are that system is limited by the time steps we can take while performing integration. Small times steps help preserve the system;however, large time steps will cause the jello to explode and the system to function incorrectly. In addition, the small time steps require the system to perform more computation. Likewise, our time step determines our stability and not the accuracy of representation.


4.) From lecture, What is an example of a stiff constrained system?
An example of a  Stiff constrained system is: y(t) = e^(-2.3t)

5.) From lecture, What is the difference between and explicit and implicit integration scheme? Does the jello behave realistically? 

In the explicit integration scheme, we use the current values of the particle at time t to estimate the values at the next time step. In implicit integration 



Documentation:

--OVERALL
I have implemented all portions of the assignment that were required.  As an extra feature, I Implemented sphere collision detection. Additionally, I add a friction force for floor collisions and added additional bending springs between particles that are 3 and 4 particles away.


---SCENES
There are also a number of xml scenes that I have constructed. The scenes are as follows:
- coolScene - contains all objects and checks all collisions
- sphere - tests the sphere collisions
- cylinderBottom - tests intersection on the cylinder end caps
- myCylinder2 - tests side intersection with a cylinder

The default scene is 'coolScene' which contains both cylinder, floor, and sphere collisions.

----INTERSECTIONS
*Sphere, Floor, and Cylinder Intersections work

For contacts, I reflected the velocity (as in done in ray tracing) and applied a coefficient of restitution to slow it down, then move the particle to the surface. For collisions I apply a spring to pull the particle away from the colliding surface. Friction forces are added in both cases for floorIntersections.


----INTEGRATION METHODS
I've implemented Euler and Midpoint as integration methods. The code is code to the specifications/in terms of performing these methods; however, I was not able to find spring constants to satisfy the stability requirement with the preset time set. When switching to this, the number of forces of the cube cause it to explode. If the cube is not interacting with the environment, Euler and Midpoint work as they should

----VIDEOS
There are two video files demonstrating unique scenes and my jello simulation. One is of the top cap on a cylinder. The other is bouncing the cube off of multiple objects in a single scene.


