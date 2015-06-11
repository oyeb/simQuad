# simQuad
simQuad is a lightweight Quadcopter simulation and control algorithm testing program built in python for quick evaluation of control strategies for an (ill-modeled) quadcopter. This program by no means *compensates* for ill-models, it's sole task is to perform reasonable non-linear simulation of Quadcopter dynamics with a control law specified by the programmer and produce an Isometric 3D wireframe animation.

# Built upon
simQuad uses
* **[matplotlib](http://www.matplotlib.org)**
to perform the 3D plotting, specifically uses the Artist.FuncAnimation() to create a Quadcopter-animation object.
* **DynamicsEngine**
Dai! we gotta search for this. few will have extreme details, we have to choose appropriate one. Remember the "Quadcopter Dynamics, Conrol, simulation.pdf"? That's probably what we are looking for.

# Current status
* Testing the FuncAnimation() with *blitting* and UI widgets like sliders or key-presses under matplotlib-io framework. If that is unsatisfactory, we need to embed matplotlib into a **Tkinter** / **wx** / Qt app to provide ultimate UI.
* Developing (θ-based)PID based control strategy and implementation in python and C++ (for the arduino).

# Future work
* Develop ω-based PID control (simpler than θ-based).
* Develop (θ+ω) hybrid PID control.
* Develop multiple LTI(Linear Time Invariant) controllers for various behaviours(hover, hold-altitude, hold-pitch/roll, yaw, launch, land, ...).
* Compare PID with LTI.
* Develop hybrid *Sliding-Mode* LTI controller, building on the work of previous step.
* Develop non-Linear Time Invariant (nLTI) controllers. {not in Coursera video list}
* Solve Localisation and Trajectory planning.

