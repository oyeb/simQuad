# simQuad
simQuad is a lightweight Quadcopter simulation and control algorithm testing program built in python for quick evaluation of control strategies for an (ill-modeled) quadcopter. This program by no means *compensates* for ill-models, it's sole task is to perform reasonable non-linear simulation of Quadcopter dynamics with a control law specified by the programmer and produce an Isometric 3D wireframe animation.

# Policy
Major project decisions need to be well documented, that is, reasoned. Documentation need not be actual "e-documents", but video demos, pictures, screenshots, pictures of hand-written text, audio(if you have a sweet voice), etc.

This is primarily to create a formal history that can be not only passed on to project interns or new (noob junior) project members.
It also serves a noble secondary aim, it honestly tracks the progress of the project and forms a basis to evaluate efficiency of group members.
Thirdly, your document is a formal contribution to the project (a single doc can have multiple authors).

Such files need to be archived chronologically into a
* web-format (html or whatever). A chronological index page with links to "documents".
   - Need to discuss with Sourav Jaiswal on this.
* any better format?

{@Ananya: Where is your doc? You've made a zillion decisions already.}

# Built upon
simQuad uses
* **[matplotlib](http://www.matplotlib.org)**
to perform the 3D plotting, specifically uses the Artist.FuncAnimation() to create a Quadcopter-animation object.
* **DynamicsEngine**
Dai! we gotta search for this. few will have extreme details, we have to choose appropriate one. Remember the "Quadcopter Dynamics, Conrol, simulation.pdf"? That's probably what we are looking for.

# Dependencies
* [wxPython](http://wxpython.org/)
* [matplotlib](http://matplotlib.org/)
* [NumPy](http://www.numpy.org/)
* [Tkinter](https://wiki.python.org/moin/TkInter) is Python's default GUI package, it should be available in the standard Python installation.

@devs install necessary packages via Synaptic package manager on Ubuntu and create another `.md` (markdown) file with tutorials, links, reference/manual pages for these packages.

{@Ananth,Aditya: Split this job among yourselves, you two gotta read about all these anyways}.

{@Aditya: Add instructions for Windows here, if you will work via Windows}.

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

