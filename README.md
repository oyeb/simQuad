# simQuad
simQuad is a lightweight Quadcopter simulation and control algorithm testing program built in python for quick evaluation of control strategies for an (ill-modeled) quadcopter. This program by no means *compensates* for ill-models, it's sole task is to perform reasonable non-linear simulation of Quadcopter dynamics with a control law specified by the programmer and produce an Isometric 3D wireframe animation.

Apart from this, the repo holds attitude estimation software that runs on an arduino. We plan to make a complete Arduino based flight controller.
* Attitude estimation using 6DOF fusion using MPU6050 *(accelerometer, gyroscope)*
* Wireless communication between PC and Quadcopter using XBEE radios.
* Trajectory control.
* Flight Control Ground Station to send commands and view liv status of Quadcopter.

Usage
=====
You need an Arduino Board and an MPU6050. See the wiki on [how to hook up an MPU6050 to an arduino]().
+ Flash `arduino/saber.cpp` onto Arduino Uno or Mega2560 using the Arduino IDE.
+ Launch `ground_station/kalman_binary.py` on the PC. You should see an animated real-time GUI visualising the attitude of MPU6050 chip.
  - Kill the process using `Ctrl` `C` to exit.
  - The `.py` script may have to launched twice incae you do not see the plot.

![Snapshot of kalman_binary.py in action](img/attitude_vis3.png "Snapshot of kalman_binary.py in action")

# Built upon
simQuad uses
* **[matplotlib](http://www.matplotlib.org)**
to perform the 3D plotting, specifically uses the Artist.FuncAnimation() to create a Quadcopter-animation object.
* **DynamicsEngine**
This is just a set of differential equations that approximately model the physics of the quadcopter. We just increment the timestep and generate the position and orientation vectors for the quadcopter. Thanks to Andrew Gibiansky's amazing [blogpost](http://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/).

# Dependencies
* Quadcopter Flight Control Libraries {Install these libraries just as described [here](https://www.arduino.cc/en/Guide/Libraries#toc5)}
    - [mytimer](https://github.com/arrow-/simQuad/tree/master/arduino)
        + This library provides an interrupt service to poll the MPU6050 via TIMER1.
    - [mympu](https://github.com/arrow-/simQuad/tree/master/arduino)
        + This is an interface to the MPU6050 built on the [I2CDevLib](https://github.com/jrowberg/i2cdevlib)
* [wxPython](http://wxpython.org/)
* [matplotlib](http://matplotlib.org/)
* [NumPy](http://www.numpy.org/)
* [Tkinter](https://wiki.python.org/moin/TkInter) is Python's default GUI package, it should be available in the standard Python installation.

@devs install necessary packages via Synaptic package manager on Ubuntu and create another `.md` (markdown) file with tutorials, links, reference/manual pages for these packages.

# Current status
* FuncAnimation works fine, it is fast enough for our simulations.
* Simulation dynamics complete.
* Attitude Estimation using MPU6050 and sensor fusion using a Kalman filter complete.
* Working on wireless comms using XBEE.

# Future work
See [todo.md](https://github.com/arrow-/simQuad/blob/master/todo.md)
* Develop (θ+ω) PID control.
* Develop multiple LTI(Linear Time Invariant) controllers for various behaviours(hover, hold-altitude, hold-pitch/roll, yaw, launch, land, ...).
* Compare PID with LTI.
* Develop hybrid *Sliding-Mode* LTI controller, building on the work of previous step.
* Solve Localisation and Trajectory planning.
  + Explore Octrees, RRT*
  + Compare with Vectorization, Edge n Corner detection, Vector representation os bounded surfaces and plane ordering and intersection with rays.