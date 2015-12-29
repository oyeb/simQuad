simQuad
=======
simQuad is a lightweight Quadcopter simulation and control algorithm testing program built in python for quick evaluation of control strategies for an (ill-modeled) quadcopter. This program by no means *compensates* for ill-models, it's sole task is to perform reasonable non-linear simulation of Quadcopter dynamics with a control law specified by the programmer and produce an Isometric 3D wireframe animation.

Apart from this, the repo holds attitude estimation software that runs on an arduino. We plan to make a complete Arduino based flight controller.

* Attitude estimation using 6DOF fusion using MPU6050 *(accelerometer, gyroscope)*
* Wireless communication between PC and Quadcopter using XBEE radios.
* Trajectory control.
* Flight Control Ground Station to send commands and view live status of Quadcopter.

Usage
=====
You need an Arduino Board and an MPU6050. See the wiki on [how to hook up an MPU6050 to an arduino](http://playground.arduino.cc/Main/MPU-6050).
If you need wireless communications, hook the `RX` and `TX` of the XBee radio into the Arduino's Serial `TX` `Rx` pins. Make sure the XBees are in `AT-transparent` (not in `ZB`) mode *(Just flash or make sure the firmware on both radios is 802.15.4)*.
Find out the serial `portID` of the Arduino or XBee.

Latest
------

+ Flash `arduino/saber2.cpp` onto Arduino Uno or Mega2560 using the Arduino IDE.
+ Launch `ground_station/gs-control.py` as shown, on the PC. You should see an animated real-time GUI visualising the attitude of quadcopter. The darker colored ends were "initially closer to you".
```bash
python gs-control.py <portID>
```
+ Kill the process by typing `quit` *(on the terminal)*. You may not be able to see what you type, that's OK.
	- You can kill the visualisation by closing window or clicking on it.

![Snapshot of gs-control.py in action](https://cloud.githubusercontent.com/assets/ "Snapshot of gs-control.py in action")

For wireless communications, invoke `gs-control.py` like this:
```bash
python gs-control.py --wl <portID>
```

Older Modules
-------------
+ Flash `arduino/saber.cpp` onto Arduino Uno or Mega2560 using the Arduino IDE.
+ Launch `ground_station/kalman_binary.py` as shown, on the PC. You should see an animated real-time GUI visualising the attitude of MPU6050.
+ Kill the process by hitting `Ctrl` `C` *(on the terminal)*.

![Snapshot of kalman_binary.py in action](https://cloud.githubusercontent.com/assets/9897445/11621002/d0c21f32-9cdd-11e5-911d-e7066bb343ef.png "Snapshot of kalman_binary.py in action")

For wireless communications use `kalman_wbinary.py` instead.

Built upon
==========

simQuad uses

* **[matplotlib](http://www.matplotlib.org)**
to perform the 3D plotting, specifically uses the Artist.FuncAnimation() to create a Quadcopter-animation object.
* **DynamicsEngine**
This is just a set of differential equations that approximately model the physics of the quadcopter. We just increment the timestep and generate the position and orientation vectors for the quadcopter. Thanks to Andrew Gibiansky's amazing [blogpost](http://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/).

## Dependencies

* Quadcopter Flight Control Libraries {Install these libraries just as described [here](https://www.arduino.cc/en/Guide/Libraries#toc5)}
    - [mytimer](arduino/mytimer)
        + This library provides an interrupt service to poll the MPU6050 via TIMER1.
    - [mympu](arduino/mympu)
        + This is an interface to the MPU6050 built on the [I2CDevLib](https://github.com/jrowberg/i2cdevlib)
    - [mydmp](arduino/mydmp)
    	+ Mostly scavenged from I2CDevLib to utilise the onboard DMP to retrieve *heading* Quaternion.
* [VisPy](http://www.vispy.org/)
	+ Used for visualisation in 3D. Utilises OpenGL.
* [NumPy](http://www.numpy.org/)
* [Tkinter](https://wiki.python.org/moin/TkInter) is Python's default GUI package, it should be available in the standard Python installation.

@devs install necessary packages via Synaptic package manager on Ubuntu and create another `.md` (markdown) file with tutorials, links, reference/manual pages for these packages.

Current status
==============

* Attitude Estimation using MPU6050 and sensor fusion using a Kalman filter *has been dumped from [74a32d0](https://github.com/arrow-/simQuad/commit/74a32d07fab50dea05abc5d0ef5794d717782bcc) onwards.*
  + An EKF will be implemented insteasd after PID controller has been made.
* Working on PID controller.
* Working on improving [`ground_station`](ground_station)

# Future work
See [todo.md](todo.md)

* Develop (θ+ω) PID control.
* Develop multiple LTI(Linear Time Invariant) controllers for various behaviours(hover, hold-altitude, hold-pitch/roll, yaw, launch, land, ...).
* Compare PID with LTI.
* Solve Localisation and Trajectory planning.
  + Explore Octrees, RRT*
  + Compare with Vectorization, Edge n Corner detection, Vector representation os bounded surfaces and plane ordering and intersection with rays.
