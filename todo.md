#QuadCopter Project @CSE

##Sensor testing

* ✔ Tested the HMC5883L digital magnetometer.
    - All axes do not have same sensitivity. Requires careful calibration.
    - For now, it is in the cold box.
* ✔ Test the MPU6050
    - ✔ optimize with I2CDevLib. Remove MPU6050.h [All currently used initialisation calls to be converted to I2CDevLib.]
    - ✔ Serial write bytes, no ASCII.
    - ✔ Utilise the onboard DMP for quaternion generation.
    - Devise a better calibration regime, ref: [the guy who makes spheres](http://www.varesano.net/blog/fabio/freeimu-gui-now-making-nice-3d-spheres)
    - ✔ Drop the MPU Interrupt as 5ms is a very fast update interval.
        + ✔ Setup arduino timer for interrupting (adaptively? {normal|debug}).

##Feature Addition

* ✔ Use [XbeeS1](http://www.digi.com/support/productdetail?pid=3430&osvid=0&type=documentation) to make communication with the board, wireless.
  + This is Master(`groud_station`) - Slave(`quadcopter`) only. Slave cannot initiate. *Hopefully, there won't be any need of Asynchronous Peer Protocol*:grin:
* Use `VisPy` for visualisations in `ground_station`.
    + Figure out the axes *(for RPY)* and solve *the `home` position problem*.
* ✔ Use Quaternions for attitude representation.
* ✔ Realised that Kalman filter cannot fuse gyro and accel estimates of attitude as the signal-noise is not *"white" (ie, normally distributed)
    + Instead make EKF that takes the control input into consideration.
* Use [Servo](https://www.arduino.cc/en/Reference/Servo) library for controlling the ESCs.
* Implement a PID controller that controls attitude and linear velocity of the quadcopter.

##Code Restructuring

* ✔ Develop the Radio Transmitter interface.
    + ✔ Can get quaternion if needed.
    + ✔ Compatible with `mpl-vis` and `vispy-vis`
* ✔ Develop the timer interface.
* ✔ Develop the MPU6050 interface
* ✔ Develop the DMP-MPU6050 interface

##Physics

* ✔ Created Physics engine for a quad.
    - Use approximate non-linear dynamics.

##Animation
I have decided to utilise [VisPy](http://vispy.org/) for all plotting work. The 3D plotting is ready but not the `mpl-plots`. I did see a way to subclass `scenegraph` [here](https://github.com/vispy/vispy/issues/1140). If this works, I'll have to decide what to do with the current 3D plotting which is primitive, though extremely robust (because I understand the transforms).

##Other Resources

+ [Varesano's Blog: FreeIMU lib+hardware](http://www.varesano.net/projects/hardware/FreeIMU)
+ [jrowberg's I2CDevLib](https://github.com/jrowberg/i2cdevlib)
+ [Related Literature, *that we collected*](https://drive.google.com/open?id=0By0rvNg9_qznd2NORDUyVGNTVWM)