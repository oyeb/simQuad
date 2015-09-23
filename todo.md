#QuadCopter Project @CSE

##Sensor testing
* ✔ Tested the HMC5883L digital magnetometer.
    - All axes do not have same sensitivity. Requires careful calibration.
    - For now, it is in the cold box.
* ✔ Test the MPU6050
    - ✔ optimize with I2CDevLib. Remove MPU6050.h [All currently used initialisation calls to be converted to I2CDevLib.]
    - ✔ Serial write bytes, no ASCII.
    - Devise a better calibration regime, ref: [the guy who makes spheres](http://www.varesano.net/blog/fabio/freeimu-gui-now-making-nice-3d-spheres)
    - Drop the MPU Interrupt as 5ms is a very fast update interval.
        + ✔ Setup arduino timer for interrupting (adaptively? {normal|debug}).

##Feature Addition
* Use [XbeeS2](http://www.digi.com/support/productdetail?pid=3430&osvid=0&type=documentation) to make communication with the board wireless.
Port the Kalman filter to cpp headers [@ananth95]
* Port the Kalman filter to cpp headers [@ananth95]
* θ-based PID control
  -  How to map [ω<sub>r</sub> ω<sub>p</sub> ω<sub>y</sub>] to [ω<sub>1</sub> ω<sub>2</sub> ω<sub>3</sub> ω<sub>4</sub>]? Won't that require some *constatnts*?

##Code Restructuring
* ✔ Develop the Radio Transmitter interface.
* ✔ Develop the timer interface.
* Develop the MPU6050 interface [✔ partial]

##Physics
* ✔ Created Physics engine for a quad.
    - Use approximate non-linear dynamics.

##Animation
I have decided to revisit camera tracking later and I'm going to focus on other stuff. I tried looking for a way in which axes re-plotting could be faster. Not much luck.
* Software switch for tracking
    - intelligent tracking using bounding box. Move axes only when it exits the current view(BBox). Otherwise center camera and hold. When to stop tracking?
    - camera has attains same velocity as `quad.pos`. How about a PD controller for the cam?
* Trajectory plotting in non-tracking mode

####Note!
If tracking is enbled or limits for the axes are set at any point, pan and zoom do not work well. Either,

* submit a bug to MPL -> {?}
* remove the limits temporarily while panning/zooming -> {difficult}
* disable pan and zoom in those cases ->{easy-peasy-japaneasy}

**Anyways**, software tracking can do intuitive pan/zoom if needed via the PD controller.

##Other Resources
[Varesano's Blog: FreeIMU lib+hardware](http://www.varesano.net/projects/hardware/FreeIMU)
[jrowberg's I2CDevLib](https://github.com/jrowberg/i2cdevlib)