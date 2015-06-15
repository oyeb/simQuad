#Animation
I have decided to revisit tracking later and I'm going to focus on other stuff. I tried looking for a way in which axes re-plotting could be faster. Not much luck.
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

#Sensor testing
* ✔ Tested the HMC5883L digital magnetometer.
    - Make vector visualisation.
* Test the MPU6050
* Test MPU6050+HMC5883L (using MPU special mode)

#GUI or UI
* Embed app into tkinter/wx.
* Hook the transmitter to the simulation via Arduino.

#Control
* θ-based PID controller.

#Physics
* ✔ Created Physics engine for a quad.
    - Use approximate non-linear dynamics.
