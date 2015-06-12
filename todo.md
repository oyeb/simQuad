#Animation
* Software switch for tracking
    - intelligent tracking using bounding box. Move axes only when it exits the current view(BBox). Otherwise center camera and hold. When to stop tracking?
    - camera has attains same velocity as `quad.pos`. How about a PD controller for the cam?
* Trajectory plotting in non-tracking mode

##Note!
If tracking is enbled or limits for the axes are set at any point, pan and zoom do not work well. Either,

* submit a bug to MPL -> {?}
* remove the limits temporarily while panning/zooming -> {difficult}
* disable pan and zoom in those cases ->{easy-peasy-japaneasy}

**Anyways**, software tracking can do intuitive pan/zoom if needed via the PD controller.

#GUI
* Embed app into tkinter/wx

#Control
* θ-based PID controller.

#Physics
* Create Physics engine for a quad.
    - Use approximate but ultra-fast linear dynamics.
    - Use non-linear dynamics.
