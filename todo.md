#Animation
* Software switch for tracking
    - intelligent tracking using bounding box. Move axes only when it exits the current view(BBox). Otherwise center camera and hold. When to stop tracking?
    - camera has attains same velocity as `quad.pos`. How about a PD controller for the cam?

#GUI
* Embed app into tkinter/wx

#Control
* θ-based PID controller.

#Physics
* Create Physics engine for a quad.
    - Use approximate but ultra-fast linear dynamics.
    - Use non-linear dynamics.
