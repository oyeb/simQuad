"""
VISPY Coordinate system is:

vispy_x horizontal (along screen), points to right on screen
vispy_y into screen (from eyes to screen)
vispy_z vertical (along screen), points to top on screen
"""

import numpy as np
from vispy import gloo, app, visuals
import vis_util 
from vispy.util.transforms import perspective, translate, rotate
from vispy.util.quaternion import Quaternion

quad_vert = """
// Uniforms
// ------------------------------------
uniform   mat4 u_model;
uniform   mat4 u_view;
uniform   mat4 u_projection;
uniform   vec4 u_color;

// Attributes
// ------------------------------------
attribute vec3 a_position;
attribute vec3 a_normal;
attribute vec4 a_color;

// Varying
// ------------------------------------
varying vec4 v_color;

void main()
{
    v_color = a_color * u_color;
    gl_Position = u_projection * u_view * u_model * vec4(a_position,1.0);
}
"""


quad_frags = """
// Varying
// ------------------------------------
varying vec4 v_color;

void main()
{
    gl_FragColor = v_color;
}
"""

class Canvas(app.Canvas):

    def __init__(self, width=800, height=600, ns_qstate=None):
        app.Canvas.__init__(self, keys='interactive', size=(width, height))
        self.title = "Heading Visual"
        self.QuadState = ns_qstate

        # make the quadcopter mesh
        self.Quadcopter(nr=12, nc=12, r=[0.1, 0.1], l=2.0)
        # tranform matrices
        self.model = np.eye(4, dtype=np.float32)
        self.projection = np.eye(4, dtype=np.float32)
        self.view = translate((0, 0, -4.0))

        # program for quadcopter mesh
        self.program_quad = gloo.Program(quad_vert, quad_frags)
        self.program_quad['u_model'] = self.model
        self.program_quad['u_view'] = self.view
        self.program_quad.bind(self.vertices_buff)

        # ui elements
        self.timer = app.Timer('auto', connect=self.on_timer, start=True)
        # User turns the visualisation {mouse_button 1 and 2}
        self.mouse_down = [False, False]
        self.ui_beta = self.ui_height = 0

    def Quadcopter(self, nr, nc, r, l):
        quad_frame, colors = vis_util.create_quad_frame(rows=nr, cols=nc, radius=r, length=l, offset=True)
        quad_mesh = vis_util.MyMeshData()
        quad_mesh.set_vertices(quad_frame.get_vertices())
        quad_mesh.set_faces(quad_frame.get_faces())
        quad_mesh.set_vertex_colors(colors)
        vertices, filled, outline = quad_mesh.get_glTriangles()
        # make vertex array, etc
        self.filled_buf = gloo.IndexBuffer(filled)
        self.outline_buf = gloo.IndexBuffer(outline)
        self.vertices_buff = gloo.VertexBuffer(vertices)
        # orientation
        if self.QuadState== None:
            self.orientation_quat = Quaternion(1, 0, 0, 0)

    def on_draw(self, event):
        gloo.clear(color=(1, 1, 1, 1))
        # Filled mesh
        gloo.set_state(blend=False, depth_test=True,
                           polygon_offset_fill=True)
        self.program_quad['u_color'] = 1, 1, 1, 1
        self.program_quad.draw('triangles', self.filled_buf)

        # Outline        
        gloo.set_state(blend=True, depth_test=True,
                       polygon_offset_fill=False)
        gloo.set_depth_mask(False)
        self.program_quad['u_color'] = 0, 0, 1, 1
        self.program_quad.draw('lines', self.outline_buf)
        gloo.set_depth_mask(True)
        

    def on_resize(self, event):
        width, height = event.size
        self.size = event.size
        gloo.set_viewport(0, 0, width, height)
        self.aspect = width / float(height)
        self.projection = perspective(45.0, width / float(height), 2,
                                      10.0)
        self.program_quad['u_projection'] = self.projection

    def on_timer(self, event):
        heading = self.getOrientation()
        displacement = translate((0, -0.5+self.ui_height, 0))
        self.program_quad['u_model'] = np.dot(
                                            heading.get_matrix(),
                                            displacement)
        self.update()

    def on_mouse_press(self, event):
        if event.button == 1:
            self.mouse_down[0] = event.pos
        if event.button == 2:
            self.mouse_down[1] = event.pos
        self.close()

    def on_mouse_release(self, event):
        if event.button == 1:
            self.mouse_down[0] = False
        if event.button == 2:
            self.mouse_down[1] = False

    def on_mouse_move(self, event):
        if event.button == 1 and event.last_event is not None:
            x, y = event.pos
            self.ui_beta = self.mouse_down[0] - x

    def getOrientation(self):
        if self.QuadState == None:
            # if not connected to gs-control.py, being run as stand alone script
            self.orientation_quat = (vis_util.sample_quat(0, 0.5, 0, degrees=True) * self.orientation_quat).normalize()
            #self.orientation_quat = Quaternion(0.9659258262890683, -0.25881904510252074, 0, 0) * self.orientation_quat
            return self.orientation_quat
        else:
            hh = self.QuadState.heading
            gg = Quaternion()
            gg.w = hh.w
            gg.x = hh.z
            gg.y = hh.y
            gg.z = -hh.x
            return gg.normalize()

if __name__ == '__main__':
    c = Canvas(800, 600)
    c.show()
    app.run()