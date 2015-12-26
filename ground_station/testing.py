import numpy as np
from vispy.geometry import MeshData
from vispy.util.quaternion import Quaternion

def sample_quat(rx, ry, rz, degrees=False):
    """ Classmethod to create a quaternion given the euler angles.
    """
    if degrees:
        rx, ry, rz = np.radians([rx, ry, rz])
    # Obtain quaternions
    qx = Quaternion(np.cos(rx/2), np.sin(rx/2), 0, 0)
    qy = Quaternion(np.cos(ry/2), 0, np.sin(ry/2), 0)
    qz = Quaternion(np.cos(rz/2), 0, 0, np.sin(rz/2))
    # Almost done
    return qx*qy*qz   


class MyMeshData(MeshData):
    """ Add to Meshdata class the capability to export good data for gloo """
    def __init__(self, vertices=None, faces=None, edges=None,
                 vertex_colors=None, face_colors=None):
        MeshData.__init__(self, vertices=None, faces=None, edges=None,
                             vertex_colors=None, face_colors=None)

    def get_glTriangles(self):
        """
        Build vertices for a colored mesh.
            V  is the vertices
            I1 is the indices for a filled mesh (use with GL_TRIANGLES)
            I2 is the indices for an outline mesh (use with GL_LINES)
        """
        vtype = [('a_position', np.float32, 3),
                 ('a_normal', np.float32, 3),
                 ('a_color', np.float32, 4)]
        vertices = self.get_vertices()
        normals = self.get_vertex_normals()
        faces = np.uint32(self.get_faces())

        edges = np.uint32(self.get_edges().reshape((-1)))
        colors = self.get_vertex_colors()

        nbrVerts = vertices.shape[0]
        V = np.zeros(nbrVerts, dtype=vtype)
        V[:]['a_position'] = vertices
        V[:]['a_normal'] = normals
        V[:]['a_color'] = colors

        return V, faces.reshape((-1)), edges.reshape((-1))

def rotate(angle, axis, dtype=np.float32):
    """The 3x3 rotation matrix for rotation about a vector.

    Parameters
    ----------
    angle : float
        The angle of rotation, in degrees.
    axis : ndarray
        The x, y, z coordinates of the axis direction vector.
    """
    angle = np.radians(angle)
    assert len(axis) == 3
    x, y, z = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    cx, cy, cz = (1 - c) * x, (1 - c) * y, (1 - c) * z
    M = np.array([[cx * x + c, cy * x - z * s, cz * x + y * s],
                  [cx * y + z * s, cy * y + c, cz * y - x * s],
                  [cx * z - y * s, cy * z + x * s, cz * z + c]], dtype).T
    return M

def make_quad_arm(rows, cols=8, r_angle=45, radius=[1.0, 1.0], length=1.0, offset=False):
    verts = np.empty((rows+1, cols, 3), dtype=np.float32)
    if isinstance(radius, int):
        radius = [radius, radius]  # convert to list
    # compute vertices
    th = np.linspace(2 * np.pi, 0, cols).reshape(1, cols)
    # radius as a function of z
    r = np.linspace(radius[0], radius[1], num=rows+1,
                    endpoint=True).reshape(rows+1, 1)
    verts[..., 2] = np.linspace(-length/2.0, length/2.0, num=rows+1,
                                endpoint=True).reshape(rows+1, 1)  # z
    if offset:
        # rotate each row by 1/2 column
        th = th + ((np.pi / cols) * np.arange(rows+1).reshape(rows+1, 1))
    verts[..., 0] = r * np.cos(th)  # x = r cos(th)
    verts[..., 1] = r * np.sin(th)  # y = r sin(th)
    # rotate all circles
    m = rotate(r_angle, (0, 1, 0))
    for r in range(0, rows+1):
        for c in range(0, cols):
            verts[r][c] = np.dot(m, verts[r][c])

    # just reshape: no redundant vertices...
    verts = verts.reshape((rows+1)*cols, 3)

    # compute faces
    faces = np.empty((rows*cols*2, 3), dtype=np.uint32)
    rowtemplate1 = (((np.arange(cols).reshape(cols, 1) +
                      np.array([[0, 1, 0]])) % cols) +
                    np.array([[0, 0, cols]]))
    rowtemplate2 = (((np.arange(cols).reshape(cols, 1) +
                      np.array([[0, 1, 1]])) % cols) +
                    np.array([[cols, 0, cols]]))
    for row in range(rows):
        start = row * cols * 2
        faces[start:start+cols] = rowtemplate1 + row * cols
        faces[start+cols:start+(cols*2)] = rowtemplate2 + row * cols
    #colors!
    color = np.resize(np.zeros((rows+1)*cols*4), ((rows+1)*cols, 4))
    c1 = np.array([0, 1, 1, 1])
    c2 = np.array([0.2, 0, 0.2, 1])
    for i in range(0, (rows+1)*cols):
        color[i] = c1 + (c2-c1)*i/((rows+1)*cols-1)
    return (verts, faces, color)

def create_cylinder(rows, cols, r_angle=0, radius=[1.0, 1.0], length=1.0, offset=False):
    """Create a cylinder

    Parameters
    ----------
    rows : int
        Number of rows.
    cols : int
        Number of columns.
    radius : tuple of float
        Cylinder radii.
    length : float
        Length of the cylinder.
    offset : bool
        Rotate each row by half a column.

    Returns
    -------
    cylinder : MeshData
        Vertices and faces computed for a cylindrical surface.
    """
    verts_n_faces = make_quad_arm(rows=rows, cols=cols, r_angle=r_angle, radius=radius, length=length, offset=offset)
    return MeshData(verts_n_faces[0], verts_n_faces[1])

def make_quad_frame(rows, cols=8, radius=[1.0, 1.0], length=1.0, offset=False):
    v1, f1, c1 = make_quad_arm(rows=rows, cols=cols, radius=radius, length=length, offset=offset, r_angle=45);
    v2, f2, c2 = make_quad_arm(rows=rows, cols=cols, radius=radius, length=length, offset=offset, r_angle=-45);
    verts = np.concatenate((v1, v2), axis=0)
    faces = np.concatenate((f1, f2+(rows+1)*cols), axis=0)
    color = np.concatenate((c1, c2), axis=0)
    return (verts, faces, color)

def create_quad_frame(rows, cols=8, radius=[1.0, 1.0], length=1.0, offset=False):
    verts, faces, colors = make_quad_frame(rows=rows, cols=cols, radius=radius, length=length, offset=offset)
    return (MeshData(verts, faces), colors)

if __name__ == '__main__':
    data = make_quad_frame(3, 8, length=2.0)
    print(data[0]) #verts
    print()
    print(data[1]) #frags
    print()
    print(data[2]) #colors