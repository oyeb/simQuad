import matplotlib.pyplot as plt
import time
import numpy as np
import quadcopter
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_zlim(-2,2)
#ax.set_xlim(-1,1)
#ax.set_ylim(-1,1)

quad = quadcopter.copter(ax)

quad_anim = animation.FuncAnimation(fig, quad.update_ends, interval=10, blit=True)
plt.show()