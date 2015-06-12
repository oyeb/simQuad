import matplotlib.pyplot as plt
import time
import numpy as np
import quadcopter
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
#don't try to understand these imports

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#	       NP.ARRAYS are NOT Matrices.            #
# Always print your array operation results to    #
# check result with expected dimensionality and   #
# values.                                         #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

#create the figure object, this will hold subplots, which will hold (multiple) axes.
fig = plt.figure()
#add an axis to first subplot (111) of the fig-object
ax = fig.add_subplot(111, projection='3d')
#set limits. Refer the todo.md for a cautionary nnote on limits
#IF LIMITS SET, THEN PAN AND ZOOM FAIL.
ax.set_xlim3d(-1.3,1.3)
ax.set_ylim3d(-1.3,1.3)
ax.set_zlim3d(-1.3,1.3)

quad = quadcopter.copter(ax, False) #false => NO tracking
#make the animation object
quad_anim = animation.FuncAnimation(fig, quad.update_ends, interval=15, blit=True)
plt.show()