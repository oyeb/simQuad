import matplotlib.pyplot as plt
import numpy as np
import time

class copter:
	def __init__(self, axes):
		self.axes = axes
		self.sz = .5
		self.pos = np.array([0, 0, 0],np.float)
		self.rpy = np.eye(3)
		self.frame = np.array([[1, -1, -1, 1], [1, -1, 1, -1], [0, 0, 0, 0]])
		self.ends = self.sz*self.frame*.5
		self.arms = [self.axes.plot([], [], 'o-')[0], self.axes.plot([], [], 'o-')[0]]
		plt.setp(self.arms, 'linewidth', 2.0)

	def init(self):
		self.arms[0].set_data([], [])
		self.arms[1].set_data([], [])
		return self.arms

	def update_ends(self, num):
		if num==0:
			return self.init()
		#Update pos here
		#self.pos[0] += .5
		self.pos[2] = np.sin(num/50.0*np.pi)
		#Update rpy here
		#pass

		#finding the actuator positions
		scaled_rpy = self.sz*.5*self.rpy
		self.ends = (self.pos+np.array([scaled_rpy[0]+scaled_rpy[1], -scaled_rpy[0]-scaled_rpy[1], scaled_rpy[1]-scaled_rpy[0], -scaled_rpy[1]+scaled_rpy[0]])).T
		#set point data (this willbe plotted when func() returns)
		self.arms[0].set_data(np.array([self.ends[0,2:], self.ends[1,2:]]))
		self.arms[0].set_3d_properties(self.ends[2,2:])
		self.arms[1].set_data(np.array([self.ends[0,:2], self.ends[1,:2]]))
		self.arms[1].set_3d_properties(self.ends[2,:2])

		#position the axes, to track the quad
		world_lims = np.array([self.pos-1, self.pos+1]).T
		self.axes.set_xlim3d(world_lims[0])
		self.axes.set_ylim3d(world_lims[1])
		#self.axes.set_zlim3d(world_lims[2])

		#~~~~~~~~~~~~~~~~~danger, which we need!
		plt.draw() #axes don't update unless this line is here
		#above line decreases FPS significantly. Ii guess the loss
		#cannot be helped as we really need the tracking camera
		#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		return self.arms