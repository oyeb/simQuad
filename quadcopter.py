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
		self.arms = [self.axes.plot(self.ends[0,2:], self.ends[1,2:], self.ends[2,2:], c='k', lw=2)[0], self.axes.plot(self.ends[0,:2], self.ends[1,:2], self.ends[2,:2], c='k', lw=2)[0]]
		self.motors = None

	def update_ends(self, num):
		#Update pos here
		self.pos[2] = np.sin(num/50.0*np.pi)
		#Update rpy here
		pass

		#finding the actuator positions
		scaled_rpy = self.sz*.5*self.rpy
		self.ends = (self.pos+np.array([scaled_rpy[0]+scaled_rpy[1], -scaled_rpy[0]-scaled_rpy[1], scaled_rpy[1]-scaled_rpy[0], -scaled_rpy[1]+scaled_rpy[0]])).T
		return self.draw()

	def draw(self):
		#if self.motors is not None:
		#	self.motors.remove()
		#self.motors = self.axes.scatter(self.ends[0], self.ends[1], self.ends[2], label='3 points', c='brrb', s=100)
		self.arms[0].set_data(np.array([self.ends[0,2:], self.ends[1,2:]]))
		self.arms[0].set_3d_properties(self.ends[2,2:])
		self.arms[1].set_data(np.array([self.ends[0,:2], self.ends[1,:2]]))
		self.arms[1].set_3d_properties(self.ends[2,:2])
		return self.arms