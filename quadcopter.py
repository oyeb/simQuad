import matplotlib.pyplot as plt
import numpy as np

class copter:
	def __init__(self, axes, track=False):
		#state-vars
		self.track_bool = track #camera tracking
		self.axes = axes #the axis to which "this" quad is connected
		self.sz = .5
		self.pos = np.array([0, 0, 0],np.float) #position of CM
		self.rpy = np.eye(3) #Normalised Roll-Pitch-Yaw (row) vectors
		self.frame = np.array([[1, -1, -1, 1], [1, -1, 1, -1], [0, 0, 0, 0]])
		#graphic-vars
		self.arms = [self.axes.plot([], [], 'o-')[0], self.axes.plot([], [], 'o-')[0]]
		#this is a list of 2 mpl Line2D objects |^^^|--forget these cryptic 0's--|^^^|
		plt.setp(self.arms, 'linewidth', 2.0)

	def init(self):
		#clean quad-figure, no points to plot
		self.arms[0].set_data([], [])
		self.arms[1].set_data([], [])
		return self.arms

	def update_ends(self, num):
		#num is the animation iteration, it's sent by FuncAnimation by default
		if num==0:
			return self.init()
		#Update pos here
		#self.pos[0] = np.cos(num/50.0*np.pi)
		#self.pos[1] = 2*np.cos(num/50.0*np.pi+num/500.0)
		self.pos[2] = np.sin(num/50.0*np.pi)
		#Update rpy here
		#pass

		#finding the actuator positions
		#scale the rpy vectors into forming the frame
		scaled_rpy = self.sz*.5*self.rpy
		#actual end points of the quad below (self.ends)!
		
						#FOR X-mode:
		#self.ends = (self.pos+np.array([scaled_rpy[0]+scaled_rpy[1], -scaled_rpy[0]-scaled_rpy[1], scaled_rpy[1]-scaled_rpy[0], -scaled_rpy[1]+scaled_rpy[0]])).T
		
						#FOR +_mode:
		self.ends = (self.pos+np.array([scaled_rpy[0], -scaled_rpy[0], scaled_rpy[1], -scaled_rpy[1]])).T
		
		#set point data (this will be plotted when func() returns)
		self.arms[0].set_data(np.array([self.ends[0,2:], self.ends[1,2:]]))
		self.arms[0].set_3d_properties(self.ends[2,2:])
		self.arms[1].set_data(np.array([self.ends[0,:2], self.ends[1,:2]]))
		self.arms[1].set_3d_properties(self.ends[2,:2])

		#position the axes, to track the quad
		if self.track_bool:
			world_lims = np.array([self.pos-1, self.pos+1]).T
			#limit the world view to a few units about the current quad position
			# .T is the transpose operator
			self.axes.set_xlim3d(world_lims[0])
			self.axes.set_ylim3d(world_lims[1])
			self.axes.set_zlim3d(world_lims[2])

			#~~~~~~~~~~~~~~~~~danger, which we need!
			plt.draw() #axes don't update unless this line is here
			#above line decreases FPS significantly. Ii guess the loss
			#cannot be helped as we really need the tracking camera
			#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		return self.arms