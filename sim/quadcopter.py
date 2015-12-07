import matplotlib.pyplot as plt
import numpy as np
import time
'''
    r
    |
    |
    |______ p
 . (y)

   +0
-3   -1
   +2

   + counter-clockwise rotation, thus clockwise torque on body! => clock means -
   - clockwise rotation
'''
def rotate(v, axis, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    t = 1-c
    mat = np.array([ [c+axis[0]*axis[0]*t, axis[0]*axis[1]*t-axis[2]*s, axis[0]*axis[2]*t+axis[1]*s],
                     [axis[0]*axis[1]*t+axis[2]*s, c+axis[1]*axis[1]*t, axis[1]*axis[2]*t-axis[0]*s],
                     [axis[0]*axis[2]*t-axis[1]*s, axis[1]*axis[2]*t+axis[0]*s, c+axis[2]*axis[2]*t] ])
    return mat.dot(v.T)

THRUST_CONST = 8.3355675
L = .3
TORQUE_CONST = .6
TORQUE_MAT = np.array([[0, -THRUST_CONST*L, -TORQUE_CONST],[-THRUST_CONST*L, 0, TORQUE_CONST],[0, THRUST_CONST*L, -TORQUE_CONST],[THRUST_CONST*L, 0, TORQUE_CONST]])
I = np.array([[.2, 0, 0],[0, .2, 0],[0, 0, .4]])
I_INV = np.array([[ 5., 0., 0. ], [ 0., 5., 0. ], [ 0., 0., 2.5]])
FRICTION = .04
MASS = 3.4
G = np.array([0,0,-9.80655])
dt = .02

class copter:
	def __init__(self, axes, track=False):
		#state-vars
		self.track_bool = track #camera tracking
		self.axes = axes #the axis to which "this" quad is connected
		self.sz = .5
		self.pos = np.array([0, 0, 0],np.float) #position of CM
		self.posdot = np.zeros(3)
		self.rpy = np.eye(3) #Normalised Roll-Pitch-Yaw (row) vectors
		self.motors = np.ones(4) #motor_i = (omega_i)^2
		self.motors[0] += .1
		self.omega = np.zeros(3)

		#graphic-vars
		self.arms = [self.axes.plot([], [], 'o-')[0], self.axes.plot([], [], 'o-')[0]]
		#this is a list of 2 mpl Line2D objects |^^^|--forget these cryptic 0's--|^^^|
		plt.setp(self.arms, 'linewidth', 2.0)

	def init(self):
		#clean quad-figure, no points to plot
		self.arms[0].set_data([], [])
		self.arms[1].set_data([], [])
		return self.arms

	def simulate(self, control_inputs):
		torque = self.motors.dot(TORQUE_MAT)
		omegadot = (torque-np.cross(self.omega, self.omega.dot(I))).dot(I_INV)
		dthetas = self.omega*dt + omegadot*dt*dt/2
		self.omega += omegadot*dt

		self.rpy[1] = rotate(self.rpy[1], self.rpy[0], dthetas[0])
		self.rpy[0] = rotate(self.rpy[0], self.rpy[1], dthetas[1])
		self.rpy[2] = np.cross(self.rpy[0], self.rpy[1])
		self.rpy[1] = rotate(self.rpy[1], self.rpy[2], dthetas[2])
		self.rpy[0] = rotate(self.rpy[0], self.rpy[2], dthetas[2])

		thrust = THRUST_CONST*np.sum(self.motors)*self.rpy[2]
		acc = G + thrust/MASS - FRICTION*self.posdot
		self.pos = self.posdot*dt + acc*dt*dt/2
		self.posdot += acc*dt
		print acc[2], self.posdot[2],

	def update_ends(self, num):
		st = time.time()
		#num is the animation iteration, it's sent by FuncAnimation by default
		if num==0:
			return self.init()
		self.simulate(None)
		print self.pos
		#Update pos here
		#self.pos[0] = np.cos(num/50.0*np.pi)
		#self.pos[1] = 2*np.cos(num/50.0*np.pi+num/500.0)
		#self.pos[2] = np.sin(num/50.0*np.pi)
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
		print time.time() - st
		return self.arms