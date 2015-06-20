'''
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
IMPORTANT!!

This code is compatible only with mpu_magnet.ino for Obvious Reasons.

Basically this script EXPECTS:
Arduino is providing:
	- Gyro         ~5ms interval via MPU Interrupt
	- Magnetometer ~13.3ms interval via HMC5883L Interrupt
* Each serial packet must be ASCII and look like:
		[x_gyro]<space>[y_gyro]<space>[z_gyro]<newline> {gyro}
		[x_magField]<space>[y_magField]<space>[z_magField]['b']<newline> {magnetometer}
	The character 'b' immediately follows the z-field to differentiate between Gyro n Mag
+ You need to do standard settings. Refer gyro_scope.py docstring.

And it Delivers:
* 3 axis loss-less Gyro readings plot (almost real time).
* 3D visualisation of current orientation based on gyro vals
* Print field_vector/its magnitude

			~~~~~~~~~~~~~~~~~~~~ NOTE ~~~~~~~~~~~~~~~~~~~~~~
			Mag readings are not uniform over the axes. If 
			you simply rotate (no translation), magnitude 
			will still change a lot.
			With respect to translation, indoor env. sucks, 
			as uniform regions are too small.
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
'''

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import serial, time

def Norm(v):
	'''
	Normalizes (computes unit vector) and returns:
	tuple(unit_vector{np.array object}, magnitude)
	'''
	mag = (v[0]**2+v[1]**2+v[2]**2)**0.5
	return (v/mag, mag)

def rotate(v, axis, theta):
	'''
	Rotates "v" vector about "axis" vector by "theta" degrees
	'''
	c = np.cos(theta)
	s = np.sin(theta)
	t = 1-c
	mat = np.array([ [c+axis[0]*axis[0]*t, axis[0]*axis[1]*t-axis[2]*s, axis[0]*axis[2]*t+axis[1]*s],
					 [axis[0]*axis[1]*t+axis[2]*s, c+axis[1]*axis[1]*t, axis[1]*axis[2]*t-axis[0]*s],
					 [axis[0]*axis[2]*t-axis[1]*s, axis[1]*axis[2]*t+axis[0]*s, c+axis[2]*axis[2]*t] ])
	return mat.dot(v.T)

def calcPose(omega):
	'''
	Helper function. Finds the "d"-theta, then calls rotate.
	'''
	theta = omega*dt
	rpy[1] = rotate(rpy[1], rpy[0], theta[0])
	rpy[0] = rotate(rpy[0], rpy[1], theta[1])
	rpy[2] = np.cross(rpy[0], rpy[1])
	rpy[1] = rotate(rpy[1], rpy[2], theta[2])
	rpy[0] = rotate(rpy[0], rpy[2], theta[2])

plt.ion()
# SET CORRECT PORT NUM HERE
arduino = serial.Serial('/dev/ttyACM0', 57600)
# dt is found experimentally. Contact Ananya for details. Basically this the time between
# 2 MPU(gyro) interrupts. The np.pi/180 converts deg/sec to rad/sec.
# SET CORRECT dt HERE
dt = .005*np.pi/180 #8.726646259971648e-05
# rpy is original orientation. These vectors are updated by calcPose()
rpy = np.eye(3)

fig = plt.figure(figsize=(16,6))
axes = fig.add_subplot(121)
a3d = fig.add_subplot(122, projection='3d')
a3d.set_xlim(-1.2,1.2)
a3d.set_ylim(-1.2,1.2)
a3d.set_zlim(-1.2,1.2)
a3d.scatter([0], [0], [0], s=40)
r, = a3d.plot([0,1], [0,0], [0,0], lw=2, c='black')
p, = a3d.plot([0,0], [0,1], [0,0], lw=2, c='red')
a3d.plot([0,2], [0,0], [0,0], c='cyan')
a3d.plot([0,0], [0,2], [0,0], c='brown')
a3d.plot([0,0], [0,0], [0,2], c='green')
a3d.plot([0,-2], [0,0], [0,0], ls='--', c='cyan')
a3d.plot([0,0], [0,-2], [0,0], ls='--', c='brown')
a3d.plot([0,0], [0,0], [0,-2], ls='--', c='green')

num_samples = 0
buff = 0
# "buff" counts till 50. Every time it reaches fifty, plt.draw() is called, since
# plt.draw() is a costly operation. Normal list append and pose calculations are fast.
# So, do those diligently, for every sample, but update display 
# rarely (while ensuring smooth animation).
gyro_x = [0]
gyro_y = [0] # gyro data lists. I use them like queues.
gyro_z = [0]

old_B = None # "previous" B vector. (magnetometer)
t = [0] # X axis data array (time)
# scopes is a list of 3 matplotlib.Line_2D objects.
scopes = [axes.plot(t, gyro_x, label=r'$\omega_x$')[0], axes.plot(t, gyro_y, label=r'$\omega_y$')[0], axes.plot(t, gyro_z, label=r'$\omega_z$')[0]]
axes.legend(prop=dict(size=14))
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=3, mode="expand", borderaxespad=0.)
axes.set_ylim(-505, 505) # Limits according to range of input signal

# Refer datasheet. Convert ADC result into a Physical measurement.
# If you don't understand this, pls. leave project.
g_conversion = 65.5 #Gyro 500
m_conversion = 820/100.0 # Mode 2 (3rd) +-1.9G, the 100 is just to scale things up.
# If I don't keep it, the true Gauss levels are used which are in the order of 10mG.
# Who cares about the "real" value? Why to push into the milli (floating point error)
# domain? This can avoid truncation errors.
MAG_TRUST_THRESHOLD = 50
mag_flag = gyro_flag = False

while True:
	try:
		datas = arduino.readline().strip()
	except:
		print 'Serial error!'
		raise RuntimeError
	if datas[-1] == 'b':
		datas = datas[:-1]
		mag_flag = True
	else:
		gyro_flag = True
	datas = datas.split(' ')
	if len(datas) == 3:
		if (gyro_flag): # If this packet is gyro data => update scopes, etc.
			datas = np.array([float(datas[0])/g_conversion, float(datas[1])/g_conversion, float(datas[2])/g_conversion])
			gyro_x.append(datas[0])
			gyro_y.append(datas[1])
			gyro_z.append(datas[2])
			num_samples += 1
			t.append(num_samples)
			calcPose(datas) #This function updates the global variable: "rpy"
			if num_samples>200:
				del t[0]
				del gyro_x[0]
				del gyro_y[0]
				del gyro_z[0]
			axes.set_xlim(t[0], num_samples)
			scopes[0].set_data(t, gyro_x)
			scopes[1].set_data(t, gyro_y)
			scopes[2].set_data(t, gyro_z)
			# pose matrix is just an easier way of giving input to the .set_data()
			# and .set_3d_properties() methods. You see, line needs 2 (end) points:
			# the rpy entries AND THE ORIGIN. pose matrix does just that: specifies
			# BOTH end points. 
			pose = np.array([np.array([np.zeros(3), rpy[0]]).T,	np.array([np.zeros(3), rpy[1]]).T, np.array([np.zeros(3), rpy[2]]).T])
			r.set_data(pose[0][:2])
			r.set_3d_properties(pose[0][2])
			p.set_data(pose[1][:2])
			p.set_3d_properties(pose[1][2])
			gyro_flag = False
		
		if (mag_flag): # If this packet is compass data => update orientation info
			datas = np.array([float(datas[0])/m_conversion, float(datas[1])/m_conversion, float(datas[2])/m_conversion])
			if old_B is not None:
				print old_B[1]
				new_B = Norm(datas)
				# use the new vector only if magnitude didn't change much. Else give it less importance (weight)
				if abs(new_B[1] - old_B[1]) < MAG_TRUST_THRESHOLD:
					# Basic algebra to find the rotation axis and amount
					axis = np.cross(old_B[0], new_B[0])
					theta = np.arccos(np.dot(old_B[0], new_B[0]))
					# Now, update the orientation vectors! Job done!
				else:
					# We entered a different magnetic region!! Wait for atleast one iteration
					# Of course, in the next iteration, old_B.mag ~= new_B'.mag, got it?
					# But, is 1 iteration enough? Should we wait more? how much and why?
					# Do you have experimental proof or convincing data?
					# Think of a novel, safe, easy, sure shot approach and experiment to solve this problem
					# Of course, this data need not be collected via flying the quad, walking around to 
					# collect data is recommended.
					pass
			old_B = Norm(datas) #Norm returns unit vector and magnitude as tuple
			mag_flag = False
			
		if buff>50:
			buff=0
			plt.draw()
		buff += 1

plt.ioff()
plt.show()