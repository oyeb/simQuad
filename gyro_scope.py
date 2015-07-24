'''
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
IMPORTANT!!

It is suggested you run this script with mpu_level2.ino first to see and understand 
its operation.
Basically this script EXPECTS:
Arduino is providing space separated gyro readings @ ~5ms intervals (via MPU Interrupt).
* Each serial packet must be ASCII and look like:
		[x_gyro]<space>[y_gyro]<space>[z_gyro]<newline>
+ You need to specify correct Serial port
+ You need to set the Y-limits of the plot axis.
+ You need to use correct value of "dt".
+ You need to set the correct conversion factor for Gyro readings.
  Mode  0		1		2		3
  Range +-250	+-500	+-1000	+-2000
  Conv. 131		65.5	32.75	16.375

AND it DELIVERS:
* 3 axis loss-less Gyro readings plot (almost real time).
* 3D visualisation of current orientation based on gyro vals

If you want to just plot data in ~real time use {oscilloscope.py}.
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
'''
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import serial, time

def rotate(v, axis, theta):
	'''
	Rotates "v" vector about "axis" vector by "theta" radians, returns vector
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
	Omega must be in ** degrees per second **
	'''
	theta = omega*dt*np.pi/180 #theta is "d-theta" in radians
	rpy[1] = rotate(rpy[1], rpy[0], theta[0])
	rpy[0] = rotate(rpy[0], rpy[1], theta[1])
	rpy[2] = np.cross(rpy[0], rpy[1])
	rpy[1] = rotate(rpy[1], rpy[2], theta[2])
	rpy[0] = rotate(rpy[0], rpy[2], theta[2])

plt.ion()
# SET CORRECT PORT NUM HERE
arduino = serial.Serial('/dev/ttyACM4', 57600)
# dt is found experimentally. Contact Ananya for details. Basically this the time between
# 2 MPU(gyro) interrupts. The np.pi/180 converts deg/sec to rad/sec.
# SET CORRECT dt HERE. TIME IN SECONDS BETWEEN TWO SENSOR PACKETS AS RECVD. BY ARDUINO.
dt = .005 # 5msec
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
t = [0]
# scopes is a list of 3 matplotlib.Line_2D objects.
scopes = [axes.plot(t, gyro_x, label=r'$\omega_x$')[0], axes.plot(t, gyro_y, label=r'$\omega_y$')[0], axes.plot(t, gyro_z, label=r'$\omega_z$')[0]]
axes.legend(prop=dict(size=14))
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=3, mode="expand", borderaxespad=0.)
axes.set_ylim(-505, 505) # SET CORRECT Y-LIM HERE
conversion = 65.5 #Gyro 500 SET CORRECT CONV FACTOR HERE

# Refer datasheet. Convert ADC result into a Physical measurement.
# If you don't understand this, pls. leave project.
print 'Me Ready'
time.sleep(2)
#Handshake MAY BE REDUNDANT
print arduino.inWaiting()
arduino.flushInput()
arduino.write('e')
print 'Sent Request...'
data = [0]*6
while True:
	try:
		num = arduino.read(12)
		num = [ord(x) for x in num]
	except:
		print 'Serial error!'
		raise RuntimeError
	_ind=0 #this var is connected to for loop below!!
	for i in range(0,12, 2):
		data[_ind] = (num[i]<<8)|num[i+1]
		if data[_ind] & 0x8000:
			data[_ind] = data[_ind] - 0x10000
		_ind += 1
	#print data[3:]
	datas = np.array([float(data[3])/conversion, float(data[4])/conversion, float(data[5])/conversion])
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

<<<<<<< HEAD
	if buff>50:
=======
	if buff>15:
>>>>>>> 6650e80dd0f8a8f779a2bb2ab7d20fd2aa3ecc83
		buff=0
		plt.draw()
	buff += 1

plt.ioff()
plt.show()