'''
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
This file visualises the attitude of the MPU6050 chip.

COMPATIBLE WITH:
+ ~/arduino/saber.cpp
+ ~/arduino/mpu_reflowed.cpp {CAL_DEBUG shoud NOT be defined}

YOU MUST ENSURE:
* In .cpp files, timer_init() must be called with same TIME_INTERVAL as specified below!
* Each serial packets are 6 bytes of binary data
* You need to specify correct Serial port
* You need to set the Y-limits of the plot axis.
* You need to set the correct conversion factor for Gyro readings.
  Mode  0		   1		  2		    3
  Range +-250	 +-500	+-1000	+-2000
  Conv. 131		 65.5	  32.75	  16.375

AND it DELIVERS:
* 3 axis loss-less Gyro readings plot (almost real time).
* 3D visualisation of current orientation based on gyro vals
   .
  / \  You might have to launch this script repeatedly if no data is transmitted by the arduino
 / ! \ If problem persists, flash the .cpp once more and the repeat.
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
'''
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import serial, time, at_talk
TIME_INTERVAL = 0.005


def rotate(v, axis, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    t = 1-c
    mat = np.array([ [c+axis[0]*axis[0]*t, axis[0]*axis[1]*t-axis[2]*s, axis[0]*axis[2]*t+axis[1]*s],
                     [axis[0]*axis[1]*t+axis[2]*s, c+axis[1]*axis[1]*t, axis[1]*axis[2]*t-axis[0]*s],
                     [axis[0]*axis[2]*t-axis[1]*s, axis[1]*axis[2]*t+axis[0]*s, c+axis[2]*axis[2]*t] ])
    return mat.dot(v.T)

def calcPose(omega):
	dt = TIME_INTERVAL*np.pi/180.0
	dtheta  = omega*dt
	rpy[1] = rotate(rpy[1], rpy[0], dtheta[0])
	rpy[0] = rotate(rpy[0], rpy[1], dtheta[1])
	rpy[2] = np.cross(rpy[0], rpy[1])
	rpy[1] = rotate(rpy[1], rpy[2], dtheta[2])
	rpy[0] = rotate(rpy[0], rpy[2], dtheta[2])

def calcAngle(accel):
	'''
	angle = np.zeros(3)
	angle[0]  = np.arctan(accel[1]/np.sqrt((accel[0]*accel[0])+(accel[2]*accel[2])))+1.5707963267948966
	angle[1]  = np.arctan(accel[0]/np.sqrt((accel[1]*accel[1])+(accel[2]*accel[2])))+1.5707963267948966
	angle[2]  = np.arctan(accel[2]/np.sqrt((accel[1]*accel[1])+(accel[0]*accel[0])))+1.5707963267948966
	'''
	angle = np.array([
					np.arctan(accel[0]/np.sqrt(accel[1]**2+accel[2]**2))+1.5707963267948966,
					np.arctan(accel[1]/np.sqrt(accel[0]**2+accel[2]**2))+1.5707963267948966,
					np.arctan(accel[2]/np.sqrt(accel[1]**2+accel[0]**2))+1.5707963267948966,
					])
	return angle
	
def variance(readings):
	sum1 = np.zeros(3)
	sum2 = np.zeros(3)
	i=0
	while i<10:
		sum1 += readings[i]
		sum2 += readings[i]*readings[i]
		i+=1
	num = (sum1*sum1)/10
	sd 	= (sum2-num)/9
	var = np.sqrt(sd)
	return var


plt.ion()
arduino = at_talk.radio('/dev/ttyACM0', 57600)
rpy = np.eye(3)

fig = plt.figure(figsize=(16,6))
axes = fig.add_subplot(121)
#axesacc = fig.add_subplot(122)
a3d = fig.add_subplot(122, projection='3d')
#a3dacc = fig.add_subplot(224, projection='3d')

a3d.set_xlim(-1.2,1.2)
a3d.set_ylim(-1.2,1.2)
a3d.set_zlim(-1.2,1.2)
a3d.scatter([0], [0], [0], s=40)
r, = a3d.plot([0,1], [0,0], [0,0], lw=2)
p, = a3d.plot([0,0], [0,1], [0,0], lw=2)
y, = a3d.plot([0,0], [0,0], [0,1], lw=2)
gyaw, = a3d.plot([0,0], [0,0], [0,1], lw=2)
a3d.plot([0,2], [0,0], [0,0])
a3d.plot([0,0], [0,2], [0,0])
a3d.plot([0,0], [0,0], [0,2])

'''a3dacc.set_xlim(-1.2,1.2)
a3dacc.set_ylim(-1.2,1.2)
a3dacc.set_zlim(-1.2,1.2)
a3dacc.scatter([0], [0], [0], s=40)
r, = a3dacc.plot([0,1], [0,0], [0,0], lw=2)
p, = a3dacc.plot([0,0], [0,1], [0,0], lw=2)
a3dacc.plot([0,2], [0,0], [0,0])
a3dacc.plot([0,0], [0,2], [0,0])
a3dacc.plot([0,0], [0,0], [0,2])
'''
num_samples = 0
buff = 0
kyaw_alpha = [0]
kyaw_beta = [0]
kyaw_gamma = [0]
t = [0]


scopes = [axes.plot(t, kyaw_alpha, label=r'$\theta_x$')[0], axes.plot(t, kyaw_beta, label=r'$\theta_y$')[0], axes.plot(t, kyaw_gamma, label=r'$\theta_z$')[0]]
axes.legend(prop=dict(size=14))
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=3, mode="expand", borderaxespad=0.)
axes.set_ylim(-55,195)

g_scale = 65.5 #Gyro 500
a_scale = 16384.0 #Accel 2g

accel_history  = np.zeros((10,3))
ktheta_history  = np.zeros((10,3))
bias_history = np.zeros((10,3))
bias = np.zeros(3)
ktheta = np.array([1.5707963267948966,1.5707963267948966,0])
P = np.array([np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3)])
K = np.array([np.zeros(3),np.zeros(3)])

print('Me Ready')
time.sleep(2.5)
#Handshake MAY BE REDUNDANT
arduino.notify()
data = [0]*6
while True:
	try:
		num = arduino.readn(12)
		num = [ord(x) for x in num]
	except:
		print('Serial error!')
		raise RuntimeError
	_ind=0 #this var is connected to for loop below!!
	for i in range(0,12, 2):
		data[_ind] = (num[i]<<8)|num[i+1]
		if data[_ind] & 0x8000:
			data[_ind] = data[_ind] - 0x10000
		_ind += 1
	accel3= np.array([float(data[0])/a_scale, float(data[1])/a_scale, float(data[2])/a_scale])
	gyro3 = np.array([float(data[3])/g_scale, float(data[4])/g_scale, float(data[5])/g_scale])

	#variance work
	accel_history = np.delete(accel_history,9,0)
	accel_history = np.insert(accel_history,0,accel3,0)
	R_measure = variance(accel_history)
	bias_history = np.delete(bias_history,9,0)
	bias_history = np.insert(bias_history,0,bias,0)
	Q_bias = variance(bias_history)
	ktheta_history = np.delete(ktheta_history,9,0)
	ktheta_history = np.insert(ktheta_history,0,ktheta,0)
	Q_angle = variance(ktheta_history)
	
	#ktheta = kalman(accel3,bias,gyro3,ktheta*180/np.pi,Q_angle,Q_bias)
	omega = gyro3 - bias #dps - dps
	ktheta = ktheta + omega*TIME_INTERVAL #d - d
	P[0] += TIME_INTERVAL * (TIME_INTERVAL*P[3] - P[1] - P[2] + Q_angle)
	P[1] -= TIME_INTERVAL * P[3]
	P[2] -= TIME_INTERVAL * P[3]
	P[3] += Q_bias * TIME_INTERVAL
	S = P[0] + R_measure
	K[0] = P[0] / S
	K[1] = P[2] / S
	kangle = calcAngle(accel3)*180/np.pi #degree
	ky = kangle - ktheta
	ktheta += K[0] * ky
	bias  += K[1] * ky #dps
	P00_temp = P[0]
	P01_temp = P[1]
	P[0] -= K[0] * P00_temp
	P[1] -= K[0] * P01_temp
	P[2] -= K[1] * P00_temp
	P[3] -= K[1] * P01_temp
	#print "%.3f %.3f %.3f" %(ktheta[0], ktheta[1], ktheta[2])

	kyaw_alpha.append(ktheta[0])
	kyaw_beta.append(ktheta[1])
	kyaw_gamma.append(ktheta[2])
	num_samples += 1
	t.append(num_samples)
	if num_samples>200:
		del t[0]
		del kyaw_alpha[0]
		del kyaw_beta[0]
		del kyaw_gamma[0]
	
	calcPose(omega) #--- has to be changed
	pose = np.array([np.array([np.zeros(3), rpy[0]]).T,	np.array([np.zeros(3), rpy[1]]).T, np.array([np.zeros(3), rpy[2]]).T])
	
	axes.set_xlim(t[0], num_samples)
	scopes[0].set_data(t, kyaw_alpha)
	scopes[1].set_data(t, kyaw_beta)
	scopes[2].set_data(t, kyaw_gamma)

	r.set_data(pose[0][:2])
	r.set_3d_properties(pose[0][2])
	p.set_data(pose[1][:2])
	p.set_3d_properties(pose[1][2])
	y.set_data(pose[2][:2])
	y.set_3d_properties(pose[2][2])
	# Kalman G-Yaw
	gyaw.set_data([0, -np.cos(ktheta[0]*np.pi/180)], [0, -np.cos(ktheta[1]*np.pi/180)])
	gyaw.set_3d_properties([0, np.cos(ktheta[2]*np.pi/180)])
	
	if buff>15:
		buff=0
		plt.draw()
	buff += 1

plt.ioff()
plt.show()
arduino.notify()
arduino.powerdown()