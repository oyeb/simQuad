#code to plot filtered angle and rpy of the quad  --- ino file mpu level2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import serial, time

def rotate(v, axis, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    t = 1-c
    mat = np.array([ [c+axis[0]*axis[0]*t, axis[0]*axis[1]*t-axis[2]*s, axis[0]*axis[2]*t+axis[1]*s],
                     [axis[0]*axis[1]*t+axis[2]*s, c+axis[1]*axis[1]*t, axis[1]*axis[2]*t-axis[0]*s],
                     [axis[0]*axis[2]*t-axis[1]*s, axis[1]*axis[2]*t+axis[0]*s, c+axis[2]*axis[2]*t] ])
    return mat.dot(v.T)

def calcPose(omega):
	dtheta  = omega*dt
	rpy[1] = rotate(rpy[1], rpy[0], dtheta[0])
	rpy[0] = rotate(rpy[0], rpy[1], dtheta[1])
	rpy[2] = np.cross(rpy[0], rpy[1])
	rpy[1] = rotate(rpy[1], rpy[2], dtheta[2])
	rpy[0] = rotate(rpy[0], rpy[2], dtheta[2])

def calcAngle(accel):
	angle = np.zeros(3)
	angle[0]  = np.arctan(accel[1]/np.sqrt((accel[0]*accel[0])+(accel[2]*accel[2])))+1.5707963267948966
	angle[1]  = np.arctan(accel[0]/np.sqrt((accel[1]*accel[1])+(accel[2]*accel[2])))+1.5707963267948966
	angle[2]  = np.arctan(accel[2]/np.sqrt((accel[1]*accel[1])+(accel[0]*accel[0])))+1.5707963267948966
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
arduino = serial.Serial('/dev/ttyACM0', 57600)
dt = .005*np.pi/180.0
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
gyro_x = [0]
gyro_y = [0]
gyro_z = [0]
t = [0]
acc_x=[0]
acc_y=[0]
acc_z=[0]


scopes = [axes.plot(t, gyro_x, label=r'$\theta_x$')[0], axes.plot(t, gyro_y, label=r'$\theta_y$')[0], axes.plot(t, gyro_z, label=r'$\theta_z$')[0]]
axes.legend(prop=dict(size=14))
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=3, mode="expand", borderaxespad=0.)
axes.set_ylim(-95,95)

'''scopesacc = [axes.plot(t, acc_x, label=r'$\omega_x$')[0], axes.plot(t, acc_y, label=r'$\omega_y$')[0], axes.plot(t, acc_z, label=r'$\omega_z$')[0]]
axes.legend(prop=dict(size=14))
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=3, mode="expand", borderaxespad=0.)
'''

g_scale = 65.5 #Gyro 500
a_scale = 16384.0 #Accel 2g

val  = np.zeros((10,3))
vala  = np.zeros((10,3))
valb = np.zeros((10,3))
bias = np.zeros(3)
ktheta = np.array([1.5707963267948966,1.5707963267948966,0])
P = np.array([np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3)])
K = np.array([np.zeros(3),np.zeros(3)])

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
	accel3= np.array([float(data[0])/a_scale, float(data[1])/a_scale, float(data[2])/a_scale])
	gyro3 = np.array([float(data[3])/g_scale, float(data[4])/g_scale, float(data[5])/g_scale])
	
	gyro_x.append(ktheta[0]*180/np.pi)
	gyro_y.append(ktheta[1]*180/np.pi)
	gyro_z.append(ktheta[2]*180/np.pi)
	num_samples += 1
		
	t.append(num_samples)
	calcPose(gyro3) #--- has to be changed

	
	val = np.delete(val,9,0)
	val = np.insert(val,0,accel3,0)
	R_measure = variance(val)
	valb = np.delete(valb,9,0)
	valb = np.insert(valb,0,bias,0)
	Q_bias = variance(valb)
	vala = np.delete(vala,9,0)
	vala = np.insert(vala,0,ktheta,0)
	Q_angle = variance(vala)
	
	#ktheta = kalman(accel3,bias,gyro3,ktheta*180/np.pi,Q_angle,Q_bias)
	omega = gyro3 - bias
	ktheta = ktheta*180/np.pi+ omega*0.005
	P[0] += 0.005 * (0.005*P[3] - P[1] - P[2] + Q_angle)
	P[1] -= 0.005 * P[3]
	P[2] -= 0.005 * P[3]
	P[3] += Q_bias * 0.005
	S = P[0] + R_measure
	K[0] = P[0] / S
	K[1] = P[2] / S
	kangle = calcAngle(accel3)
	ky = kangle*180/np.pi - ktheta
	ktheta += K[0] * ky
	bias  += K[1] * ky
	P00_temp = P[0]
	P01_temp = P[1]
	P[0] -= K[0] * P00_temp
	P[1] -= K[0] * P01_temp
	P[2] -= K[1] * P00_temp
	P[3] -= K[1] * P01_temp
	ktheta = ktheta*np.pi/180
	
	if num_samples>200:
		del t[0]
		del gyro_x[0]
		del gyro_y[0]
		del gyro_z[0]
		#del acc_x[0]
		#del acc_y[0]
		#del acc_z[0]
	
	axes.set_xlim(t[0], num_samples)
	#axesacc.set_xlim(t[0], num_samples)
	
	scopes[0].set_data(t, gyro_x)
	scopes[1].set_data(t, gyro_y)
	scopes[2].set_data(t, gyro_z)
	#scopesacc[0].set_data(t, acc_x)
	#scopesacc[1].set_data(t, acc_y)
	#scopesacc[2].set_data(t, acc_z)
	
	pose = np.array([np.array([np.zeros(3), rpy[0]]).T,	np.array([np.zeros(3), rpy[1]]).T, np.array([np.zeros(3), rpy[2]]).T])
	r.set_data(pose[0][:2])
	r.set_3d_properties(pose[0][2])
	p.set_data(pose[1][:2])
	p.set_3d_properties(pose[1][2])
	y.set_data(pose[2][:2])
	y.set_3d_properties(pose[2][2])
	
	if buff>15:
		buff=0
		plt.draw()
	buff += 1

plt.ioff()
plt.show()
