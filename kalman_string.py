#code for printing filtered angle and accel angle--- ino file trial1
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import serial, time

'''def rotate(v, axis, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    t = 1-c
    mat = np.array([ [c+axis[0]*axis[0]*t, axis[0]*axis[1]*t-axis[2]*s, axis[0]*axis[2]*t+axis[1]*s],
                     [axis[0]*axis[1]*t+axis[2]*s, c+axis[1]*axis[1]*t, axis[1]*axis[2]*t-axis[0]*s],
                     [axis[0]*axis[2]*t-axis[1]*s, axis[1]*axis[2]*t+axis[0]*s, c+axis[2]*axis[2]*t] ])
    return mat.dot(v.T)



def calcPose(omega):
	theta = omega*dt
	rpy[1] = rotate(rpy[1], rpy[0], theta[0])
	rpy[0] = rotate(rpy[0], rpy[1], theta[1])
	rpy[2] = np.cross(rpy[0], rpy[1])
	rpy[1] = rotate(rpy[1], rpy[2], theta[2])
	rpy[0] = rotate(rpy[0], rpy[2], theta[2])'''

def calcAngle(accel):
	angle = np.zeros(3)
	angle[0]  = np.arctan(accel[0]/np.sqrt((accel[1]*accel[1])+(accel[2]*accel[2])))
	angle[1]  = np.arctan(accel[1]/np.sqrt((accel[0]*accel[0])+(accel[2]*accel[2])))
	angle[2]  = np.arctan(accel[2]/np.sqrt((accel[1]*accel[1])+(accel[0]*accel[0])))
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
#dt = 5.235987755982989e-05 #.003 ms * (pi/180)
dt = .005*np.pi/180
rpy = np.eye(3)

'''fig = plt.figure(figsize=(16,6))
axes = fig.add_subplot(121)
axesacc = fig.add_subplot(122)
a3d = fig.add_subplot(222, projection='3d')
a3dacc = fig.add_subplot(224, projection='3d')

a3d.set_xlim(-1.2,1.2)
a3d.set_ylim(-1.2,1.2)
a3d.set_zlim(-1.2,1.2)
a3d.scatter([0], [0], [0], s=40)
r, = a3d.plot([0,1], [0,0], [0,0], lw=2)
p, = a3d.plot([0,0], [0,1], [0,0], lw=2)
a3d.plot([0,2], [0,0], [0,0])
a3d.plot([0,0], [0,2], [0,0])
a3d.plot([0,0], [0,0], [0,2])

a3dacc.set_xlim(-1.2,1.2)
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
val  = np.zeros((10,3))
valb = np.zeros((10,3))


'''scopes = [axes.plot(t, gyro_x, label=r'$\theta_x$')[0], axes.plot(t, gyro_y, label=r'$\theta_y$')[0], axes.plot(t, gyro_z, label=r'$\theta_z$')[0]]
axes.legend(prop=dict(size=14))
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=3, mode="expand", borderaxespad=0.)
axes.set_ylim(-4,4)

scopesacc = [axesacc.plot(t, acc_x, label=r'$\theta_x$')[0], axesacc.plot(t, acc_y, label=r'$\theta_y$')[0], axesacc.plot(t, acc_z, label=r'$\theta_z$')[0]]
axesacc.legend(prop=dict(size=14))
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=3, mode="expand", borderaxespad=0.)
axesacc.set_ylim(-4,4)'''

conversion = 131.0 #Gyro 250
conversionacc = 16384.0 #Accel 2g
Q_angle = np.array([0.,0.,0.]) 
Q_bias = np.array([0.,0.,0.])
R_measure = np.array([0.003,0.003,0.003])
bias = np.zeros(3)
theta = np.zeros(3)
P = np.array([np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3)])
K = np.array([np.zeros(3),np.zeros(3)])

while True:
	try:
		datas = arduino.readline().strip()
	except:
		#variance(val)
		print 'Serial error!'
		raise RuntimeError
	datas = datas.split(" / / ")
	datas[0]=datas[0].split(' ')
	datas[1]=datas[1].split(' ')
	if (len(datas[1]) == 3) and (len(datas[0]) ==3):
		datas[1] = np.array([float(datas[1][0])/conversion, float(datas[1][1])/conversion, float(datas[1][2])/conversion])
		datas[0] = np.array([float(datas[0][0])/conversionacc, float(datas[0][1])/conversionacc, (float(datas[0][2])/conversionacc)+1.0])
		datas =np.array([datas[0],datas[1]])
		
		#Angle = calcAngle(datas[1],datas[0])
		gyro_x.append(datas[1][0])
		gyro_y.append(datas[1][1])
		gyro_z.append(datas[1][2])
		acc_x.append(datas[0][0])
		acc_y.append(datas[0][1])
		acc_z.append(datas[0][2])
		num_samples += 1

		print "filterd"
		print theta*180/np.pi
		print "accel" 
		print calcAngle(datas[0])*180/np.pi
		t.append(num_samples)
		val = np.delete(val,9,0)
		val = np.insert(val,0,datas,0)
		Q_angle = variance(val)

		#kalman filter
		valb = np.delete(valb,9,0)
		valb = np.insert(valb,0,bias,0)
		Q_bias = variance(valb)
		omega = datas[1] - bias
		theta += omega*dt
		P[0] += dt * (dt*P[3] - P[1] - P[2] + Q_angle)
		P[1] -= dt * P[3]
		P[2] -= dt * P[3]
		P[3] += Q_bias * dt
		S = P[0] + R_measure
		K[0] = P[0] / S
		K[1] = P[2] / S
		y = calcAngle(datas[0]) - theta
		theta += K[0] * y
		bias  += K[1] * y
		P00_temp = P[0]
		P01_temp = P[1]
		P[0] -= K[0] * P00_temp
		P[1] -= K[0] * P01_temp
		P[2] -= K[1] * P00_temp
		P[3] -= K[1] * P01_temp
		


		if num_samples>200:
			del t[0]
			del gyro_x[0]
			del gyro_y[0]
			del gyro_z[0]
			del acc_x[0]
			del acc_y[0]
			del acc_z[0]
		
		'''axes.set_xlim(t[0], num_samples)
		axesacc.set_xlim(t[0], num_samples)
		
		scopes[0].set_data(t, gyro_x)
		scopes[1].set_data(t, gyro_y)
		scopes[2].set_data(t, gyro_z)

		scopesacc[0].set_data(t, acc_x)
		scopesacc[1].set_data(t, acc_y)
		scopesacc[2].set_data(t, acc_z)'''
		
		#print (datas[0]**2+datas[1]**2+datas[2]**2)**.5

		'''pose = np.array([np.array([np.zeros(3), rpy[0]]).T,	np.array([np.zeros(3), rpy[1]]).T, np.array([np.zeros(3), rpy[2]]).T])
		r.set_data(pose[0][:2])
		r.set_3d_properties(pose[0][2])
		p.set_data(pose[1][:2])
		p.set_3d_properties(pose[1][2])
		'''
		if buff>15:
			buff=0
			plt.draw()
		buff += 1

plt.ioff()
plt.show()
