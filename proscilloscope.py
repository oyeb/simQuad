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
	theta = omega*dt
	rpy[1] = rotate(rpy[1], rpy[0], theta[0])
	rpy[0] = rotate(rpy[0], rpy[1], theta[1])
	rpy[2] = np.cross(rpy[0], rpy[1])
	rpy[1] = rotate(rpy[1], rpy[2], theta[2])
	rpy[0] = rotate(rpy[0], rpy[2], theta[2])

plt.ion()
arduino = serial.Serial('/dev/ttyACM4', 57600)
dt = 8.726646259971648e-05 #.005 ms * (pi/180)
#dt = .00006
rpy = np.eye(3)

fig = plt.figure(figsize=(16,6))
axes = fig.add_subplot(121)
a3d = fig.add_subplot(122, projection='3d')
a3d.set_xlim(-1.2,1.2)
a3d.set_ylim(-1.2,1.2)
a3d.set_zlim(-1.2,1.2)
a3d.scatter([0], [0], [0], s=40)
r, = a3d.plot([0,1], [0,0], [0,0], lw=2)
p, = a3d.plot([0,0], [0,1], [0,0], lw=2)
a3d.plot([0,2], [0,0], [0,0])
a3d.plot([0,0], [0,2], [0,0])
a3d.plot([0,0], [0,0], [0,2])

num_samples = 0
buff = 0
gyro_x = [0]
gyro_y = [0]
gyro_z = [0]
t = [0]
scopes = [axes.plot(t, gyro_x, label=r'$\omega_x$')[0], axes.plot(t, gyro_y, label=r'$\omega_y$')[0], axes.plot(t, gyro_z, label=r'$\omega_z$')[0]]
axes.legend(prop=dict(size=14))
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=3, mode="expand", borderaxespad=0.)
axes.set_ylim(-510, 510)

conversion = 65.5 #Gyro 500
#conversion = 16384 #Accel 2g

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
	calcPose(datas)
	if num_samples>200:
		del t[0]
		del gyro_x[0]
		del gyro_y[0]
		del gyro_z[0]
	axes.set_xlim(t[0], num_samples)
	scopes[0].set_data(t, gyro_x)
	scopes[1].set_data(t, gyro_y)
	scopes[2].set_data(t, gyro_z)
	#print (datas[0]**2+datas[1]**2+datas[2]**2)**.5

	pose = np.array([np.array([np.zeros(3), rpy[0]]).T,	np.array([np.zeros(3), rpy[1]]).T, np.array([np.zeros(3), rpy[2]]).T])
	r.set_data(pose[0][:2])
	r.set_3d_properties(pose[0][2])
	p.set_data(pose[1][:2])
	p.set_3d_properties(pose[1][2])

	if buff>15:
		buff=0
		plt.draw()
	buff += 1

plt.ioff()
plt.show()