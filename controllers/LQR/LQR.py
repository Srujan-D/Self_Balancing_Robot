from controller import Robot, Keyboard, Supervisor,Emitter,Receiver,PositionSensor, InertialUnit, GPS
from math import atan, pi, acos, sqrt, sin, cos, atan2
import numpy as np

robot = Robot()
TIMESTEP = int(robot.getBasicTimeStep())
timestep = TIMESTEP

angle = 0

max_spot_speed = 8
max_travel_speed = 2
turn_speed = 1

theta = [0,0,0]

def setSpeed(l,r):
    motor[0].setVelocity(l)
    motor[1].setVelocity(r)

state = [0, 0, 0, 0]
K = [4., -2.7, 122, 550]

# K = [30, -5.7, 350, 900] # works for standing still, not for going to a point
# K = [70, -6, 123, 800] # works for lDis = rDis = 460 rs.

def lqrSpeed(phi, dphi, theta, theta_dot):
    state[0] = phi
    state[1] = dphi
    state[2] = theta 
    state[3] = theta_dot
    
    d = [K[i]*state[i] for i in range(len(K))]
    s = sum(d)
    
    if abs(s) > max_spot_speed:
        s = max_spot_speed*(s/abs(s))
    return s
    
def getDist(setpoint):
	global gps
	current_pos = gps.getValues()
	return sqrt((current_pos[0]-setpoint[0])**2 + (current_pos[2]-setpoint[1])**2)

#Setup motors
motorNames = ['motor_1', 'motor_2']
motor = []
   
for i in range(len(motorNames)):
    motor.append(robot.getDevice(motorNames[i]))
    motor[i].setPosition(float('inf'))
    motor[i].setVelocity(0)


#Setup IMU
# imu = robot.getDevice('InertialUnit')
imu = InertialUnit("inertial unit")
imu.enable(TIMESTEP)

#Setup GPS
gps = GPS('gps')
gps.enable(TIMESTEP)
print(gps.getValues())

#Setup Accelerometer and Gyroscope
accMeter = robot.getDevice('accel')
accMeter.enable(TIMESTEP)
gyro = robot.getDevice('gyro')
gyro.enable(TIMESTEP)

#Setup Encoders
lEnc = robot.getDevice("leftEncoder")
lEnc.enable(TIMESTEP)
rEnc = robot.getDevice("rightEncoder")
rEnc.enable(TIMESTEP)
robot.step(timestep)

oT = robot.getTime()
gAng = 0
oldxD = 0
oldT = oT

# Get reference value of encoder
lDisRef = lEnc.getValue()
rDisRef = rEnc.getValue()

stop = False
xD = 0

path = [[0.25,0.],[0.25,0.25]]

theta_setpoint = None
setpoint = None

turn = False
travel = False

while robot.step(timestep) != -1:
	# Run Control Algorithm
	[x_pos,y_pos,z_pos] = gps.getValues()
	[roll,pitch,yaw] = imu.getRollPitchYaw()
	[a_x,a_y,a_z] = accMeter.getValues()
	[g_x,g_y,g_z] = np.array(gyro.getValues())

	gAng += (g_x*(robot.getTime()-oT))*180/pi	# angular velocity * time in the x direction wrt the robot
	
	oT = robot.getTime()
	if g_y!=0:
		aAng = -atan(a_z/a_y)*180/pi	# tilt angle in the x direction of robot frame 
	else:
		aAng = 0
	if aAng>0:
		aAng = 90-aAng	# complement of actual tilt is given by accelerometer
	else:
		aAng = -1*(90+aAng)
	'''	Complementary filter: A complimentary filter is used to remove both of these noises. A low pass filter and a high pass filter is
		combined to fuse the pitch angles'''
	angle = 0.99*gAng + 0.01*aAng

	if not(stop) :
		
		lDis = lEnc.getValue() - lDisRef
		rDis = rEnc.getValue() - rDisRef
		# print("lDis = ", lDis)
		# print("rDis = ", rDis)

		xD = (lDis+rDis)*90/pi # total rotation in degree
		
		# dist = xD*0.025*pi/90
		# print("dist is : ", xD*0.025*pi/90)
		
		if (robot.getTime()-oldT)>0:
			speed = lqrSpeed(xD, (oldxD-xD)/(robot.getTime()-oldT), angle, g_x)
		else:
			speed = lqrSpeed(xD, 0, angle, g_x)
		oldxD = xD
		oldT = robot.getTime()
			
	elif stop:
		speed = speed

	#print(speed)
	# print(yaw)
	
	try:
		if setpoint != path[0]:
			theta_setpoint = atan2((path[0][0] - round(x_pos,2)), (path[0][1] - round(z_pos,2)))
			setpoint = path[0]
	except IndexError:
		stop = True
	print(yaw, theta_setpoint, setpoint)
	if not stop and abs(theta_setpoint - yaw) > 0.005:	# for clockwise
		print("turn, dont travel")
		turn = True
		travel = False
		if	(theta_setpoint-yaw) > 0:
			setSpeed(speed+turn_speed,speed-turn_speed)  
		else:
			setSpeed(speed-turn_speed,speed+turn_speed)
	elif not stop and path != [] and getDist(setpoint) > 0.02:
		# print(gps.getValues(), getDist(setpoint))
		print("dont turn, travel")
		turn = False
		travel = True
		setSpeed(speed+max_travel_speed, speed+max_travel_speed)
	else:
		print("dont turn, dont travel")
		turn = False
		travel = False
		try:
			path.pop(0)
			stop = False
		except IndexError:
			# stop = True
			continue
		setSpeed(speed,speed)
    