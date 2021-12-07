from typing import Type
from controller import Robot, Keyboard, Supervisor,Emitter,Receiver,PositionSensor, InertialUnit, GPS
from math import atan, pi, acos, sqrt, sin, cos, atan2
import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure

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

# static gain matrix
K = [4., -2.7, 122, 550]

def lqrSpeed(x, dx, theta, theta_dot):
    state[0] = x
    state[1] = dx
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
imu = InertialUnit("inertial unit")
imu.enable(TIMESTEP)

#Setup GPS
gps = GPS('gps')
gps.enable(TIMESTEP)

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

def map_return(x, y):
	
	n = x
	m = y

	grid = [[0 for i in range(n)] for j in range (m)]

	grid [0][0] = 0
	grid [11][11] = 0

	grid[2][11] = 1
	grid[2][10] = 1
	grid[2][4] = 1
	grid[2][5] = 1
	grid[3][11] = 1
	grid[3][10] = 1
	grid[3][4] = 1
	grid[3][5] = 1
	grid[4][2] = 1
	grid[4][3] = 1
	grid[5][2] = 1
	grid[5][3] = 1

	grid[6][6] = 1
	grid[6][7] = 1
	grid[6][8] = 1
	grid[6][9] = 1
	grid[7][6] = 1
	grid[7][7] = 1
	grid[7][8] = 1
	grid[7][9] = 1

	grid[8][1] = 1
	grid[8][2] = 1
	grid[9][1] = 1
	grid[9][2] = 1

	grid[10][3] = 1
	grid[10][4] = 1
	grid[11][3] = 1
	grid[11][4] = 1


	grid = np.array(grid)

	return grid

def heuristic(a, b, type = "euclidean"):
	if type == "euclidean":
		return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
	elif type == "manhattan":
		return abs(a[0] - b[0]) + abs(a[1] - b[1])

def check_neighbours(array, neighbor):
	if array[neighbor[0]][neighbor[1]] == 1:
		return True
	else:
		return False

def astar(array, start, goal, heuristic_type):

	neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
	close_set = set()
	came_from = {}
	gscore = {start:0}
	fscore = {start:heuristic(start, goal, heuristic_type)}
	oheap = []

	heapq.heappush(oheap, (fscore[start], start))
 
	while oheap:

		current = heapq.heappop(oheap)[1]
		if current == goal:
			data = []
			while current in came_from:
				data.append(current)
				current = came_from[current]
			return data
		close_set.add(current)
		for i, j in neighbors:
			neighbor = current[0] + i, current[1] + j
			tentative_g_score = gscore[current] + heuristic(current, neighbor, heuristic_type)
			if 0 <= neighbor[0] < array.shape[0]:
				if 0 <= neighbor[1] < array.shape[1]:
					if check_neighbours(grid, neighbor):
						continue
				else:
                    # array bound y walls
					continue
			else:
				# array bound x walls
				continue

			if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
				continue

			if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
				came_from[neighbor] = current
				gscore[neighbor] = tentative_g_score
				fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal, heuristic_type)
				heapq.heappush(oheap, (fscore[neighbor], neighbor))

	return False

def path_return(route):

	# route = route + [start]

	route = route[::-1]
	#extract x and y coordinates from route list

	x_coords = []
	y_coords = []

	path = []
	for i in (range(0,len(route))):
		x = route[i][0]*0.25 - 1.375
		y = route[i][1]*0.25 - 1.375
		# x = route[i][0]
		# y = route[i][1]
		x_coords.append(x)
		y_coords.append(y)
		path.append((y,x))
	# print(path)
	return path
# plot map and path

def plot_path(grid, start, goal, x_coords, y_coords):
	fig, ax = plt.subplots(figsize=(20,20))
	ax.imshow(grid, cmap=plt.cm.Dark2)
	ax.scatter(start[1],start[0], marker = "*", color = "yellow", s = 200)
	ax.scatter(goal[1],goal[0], marker = "*", color = "red", s = 200)
	ax.plot(y_coords,x_coords, color = "black")
	plt.show()

start = (0,0)
goal = (11,11)

grid = map_return(12, 12)
route = astar(grid, start, goal, heuristic_type="euclidean")
path = path_return(route)

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
		
		if (robot.getTime()-oldT)>0:
			speed = lqrSpeed(xD, (oldxD-xD)/(robot.getTime()-oldT), angle, g_x)
		else:
			speed = lqrSpeed(xD, 0, angle, g_x)
		oldxD = xD
		oldT = robot.getTime()
			
	elif stop:
		speed = speed
	
	try:
		if setpoint != path[0]:
			theta_setpoint = atan2((path[0][0] - round(x_pos,2)), (path[0][1] - round(z_pos,2)))
			setpoint = path[0]
	except IndexError:
		setpoint = (z_pos, x_pos)
		theta_setpoint = yaw
		# stop = True
	except TypeError:
		print("path is ", path)
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
			# print("Falling..............")
			setSpeed(speed, speed)
			continue
		# setSpeed(speed,speed)
