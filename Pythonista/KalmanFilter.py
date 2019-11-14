import ui
import location 
import motion 
import time
import math 
from numpy import *
from numpy.linalg import inv 


loc = {}
lat = 42.7807
lon = 3.0276
accel_var = 0.0001407431765
accel_var = 0.25
messen = True
compensation = 0 
file = open("data.txt","w")

def button_tapped(sender):
	global messen
	messen = not messen
	
def gps_location_label_change_lat(label):
	label.text = str(loc['latitude'])

def gps_location_label_change_lon(label):
	label.text = str(loc['longitude'])

def height(label):
	label.text = str(loc['altitude'])
	
def kf_predict(X, P, A, Q, B, U):
	X = dot(A,X) + dot(B,U)
	P = dot(A, dot(P, A.T)) + Q
	return(X,P)
	
def kf_update(X, P, Y, H, R):
	IM = dot(H, X)
	IS = R + dot(H, dot(P, H.T))
	K = dot(P, dot(H.T, inv(IS)))
	X = X + dot(K, (Y-IM))
	P = P - dot(K, dot(IS, K.T))
	return (X,P,K,IM,IS)

location.start_updates()
motion.start_updates()
loc = location.get_location()
var_vel = 0.2

#time step of mobile movement 
dt = 0.1

#initialization of mobile movement 
P = diag((1, 1, 1, 1))
'''P = [[3.28273128e-07, 6.56542951e-10, 3.28269580e-07, 6.56545752e-10],[1.53905371e-11, 3.07647635e-14, 1.53788076e-11, 3.07719593e-14], [7.56233278e-10, 1.51245836e-12, 7.56224458e-10, 1.51246514e-12], [4.29750967e-14, 8.01167850e-17, 3.64882535e-14, 8.33920877e-17]]'''
A = array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
Q = accel_var*array([[0.0004, 0.004, 0, 0], [0.004, 0.04, 0, 0], [0, 0, 0.0004, 0.004], [0, 0, 0.004, 0.04]])
B = array([[0.02, 0], [0.2, 0], [0, 0.02], [0, 0.2]])

#Rt = P
heading_direction = 0
heading = 0

v = ui.load_view('Position')
my_label_lat = v['label1']
my_label_lon = v['label2']
my_label_height = v['label3']
my_label_heading = v['label7']
my_label_speed = v['label8']
my_label_accel = v['label9']
my_label_est_lat = v['label10']
my_label_est_lon = v['label11']
v.present('fullscreen')

while messen:
	time.sleep(0.2)
	mag = motion.get_magnetic_field()
	loc = location.get_location()
	gyro = motion.get_attitude()
	theta = gyro[1]
	psi = gyro[0]
	#B = 4.77867*10**(-5)
	I = 62.8166667*math.pi/180
	
	my_label_heading.text = str(heading_direction)
	my_label_speed.text = str(heading_direction)
	if int(mag[1]) > 0:
		heading = math.atan2(mag[0], mag[1])*180/math.pi
	if int(mag[1]) == 0 and int(mag[0]) > 0:
		heading = 90
	if int(mag[1]) == 0 and int(mag[0]) < 0:
		heading = -90
	if int(mag[0]) == 0 and int(mag[1]) > 0:
		heading = 0
		compensation = int(gyro[2])
	if int(mag[0]) == 0 and int(mag[1]) < 0:
		heading = 180
	if int(mag[1]) < 0 and int(mag[0]) > 0:
		heading = (math.atan2(mag[0], mag[1]))*180/math.pi
	if int(mag[1]) < 0 and int(mag[0]) < 0:
		heading = (math.atan2(mag[0], mag[1]))*180/math.pi
	my_label_accel.text = str(int(heading))


while not messen:
	loc = location.get_location()
	accel = motion.get_user_acceleration()
	mag = motion.get_magnetic_field()
	gyro = motion.get_attitude()

#determining the heading direction
	if int(mag[1]) > 0:
		heading = math.atan2(mag[0], mag[1])*180/math.pi
	if int(mag[1]) == 0 and int(mag[0]) > 0:
		heading = 90
	if int(mag[1]) == 0 and int(mag[0]) < 0:
		heading = -90
	if int(mag[0]) == 0 and int(mag[1]) > 0:
		heading = 0
	if int(mag[0]) == 0 and int(mag[1]) < 0:
		heading = 180
	if int(mag[1]) < 0 and int(mag[0]) > 0:
		heading = (math.atan2(mag[0], mag[1]))*180/math.pi
	if int(mag[1]) < 0 and int(mag[0]) < 0:
		heading = (math.atan2(mag[0], mag[1]))*180/math.pi

	my_label_heading.text = str(int(heading))
	my_label_speed.text = str(loc['speed'])
	my_label_accel.text = str(accel[1])
	
	
	accel_lat = math.fabs(accel[1])*math.sin(heading*math.pi/180)
	accel_lon = math.fabs(accel[1])*math.cos(heading*math.pi/180)

	vel_lat = loc['speed']*math.sin(heading)
	vel_lon = loc['speed']*math.cos(heading)
	vel_lat = 0.95*accel_lat*0.2 + 0.05*vel_lat
	vel_lon = 0.95*accel_lon*0.2 + 0.05*vel_lon
	
	lon = lon + vel_lon*0.2
	lat = lat + vel_lat*0.2
	
	#Initialization of U, H, R and Y
	print(loc['horizontal_accuracy'])
	R = array([[loc['horizontal_accuracy']**4, loc['horizontal_accuracy']**2*var_vel, loc['horizontal_accuracy']**4, loc['horizontal_accuracy']**2*var_vel],
						[var_vel*loc['horizontal_accuracy']**2, var_vel**2, var_vel*loc['horizontal_accuracy']**2, var_vel**2],
						[loc['horizontal_accuracy']**4, loc['horizontal_accuracy']**2*var_vel, loc['horizontal_accuracy']**4, loc['horizontal_accuracy']**2*var_vel],
						[var_vel*loc['horizontal_accuracy']**2, var_vel**2, var_vel*loc['horizontal_accuracy']**2, var_vel**2]])
						
	U = array([[accel_lon], [accel_lat]])
	H = diag((1, 1, 1, 1))
	Y = array([[loc['longitude']], [loc['speed']], [loc['latitude']], [loc['speed']]])
	
	X = array([[lon], [0], [lat], [0]])
	(X, P) = kf_predict(X, P, A, Q, B, U)
	time.sleep(0.2)
	(X, P, K, IM, IS) = kf_update(X, P, Y, H, R)
	
	my_label_est_lon.text = str(X[0][0])
	my_label_est_lat.text = str(X[2][0])
	
	file.write(str(loc['latitude']) + ", " + str(loc['longitude']) + "," + str(loc['altitude']) + "," + str(X[0][0]) + "," + str(X[2][0]) + ",\n")
	gps_location_label_change_lat(my_label_lat)
	gps_location_label_change_lon(my_label_lon)
	height(my_label_height)
	
print(P)
motion.stop_updates()
location.stop_updates()
file.close()
