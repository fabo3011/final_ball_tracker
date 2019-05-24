#!/usr/bin/env python2
# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import math
import cv2
import rospy
import sys

from std_msgs.msg import Char
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from collections import deque
from nav2d_operator.msg import cmd

#ROS information
rospy.init_node('ball_tracker')
publisher_servos = rospy.Publisher('servo_move', Char, queue_size = 1)
publisher_cmd = rospy.Publisher('cmd', cmd, queue_size=10)
publisher_controller = rospy.Publisher('ball_controller', UInt8, queue_size=1)

#ROS subscriber for servo possition
pos=Int8()
def callback (data):
	global pos
	pos=data
rospy.Subscriber("servo_pos", Int8, callback)

gps_controller = UInt8()
tracking=0
def callback2 (data):
	global gps_controller
	gps_controller=data
rospy.Subscriber("gps_controller", UInt8, callback2)

#Data to calculate distance
F=710.67
W=6.7
x=0
y=0
profundity=0
min_profundity=50

#Variables for ROS messages
follow = cmd()
servos = Char()
ball_controller = UInt8()

area=0
area_before=0
x_before=0
y_before=0
start=0
circularity_thresh = 0.6

#Thresholds to determine constant presence of suspect object
samples=50 # samples for the filter
thresh1 =samples*0.20 
thresh2 =samples*0.40
thresh3 =samples*0.60
thresh4 =samples*0.85
change_thresh=0

#Timer
start=rospy.get_rostime()
now=rospy.get_rostime()
start.secs=0
now.secs=0
dt= 0
timeout=60

#Filtrrate by image samples
result=0
results = deque([0])

#Byte to control servos
servos=0x00
pastServos=0x00 #compare to only send when the byte changes

#Variables for rover velocity
max_velocity=0.5
max_turn=0.6

#Area to determine if ball is centered
window=100

#Size of screen image
width=800
height=width*0.606

def callback(value):
	save_callibration(v1, v2, v3, v4, v5, v6)	
	
#Create bars to callibrate ball
def callibrate_setup( range_filter ):
	#Default values are given according to previous tests
	v = load_callibration()

	cv2.namedWindow("Trackbars", 0)
	
	aux=0
	for i in ["MIN", "MAX"]:
		for j in range_filter:
			cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", int(v[aux]), 255, callback)
			aux+=1

#Get values of callibration
def callibrate ( range_filter ):
	values = []

	for i in ["MIN", "MAX"]:
		for j in range_filter:
			v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
			values.append(v)
	return values

def save_callibration (v1, v2, v3, v4, v5, v6):
	f=open('callibration.txt','w')
	f.write(str(v1)+'\n')
	f.write(str(v2)+'\n')
	f.write(str(v3)+'\n')
	f.write(str(v4)+'\n')
	f.write(str(v5)+'\n')
	f.write(str(v6)+'\n')	
	f.close()

def load_callibration():
	try:
		f=open('callibration.txt','r')
		v1 = f.readline()	
		v2 = f.readline()	
		v3 = f.readline()	
		v4 = f.readline()	
		v5 = f.readline()	
		v6 = f.readline()	
		f.close()
	except:
		v1  = 0
		v2  = 0
		v3  = 0
		v4  = 255
		v5  = 255
		v6  = 255
	return int(v1), int(v2), int(v3), int(v4), int(v5), int(v6)

def fix_distortion (camera):
	mtx= np.array([[1307.664136, 0.000000, 953.434477], [0.000000, 1304.310096, 568.717493], [0.000000, 0.000000, 1.000000]])
	dist=np.array([-0.387577, 0.121987, -0.001898, -0.002118, 0.000000])
	
	#WEBCAM
	passed, fm=camera.read()
	h,  w = fm.shape[:2]
	newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

	# undistort
	frame = cv2.undistort(fm, mtx, dist, None, mtx)
	frame=fm
	# crop the image
	x,y,w,h = roi
	frame = frame[y:y+h, x:x+w]

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=width)
	return frame

def filtrate_image (frame, greenLower, greenUpper):
	blurred = cv2.GaussianBlur(frame, (17, 17), 0)
	blurred = cv2.medianBlur(blurred,15)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)#6 to 4
	#cv2.imshow("Filtrate", mask)
	#cv2.imshow("Frame2", mask)
	#mask = cv2.inRange(blurred, greenLower, greenUpper)#6 to 4
	

	mask = cv2.erode(mask, None, iterations=2)
	#cv2.imshow("Erode1", mask)
	mask = cv2.dilate(mask, None, iterations=4)
	#cv2.imshow("Dilate1", mask)
	mask = cv2.erode(mask, None, iterations=2)
	#cv2.imshow("Erode2", mask)
	return mask

def search_ball (mask):
	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE) [1]
	
	#Variables to store best result
	i=0
	i_max=0
	circularity_max=0
	area_max=0
				
	# check if contour is of circular shape	
	for con in cnts:
		#Get area and perimeter of each object
		perimeter = cv2.arcLength(con, True)
		area = cv2.contourArea(con)
		circularity = 4*math.pi*(area/(perimeter*perimeter))
		#print str(i) + ': '+str(area)+', '+str(circularity)
		#If object is circular enough, and is the biggest, store it
		if circularity>=circularity_thresh:
			if area>area_max:
				area_max=area
				circularity_max=circularity
				i_max=i
				#print str(i_max)+': Max area: '+str(area_max)+', Max circ: '+str(circularity_max)
		i=i+1

	return cnts,i_max, area_max, circularity_max


def alarm ():
	global tracking
	global start

	#Start timer
	start=rospy.get_rostime()

	#Send signal
	ball_controller.data=0
	publisher_controller.publish(ball_controller.data)

	#Set tracking signal
	tracking=1	

def false_alarm ():
	global tracking
	global start
	global now
	global dt
	global results
	global servos

	#Clear buffer
	results.clear()	
	
	#Clean timer
	start.secs=0
	now.secs=0
	dt=0	

	#Send false alarm signal
	ball_controller.data=1
	publisher_controller.publish(ball_controller.data)

	#Stop pantilt
	servos=0x00 #Restart servos				
	publisher_servos.publish(servos)

	#Stop rover
	follow.Velocity=0.0	
	follow.Turn=0.0	
	publisher_cmd.publish(follow)

	#Reset tracking flag
	tracking=0

def check_thresh (servos):
	global tracking
	global start

	if results.count(1)>thresh4:
		return servos
	elif results.count(1)>thresh3:
		servos = 0x30	
		cv2.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)
	elif results.count(1)>thresh2:
		servos = 0x20
		cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
	elif results.count(1)>thresh1:
		servos = 0x10
		cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 2)

		#If starting to track, alert gps to stop controlling
		if not tracking:
			alarm()
	else:
		servos = 0x00
		
		#False alarm
		if tracking:
			false_alarm()
	return servos

def follow_ball ():
	if profundity>min_profundity:
		follow.Velocity = max_velocity
	else:
		#Announce you found it
		ball_controller.data=2
		publisher_controller.publish(ball_controller.data)

		#Stop rover
		follow.Velocity=0.0	
		follow.Turn=0.0	
		publisher_cmd.publish(follow)
		print 'Ball found!!!'
		exit(0)

	thresh=10
	if pos.data > thresh or pos.data < -thresh:
		follow.Turn = (pos.data/90.0) *  max_turn


if __name__=="__main__":
	#Get WEBCAM image
	camera=cv2.VideoCapture(int(sys.argv[2]))

	if (sys.argv[1] == 'manual'):
		#Callibrate HSV
		hsv_callibrate=["H", "S", "V"]
		callibrate_setup(hsv_callibrate)
	elif (sys.argv[1] == 'auto'):
		v1, v2, v3, v4, v5, v6 = load_callibration()
		cv2.imshow("Frame", fix_distortion (camera))	
		key = cv2.waitKey(1) & 0xFF

	gps_controller.data = 0
	#Restart pantilt
	servos=0x00 #Restart servos				
	publisher_servos.publish(servos)

	while not rospy.is_shutdown():
		# grab the current frame
		try:
			if gps_controller.data == 1:
			
				#Update timer, send signal if over one minute passes
				if start.secs != 0:
					now=rospy.get_rostime()
					dt=now.secs-start.secs
				else:
					dt=0
			
				if dt > timeout or (results.count(1) < thresh1 and tracking):
					print "False alarm"
					false_alarm()	
			
				#Fix distortion	
				frame= fix_distortion (camera)
		
				#Get callibration values
				if (sys.argv[1] == 'manual'):
					v1, v2, v3, v4, v5, v6 = callibrate (hsv_callibrate)

				greenLower = (v1, v2, v3) 
				greenUpper = (v4, v5, v5) 

				#Filtrate image by color
				mask= filtrate_image(frame, greenLower, greenUpper)
		
				#Identify ball by form and size
				cnts, i_max, area_max, circularity_max = search_ball(mask)
			
				#Check if is constant
				if circularity_max > circularity_thresh:

					((x, y), radius) = cv2.minEnclosingCircle(cnts[i_max])
					servos = check_thresh(servos)
					result=1

					#Chek thresh of frame samples
			
					if results.count(1)>thresh4:
					
						servos = 0x00
						# only proceed if the radius meets a minimum size
						# draw the circle and centroid on the frame,
						# then update the list of tracked points
				
						cv2.circle(frame, (int(x), int(y)), int(radius), (255, 255, 255), 2)

						#profundity=1103.9*(radius)**(-1.131) #eq obtaine by excel
						profundity=(W*F)/(radius*2)
						cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
						cv2.putText(frame, "%.1f cm" % profundity, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
					
						if x<(width/2)-window:
							servos |= 0x0A
						elif x>(width/2)+window:
							servos |= 0x08
		    					servos &= ~0x02
						else:
							servos &= ~0x08
							servos |= 0x02

						if y<(height/2)-window:
							servos |= 0x05
						elif y>(height/2)+window:
							servos |= 0x04
		    					servos &= ~0x01
						else:
							servos &= ~0x04
							servos |= 0x01

						area_before=area
						print 'Profundity: ' + str(profundity)
						follow_ball ()
				else:

					follow.Velocity = 0.0
					#servos = 0x00
					result=0
		
				#Since it seems to be the ball a comparison against 100 frames is done
				if len(results)<samples:
					results.append(result)
				else:
					results.rotate(1)
					results[0]=result
				
				print '############## Data ##############'
				print 'Approved samples: ' + str(results.count(1))
				print 'Circularity: ' + str(circularity_max)
				print 'Area: ' + str(area_max)
				print 'Servo pos: '+str(pos.data)
				print 'Velocity : '+str(follow.Velocity)
				print 'Turn: '+str(follow.Turn)
				print 'Seconds: '+str(dt)

				if (pastServos != servos):
					pastServos=servos
					#Check quadrant of object
					if servos & 0x30 != 0x00:
					
						if x < (width/2):
							servos &= ~0x40
						else:
							servos |= 0x40

								
					publisher_servos.publish(servos)
				publisher_cmd.publish(follow)

				# show the frame to our screen
				if (sys.argv[1] == 'manual'):
					cv2.imshow("Frame", frame)
					cv2.imshow("Mask", mask)

				key = cv2.waitKey(1) & 0xFF
				if key  == 115:
					save_callibration(v1, v2, v3, v4, v5, v6)			

		except Exception as e:
			print(e) 
			pass
	# cleanup the camera and close any open windows
	servos=0x00	
	publisher_servos.publish(servos)
	camera.release()
	cv2.destroyAllWindows()
