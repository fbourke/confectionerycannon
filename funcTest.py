import cv2
import numpy as np
import serial
import time
import types
from scipy.interpolate import interp1d


### SETUP ###
webcam = cv2.VideoCapture(1)
cv2.namedWindow("preview")

#Read the cascade file and make sense of it
cascade = cv2.CascadeClassifier("/home/elliottwyse/Desktop/haarcascade_frontalface_alt.xml")

#Generate the interpolation function for use later
xdata = np.array([24,35,43,50,55,60,72,85,100,115,130,180,235,300])
ydata = np.array([20,15,12,10,9,8,7,6,5,4,3,2.5,2,1.5])
f = interp1d(xdata,ydata)


def sendCommand(command, data):
	if command == 'fire':
		ser.write('f')
		ser.write('aa')
	elif command == 'pan':
		ser.write('p')
		ser.write(chr(data/256))	#Data[0] is the pan signal in degrees
		ser.write(chr(data%256))	
	elif command == 'tilt':
		ser.write('t')
		ser.write(chr(data/256))
		ser.write(chr(data%256))
	#elif command == 'range':
	#	ser.write('r')
	#	ser.write('aaaa')

def detect(img,cascade):
	#Find all the faces and return a list of their boxes and the image
	rects = cascade.detectMultiScale(img, 1.2, 2, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))

	if len(rects) == 0:
		return [], img
	rects[:, 2:] += rects[:, :2]	#Converts from corner and dimension to corner and corner 
	
	return rects, img

def closestCenter(rects, f):	#This function needs to be rewritten: Currently it will kill all but the last face
	centers = []
	picWidth = 600
	for x1, y1, x2, y2 in rects:
		#Find the center of each rectangle and how far away from centered it is
		xc = (x2-x1)/2+x1
		yc = (y2-y1)/2+y1
		offCenter = abs(xc-picWidth/2)
		#print offCenter
		
		#Find the distance of each face from the launcher
		width = abs(x1-x2)
		if width < 300 and width > 24:
			distance = f(width)
		elif width >= 300:
			distance = f(300)
		elif width <= 24:
			distance = f(24)

		#Compile calculated data into a tuple of values
		centers.append((offCenter,xc,yc,distance))

	#Pick the point that is closest to being centered in the frame
	#print len(centers)
	if len(centers) > 0:
		points = min(centers)
		#points = np.array([[(x2-x1)/2+x1, (y2-y1)/2+y1]])

		frame = point((points[1],points[2]), img)

		return points
	else:
		return []

def getValue():	
	#If there's new data, grab it, if not, return past value
	if ser.inWaiting() >= 5:	#Check that there is a full command in the buffer
		if ser.read() == 'o':	#Confirm that this is the first bit of the input

			#hi = ser.read()
			#low = ser.read()
			#rawDistance = ord(hi)*256 + ord(low)	#Combine bits
			#distance = rawDistance	#Need correction equation from voltage -> distance

			panPos = ord(hi)*256 + ord(low) - 150	#Totally centered occurs at 0 degrees

			return panPos

def panPID(paAngle):
	#Figure out what the control input should be to the servo based on the current pan angle between the face and the barrel
	pAngle = paAngle - 300

	controlSignal = 160 + pAngle / 4

	print controlSignal
	return controlSignal


def findAngles(faceCenters):
	#Turn pixel distances into real world angle things
	return faceCenters

def trajectory(distance, tiltAngle):
	#Some function that calculates the angle that the barrel should be tilted to in order to hit the mouth
	elevation = distance*np.sin(tiltAngle)
	length = distance*np.cos(tiltAngle)

	return tiltAngle

def point(points, img):
	#print points
	
	cv2.circle(img, points, 5, (255,0,0), 3)
	return img

target = 0
canFire = True
connected = False
threshold
while True: 

	valHold = getValue()

	if type(valHold) == types.NoneType:	#Checks to see if getValue got any data
		newData = False:	#If not, set flag to show not

	else:
		connected = True:
		newData = True:		#If yes, set flag to show yes 
		panPos = valHold	#and distribute data accordingly

	if connected:	#If we have ever received data from the arduino, start doing stuff

		if time.clock() > prevTime + 60:	#If we haven't shot recently, say we can fire again
			#prevTime = time.clock()
			canFire = True	

		if canFire == False:	#If we have shot recently, just pan back and forth

			if target == 0:

				if panPos < 0.5:
					target = 300

				else:
					sendCommand('pan', 165)

			elif target == 300:

				if panPos > 299.5:
					target = 0

				else:
					sendCommand('pan', 155)

		else:

			rval, frame = webcam.read()	#Get new image from webcam
			rects, img = detect(frame, cascade)	#Detect faces in image and return rectangles
			#print rects
			valHold = closestCenter(rects,f)	#Find the Center of the closest rectangle
			print valHold
			#k = cv2.waitKey(10)
			cv2.imshow("preview", frame)
			k = cv2.waitKey(10)

			if k == 27:	#If someone presses escape, quit
				break
			if len(valHold) > 0:
				offset, xcenter, ycenter, distance = valHold
				panAngle, tiltAngle = findAngles((xcenter,ycenter))	#Get the angle of the face relative to the current facing
				panFeedback = panPID(panAngle)	#Control system for deciding what to output 
			
					diff = panAngle	
				if diff < threshold:	#Compare the difference in angle to the threshold that defines when it is pointed at the target
					desiredAngle = trajectory(distance, tiltAngle)	#Find the desired tilt
					sendCommand('tilt', desiredAngle)	#Tell the arduino to go to that tilt

					if p1 == False:	#If we've just gone below the threshold for pan angle
						pastTime = time.clock()	#Start the timer
						p1 = True	#Flag that we're below the threshold for pan angle

					elif (time.clock() - pastTime) > 0.5:	#If we've had sufficient time to tilt
						sendCommand('fire', [])	#Tell arduino to fire
						canFire = False	#Set a flag to show we've fired recently
						p1 = False		#Reset the flag showing that we're in the pan threshold
						prevTime = time.clock()	#Set the timer for when we last fired

				else:	#If we leave the pan threshold
					p1 = False	#Set flag to show that we were out of the pan threshold
					#sendCommand('tilt', )