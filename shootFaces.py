import cv2
import numpy as np
import serial
import time
import types
from scipy.interpolate import interp1d


### SETUP ###

stopVal = 81

#Initialize Serial Communications
ser = serial.Serial('/dev/ttyACM0',9600)
time.sleep(2) # let the Arduino reset
ser.flushInput()

#Read the cascade file and make sense of it
cascade = cv2.CascadeClassifier("/home/elliottwyse/Desktop/lbpcascade_frontalface.xml")

#Generate the interpolation functions for use later
#Range Interpolation
xdata = np.array([24,35,43,50,55,60,72,85,100,115,130,180,235,300])
ydata = np.array([20,15,12,10,9,8,7,6,5,4,3,2.5,2,1.5])
f = interp1d(xdata,ydata)

#Tilt Angle Interpolation
vpdata = [-238, 0, 238]
vadata = [np.arctan(-18./61), 0, np.arctan(18./61)]
fv = interp1d(vpdata, vadata)

#Horizontal Angle Interpolation
hpdata = [-205, 0, 205]
hadata = [np.arctan(-15./61), 0, np.arctan(15./61)]
fh = interp1d(hpdata, hadata)

webcam = cv2.VideoCapture(1)


def sendCommand(command, data):

	data = int(data)
	if command == 'fire':
		ser.write('f')
		ser.write('aa')
	elif command == 'pan':
		ser.write('p')
		ser.write(chr(data/256))	#Data[0] is the pan angle in degrees
		ser.write(chr(data%256))	
	elif command == 'tilt':
		ser.write('t')
		ser.write(chr(data/256))
		ser.write(chr(data%256))
	elif command == 'get':
		ser.write('g')
		ser.write('aa')
	#elif command == 'range':
	#	ser.write('r')
	#	ser.write('aaaa')

def detect(img,cascade):
	#Find all the faces and return a list of their boxes and the image
	rects = cascade.detectMultiScale(img, 1.2, 5, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))

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
	if len(centers) == 0:
		points = []
	else:
		points = min(centers)
	#points = np.array([[(x2-x1)/2+x1, (y2-y1)/2+y1]])
	print points
	return points

def getValue():	
	#If there's new data, grab it, if not, return past value
	if ser.inWaiting() >= 3:	#Check that there is a full command in the buffer
		if ser.read() == 'a':	#Confirm that this is the first bit of the input

			#hi = ser.read()
			#low = ser.read()
			#rawDistance = ord(hi)*256 + ord(low)	#Combine bits
			#distance = rawDistance	#Need correction equation from voltage -> distance

			potDegrees = 300	#The range of rotation for the potentiometer
			hi = ser.read()
			low = ser.read()
			panPos = ord(hi)*256 + ord(low)	#Combine bits
			return panPos

def panPID(pAngle):
	#Figure out what the control input should be to the servo based on the current pan angle between the face and the barrel
	pAngle = pAngle - 300

	controlSignal = stopVal + pAngle
	if controlSignal < 0:
		controlSignal = 0	
	return controlSignal


def findAngles(faceCenters, fv):
	#Turn pixel distances into real world angle things

	faceTiltRad = fv(faceCenters[1]-240)
	faceTilt = round(np.degrees(faceTiltRad))

	return faceTilt

def trajectory(distance, tiltAngle):
	#Some function that calculates the angle that the barrel should be tilted to in order to hit the mouth
	elevation = distance*np.sin(tiltAngle)
	length = distance*np.cos(tiltAngle)

	return tiltAngle

def tiltCompensation(tiltAngle, baseAngle):
	compAngle = (tiltAngle - baseAngle)*3
	if compAngle < 0:
		compAngle = 0
	return compAngle

def updateTilt(tiltCommand, position, lastCheck):
	rateMult = 1
	roc = (tiltCommand - position) * rateMult
	delta = time.clock() - lastCheck
	newPosition = roc * delta + position

	return newPosition



target = 0
canFire = True
connected = False
newData = False
prevTime = time.clock()
threshold = 5
desiredAngle = 0
lCheck = time.clock()
tiltPosition = 0
baseAngle = 10
rval,frame = webcam.read()

while True: 


	cv2.imshow("preview", frame)



	k = cv2.waitKey(10)

	if k == 27:	#If someone presses escape, quit
		break

	sendCommand('get',0)
	valHold = getValue()
	#print valHold

	tiltPosition = updateTilt(desiredAngle, tiltPosition, lCheck)
	lCheck = time.clock()

	if type(valHold) == types.NoneType:	#Checks to see if getValue got any data
		newData = False	#If not, set flag to show not

	else:
		connected = True
		newData = True	#If yes, set flag to show yes 
		panPos = valHold	#and distribute data accordingly

	if connected:	#If we have ever received data from the arduino, start doing stuff

		if time.clock() > prevTime + 60:	#If we haven't shot recently, say we can fire again
			#prevTime = time.clock()
			canFire = True	

		if canFire == False:	#If we have shot recently, just pan back and forth
			time.sleep(.3)
			desiredAngle = 0
			sendCommand('tilt', desiredAngle)
			if target == 0:

				if panPos < 0.5:
					target = 300

				else:
					sendCommand('pan', stopVal + 10)

			elif target == 300:

				if panPos > 299.5:
					target = 0

				else:
					sendCommand('pan', stopVal - 10)

		else:
			rval, frame = webcam.read()	#Get new image from webcam
			rects, img = detect(frame,cascade)	#Detect faces in image and return rectangles
			if not (len(rects) == 0):
				offset, xcenter, ycenter, distance = closestCenter(rects,f)	#Find the Center of the closest rectangle
				tiltAngle = findAngles((xcenter,ycenter), fv)	#Get the angle of the face relative to the current facing
				panFeedback = panPID(xcenter)	#Control system for deciding what to output 
				sendCommand('pan', panFeedback)	#Tell the arduino to pan to the angle of the face
				#print panFeedback
				desiredAngle = trajectory(distance, tiltAngle)	#Find the desired tilt
				signalAngle = tiltCompensation(desiredAngle, baseAngle)
				print desiredAngle
				print signalAngle
				sendCommand('tilt', signalAngle)	#Tell the arduino to go to that tilt

				if abs(tiltPosition - desiredAngle) < threshold and offset < threshold:
					sendCommand('fire',0)
					print 'Fire'
					canFire = False
					prevTime = time.clock()







				"""
				diff = panAngle	
				if diff < threshold:	#Compare the difference in angle to the threshold that defines when it is pointed at the target
					
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
				"""
