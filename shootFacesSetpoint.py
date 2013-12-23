
import cv2
import numpy as np
import serial
import time
import types
from scipy.interpolate import interp1d


### SETUP ###

stopVal = 80

#Initialize Serial Communications
ser = serial.Serial('/dev/ttyACM0',38400)
time.sleep(2) # let the Arduino reset
ser.flushInput()

#Read the cascade file and make sense of it
cascade = cv2.CascadeClassifier("lbpcascade_frontalface.xml")

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
interpEndH = 350
interpEndA = 30.
hpdata = [-interpEndH, 0, interpEndH]
hadata = [np.arctan(-interpEndA/61), 0, np.arctan(interpEndA/61)]
fh = interp1d(hpdata, hadata)

#Tilt Interpolation
ddata = np.array([1,30])
angdata = np.array([10,60])
ftilt = interp1d(ddata,angdata)

webcam = cv2.VideoCapture(2)


def sendCommand(command, data):

	data = int(data)
	if command == 'fire':
		ser.write('f')
		ser.write('aa')
		print 'fire'
	elif command == 'flag':
		ser.write('c')
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
	ser.flush()
	#elif command == 'range':
	#	ser.write('r')
	#	ser.write('aaaa')

def detect(img,cascade):
	#Find all the faces and return a list of their boxes and the image
	rects = cascade.detectMultiScale(img, 1.15, 6, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))

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

			potDegrees = 300	#The range of rotation for the potentiometer
			hi = ser.read()
			low = ser.read()
			panPos = ord(hi)*256 + ord(low)	#Combine bits
			return panPos

def findAngles(faceCenters, fv, fh):
	#Turn pixel distances into real world angle things

	faceTiltRad = fv(faceCenters[1]-240)
	faceTilt = int(round(np.degrees(faceTiltRad)))

	#print faceCenters[0] - 300
	facePanRad = fh(faceCenters[0] - 300)
	facePan = int(round(np.degrees(facePanRad)) + 40)

	return facePan, faceTilt

def trajectory(distance, tiltAngle,ftilt):
	#Some function that calculates the angle that the barrel should be tilted to in order to hit the mouth
	elevation = abs(distance*np.sin(tiltAngle))
	length = abs(distance*np.cos(tiltAngle))
	print distance
	print length
	if length < 1:
		length = 1
	tiltAngle = ftilt(length)

	return tiltAngle

def tiltCompensation(tiltAngle, baseAngle):
	compAngle = (tiltAngle)*3 - baseAngle
	if compAngle < 0:
		compAngle = 0
	return compAngle

def updateTilt(tiltCommand, position, lastCheck):
	rateMult = 1
	roc = (tiltCommand - position) * rateMult
	delta = time.time() - lastCheck
	newPosition = roc * delta + position

	return newPosition

def box(rects,img):
	for x1, y1, x2, y2 in rects:
		cv2.rectangle(img, (x1,y1), (x2,y2), (0,255,0),2)
	return img

prevTime = time.time()
threshold = 5
tiltPosition = 0
baseAngle = -13
rval,frame = webcam.read()
lCheck = time.time()
lastShot = time.time()
desiredAngle = 0

while True: 

	canFire = time.time() - 3 > lastShot

	#sendCommand('get',0)
	#print getValue()

	if not (frame == None):


		cv2.imshow("preview", frame)

		k = cv2.waitKey(10)

		if k == 27:	#If someone presses escape, quit
			break

		tiltPosition = updateTilt(desiredAngle, tiltPosition, lCheck)
		lCheck = time.time()

		rval, frame = webcam.read()
		sendCommand('flag', 0)
		rects, img = detect(frame,cascade)
		frame = box(rects,img)



		print len(rects)
		if not (len(rects) == 0):

			offset, xcenter, ycenter, distance = closestCenter(rects,f)
			panAngle, tiltAngle = findAngles((xcenter, ycenter), fv, fh)
		
			print panAngle
			sendCommand('pan', panAngle)

			desiredAngle = trajectory(distance, tiltAngle, ftilt)
			signalAngle = tiltCompensation(desiredAngle, baseAngle)

			sendCommand('tilt', signalAngle)
			#print offset
			if abs(tiltPosition - desiredAngle) < threshold and offset < threshold:
				if canFire == True:
					sendCommand('fire', 0)
					lastShot = time.time()


