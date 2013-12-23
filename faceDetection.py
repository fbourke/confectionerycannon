import cv2
import time
import numpy as np

#Reads the cascade file and turns it into something cv2 can understand
cascade = cv2.CascadeClassifier("./lbpcascade_frontalface_alt.xml")
#make sure to run this in the same folder as the cascade XML file

def detect(img,cascade):
    
    rects = cascade.detectMultiScale(img, 1.15, 2, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))
    #returns an array of rectangles around each face

    if len(rects) == 0:
        return [], img
    points = np.array([rects[:,0]+rects[:,2]/2, rects[:,1]+rects[:,3]/2])
    points.transpose()
    rects[:, 2:] += rects[:, :2]	#Converts from corner and dimension to corner and corner
    return rects, img


def box(rects, img):
    print rects
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
    return img

#def point(points,img):
#   for xc, yc in points:
#       cv2.circle(img, (xc,yc), 5, (255,0,0), 3)
#   return img

webcam = cv2.VideoCapture(1)

cv2.namedWindow("preview")

while True:
	rval, frame = webcam.read()
	rects, img = detect(frame,cascade)
	frame = box(rects, img)
	#frame = point(points,img)
	cv2.imshow("preview", frame)

	k = cv2.waitKey(10)

	if k == 27:	#If someone presses escape, quit
		break

