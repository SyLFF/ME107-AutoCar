import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import time

""" u(t) = Kp * e(t) + Ki \int_{0}^{t} + Kd {de}/{dt} """

def readyImage(frame):
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
	thresh = (255 - thresh)
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
	eroded = cv2.erode(thresh, kernel)
	return eroded

def splitImage(erodedImage, numRows):
	height, width = erodedImage.shape
	rowHeight = round(height / numRows)

	imRows = []
	ycount = 0
	for i in range(0, numRows):
		imRows.append(erodedImage[ycount: ycount + rowHeight, :])
		ycount += rowHeight
	return imRows

def findCentroid(erodedImage):
	im2, contours, heirarchy = cv2.findContours(erodedImage, 1, 2)

	if len(contours) != 0:
		M = cv2.moments(contours[0])

		if M['m00'] != 0:
			cx = int(M['m10'] / M['m00'])
			return cx
		else:
			return -1
	else:
		return -1

def showRows(image, numRows):
	height, width = image.shape
	rowHeight = round(height / numRows)

	for i in range(0, numRows):
		image[rowHeight*i, :] = [0, 255, 0]

def showCentroids(image, numRows, centroidArray):
	height, width = image.shape
	rowHeight = round(height / numRows)

	for i in range(0, numRows):
		y = rowHeight*i + round((rowHeight / 2))
		x = centroidArray[i]
		cv2.circle(image, (x,y), 10, [0, 255, 0], -1)

def errorCalc(image, centroidArray):
	height, width = image.shape

	errorArray = []
	for i in range(0, len(centroidArray)):
		errorArray.append(centroidArray[i] - (width/2))

	return errorArray

class PID:
	def __init__(self):
		self.Kp = 0
		self.Kd = 0
		self.Ki = 0

		self.Initialize()

	def setKp(self, invar):
		self.Kp = invar

	def setKd(self, invar):
		self.Kd = invar

	def setKi(self, invar):
		self.Ki = invar

	def setPrevError(self, prevError):
		self.prevError = prevError

	def Initialize(self):
		self.currTime = time.time()
		self.prevTime = self.currTime

		self. prevError = 0

		self.Cp = 0
		self.Cd = 0
		self.Ci = 0

	def Generate(self, error):
		self.currTime = time.time()
		dt = self.currTime - self.prevTime
		de = error - self.prevError

		self.Cp = self.Kp*error
		self.Cd = 0
		if dt > 0:
			self.Cd = de / dt

		self.Ci += error*dt

		self.prevTime = self.currTime
		self.prevError = error

		return self.Cp + (self.Ki*self.Ci) + (self.Kd*self.Cd)

	def update(self, error, dt):
		errorSum += error*dt
		errordt = (error - prevError) / dt

		u = kp * error + ki * errorSum + kd * errordt
		prevError = error