import numpy as np 
import cv2
import matplotlib.pyplot as plt 
import car_dir
car_dir.setup()
import motor
motor.setup()
motor.setSpeed(0)
import time
from myFunctions import readyImage, splitImage, findCentroid, showRows, showCentroids, errorCalc

cap = cv2.VideoCapture(0)
cv2.namedWindow('frame')

motor.setSpeed(40)

prevCentroid = [0, 0] # fix all initialization values
prevAlpha = 0

dt = sampleTime
prevError = np.zeros()
while(True):
	ret, frame = cap.read()

	numRows = 20
	processed = readyImage(frame)
	imageRows = splitImage(processed, numRows)

	centroidArray = []
	for i in range(0, len(imageRows)):
		xCentroid = findCentroid(imageRows[i])
		centroidArray.append(xCentroid)

	showRows(frame, numRows)
	showCentroids(frame, numRows, centroidArray)

	cv2.imshow('frame', frame)
	cv2.waitKey(0)

	# PID controller
	error = errorCalc2(frame, centroidArray, sampleTime)

	prevCentroid = centroidArray[9]

	errorSum = []
	errordt = []
	u = []
	for i in range(0, len(error)):
		errorSum += error*dt
		errordt = (error - prevError) / dt
		u = kp * error + ki * errorSum + kd * errordt
		prevError = error
