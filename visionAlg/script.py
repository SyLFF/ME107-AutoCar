import numpy as np 
import cv2
from myFunctions import readyImage, splitImage, findCentroid, showRows, showCentroids, errorCalc

cap = cv2.VideoCapture(0)
cv2.namedWindow('frame')

dt = sampleTime

prevError = 0
while(True):
	ret, frame = cap.read()

	numRows = 8
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
	
	error = errorCalc(frame, centroidArray, sampleTime)
	error = error[-1]

	errorSum += error * dt
	errordt = (error - prevError) / dt

	u = kp * error + ki * errorSum + kd * errordt
	prevError = error