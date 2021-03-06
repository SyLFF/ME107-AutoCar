import car_dir
car_dir.setup()
import motor
motor.setup()
motor.setSpeed(0)
import time
import numpy as np
import cv2

from myFunctions import readyImage, splitImage, findCentroid, showRows, showCentroids, errorCalc

cap = cv2.VideoCapture(0)
cv2.namedWindow('frame')

kp = 36
ki = 0
kd = 8.7

sampleTime = .0166667
dt = sampleTime

totalTime = 0

prevError = 0
while(totalTime < 2500):
        totalTime += 1
        
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

	motor.forward()
        car_dir.turn(int(Map(np.pi/2 + u, 0, np.pi, 0, 255)))
        visData = open('visData.txt', 'a')
        visData.write(str(totalTime))
        visData.write(',')
        visData.write(str(error))
        visData.write(',')
        visData.write(str(errordt))
        visData.write(',')
        visData.write(str(u))
        visData.write(',')
        visData.write(raw_vals)
        visData.close()
	time.sleep(sampleTime)
