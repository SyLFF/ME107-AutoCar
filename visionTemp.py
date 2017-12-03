import car_dir
car_dir.setup()
import motor
motor.setup()
motor.setSpeed(0)
import time
import numpy as np
import cv2

from visionAlg.myFunctions import readyImage, splitImage, findCentroid, showRows, showCentroids, errorCalc

def Map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def saturate(x,ub,lb):
    if x>ub:
        return(ub)
    elif x<lb:
        return(lb)
    else:
        return(x)

kp = 1
ki = 80
kd = 0

lbase = -70    # numeric values for the turn limits of the car (lbase = left rbase = right)
rbase = 120
langle = np.pi/2-np.pi/6    # Angular limits for turning the car
rangle = np.pi/2+np.pi/6

cap = cv2.VideoCapture(0)
# cv2.namedWindow('frame')

sampleTime = .0166667
dt = sampleTime

motor.setSpeed(55)
totalTime = 0
errorSum = 0
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

        # cv2.imshow('frame', frame)
        cv2.waitKey(1)

	# PID controller
	
        error = errorCalc(frame, centroidArray)
        error = error[-1]

        errorSum += error * dt
        errordt = (error - prevError) / dt

        u = kp * error + ki * errorSum + kd * errordt
        u = saturate(u, rangle, langle)
        prevError = error

        motor.forward()
        car_dir.turn(int(Map(u, langle, rangle, lbase, rbase)))

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
        visData.write('\n')
        visData.close()

        time.sleep(sampleTime)
