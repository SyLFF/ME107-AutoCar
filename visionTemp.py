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

display = 1  # boolean to show whether display is connected

kp = 25
ki = 10
kd = 0.018

lbase = -50    # numeric values for the turn limits of the car (lbase = left rbase = right)
rbase = 100
langle = np.pi/2-np.pi/6    # Angular limits for turning the car
rangle = np.pi/2+np.pi/6
ledge = -4000   # edges of vision frame
redge = 4000

cap = cv2.VideoCapture(0)
if display == 1:
    cv2.namedWindow('frame')

sampleTime = .0166667
dt = sampleTime

motor.setSpeed(0)
totalTime = 0
errorSum = 0
prevError = 0
while(totalTime < 300):
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

        if display == 1:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)

        # PID controller
        error = errorCalc(frame, centroidArray)
        error = error[5]
        errorSum += error * dt
        errordt = (error - prevError) / dt

        u = kp * error + ki * errorSum + kd * errordt
        
        # u = saturate(u, rangle, langle)

        prevError = error
        motor.forward()
        # print 'u is: ', u
        # controlsignal = int(Map(np.pi/2 + u, 0, np.pi, 0, 255))
        controlsignal = int(Map(u, ledge, redge, lbase, rbase))
        print 'Control siganl is: ', controlsignal
        car_dir.turn(controlsignal)

        visData = open('visData3.txt', 'a')
        visData.write(str(totalTime))
        visData.write(',')
        visData.write(str(error))
        visData.write(',')
        # visData.write(str(errordt))
        # visData.write(',')
        visData.write(str(u))
        visData.write('\n')
        visData.close()

        time.sleep(sampleTime)

motor.setSpeed(0)
