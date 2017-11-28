import numpy as np
import cv2
import car_dir
car_dir.setup()
import motor
motor.setup()
motor.setSpeed(0)
import time
# from visionAlg.myFunctions import readyImage, splitImage, findCentroid
import RPi.GPIO as GPIO
import serial

ser = serial.Serial('/dev/ttyACM0', 9600)

def determine_steering_angle(time):
    freq = 10 - np.floor(time/200)
    return int((28.3*np.cos(freq*3.14159/250*time)))

cap = cv2.VideoCapture(0)
cv2.namedWindow('frame')

cur_time = 0

motor.setSpeed(40)
while(cur_time < 2500):
    line = ser.readline()
    magData = open('magData.txt', 'a')
    magData.write(str(cur_time))
    magData.write(',')
    magData.write(line)
    magData.close()
    motor.forward()
    cur_time += 1
    car_dir.turn(determine_steering_angle(cur_time)+20)
    time.sleep(1/60)
    if (cur_time % 200 == 0):
        motor.setSpeed(0)
        time.sleep(5)
        motor.setSpeed(40)
##    ret, frame = cap.read()
##
##    numRows = 10
##    processed = readyImage(frame)
##    imageRows = splitImage(processed, numRows)
##
##    centroidArray = []
##    centroidData = open('centroidData.txt', 'a')

##    for i in range(7, len(imageRows)):
##        xCentroid = findCentroid(imageRows[i])
##        centroidArray.append(xCentroid)
##        centroidData.write(str(xCentroid))
##        centroidData.write(',')
##
##    centroidData.write(str(cur_time))
##    centroidData.write('\n')
##    centroidData.close()
    
##    print("Hall Left")
##    print(GPIO.input(HALL_LEFT))
##    print("Hall Right")
##    print(GPIO.input(HALL_RIGHT))
    
motor.setSpeed(0)
