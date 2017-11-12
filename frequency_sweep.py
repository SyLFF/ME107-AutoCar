import numpy as np
import car_dir
import motor
def determine_steering_angle(time):
    freq = 10 - np.floor(time/200)
    return 127.5 + (14.2*np.cos(freq*3.14159/30*time))
cur_time = 0
while True:
    motor.forward()
    cur_time += 1
    car_dir.turn(determine_steering_angle(cur_time))
