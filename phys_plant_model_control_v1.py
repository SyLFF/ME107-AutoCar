import car_dir
car_dir.setup()
import motor
motor.setup()
motor.setSpeed(0)
import time
import serial
import numpy as np

# In command window, type ls dev/tty* and make sure ttyACM0 is there.
# If it is not, unplug and plug the Teensy back into the RPi.
ser = serial.Serial('dev/ttyACM0', 9600)

# Parameters that may end up being tuned
sens_dist = 3
prevError = 0

def volt2dist_right(voltage):
    if (voltage < 515):
        voltage = 1030 - voltage;
    voltage -= 515;
    if (voltage < 5):
        return 2.5
    right = np.log((voltage - 3.571) / 277.4) / -3.541 # This may need to casted to a float
    if (right < 0):
        right = 0
    return right

def volt2dist_left(voltage):
    if (voltage < 515):
        voltage = 1030 - voltage
    voltage -= 515
    if (voltage < 5):
        return 2.5;
    left = -np.log((voltage - .6978) / .04688) / 2.523 + 3 # This may need to be casted to a float
    if (left < 0):
        left = 0
    return left

def conv2err(x1, x2):
    return (np.power(x1, 2) - numpy.power(x2,2)) / (2 * sens_dist)
# what is thiserror supposed to be?  Is it the erorr in the heading angle for the car?
# I'm supposing that it's the error in the heading angle for the car, and if it's not,
# then we can convert this function to be exactly that.


# Uncomment delay if you want to run the car on the ground.
# time.sleep(20)

motor.setSpeed(40)
SampleTime = 1/60

while(True):
    raw_vals = ser.readline()
    split_vals = raw_vals.split(',')
    left_val_raw = int(split_vals[0]) # Make sure these two lines actually convert the string into an int
    right_val_raw = int(split_vals[1])
    err = conv2err(volt2dist_left(left_val_raw), volt2dist_right(right_val_raw))
    # Unless I'm mistaken, we have some sample speed associated with the Teensy (I believe
    # you mentioned it was 60 Hz, I'm supposing that)
    # Note that the output is the required steering angle for the car, so on top of the input calculated
    # by the function, we need to have a control system that maps steering angle input to actual
    # steering angle for the car, as I think that you mentioned that was a problem prior.
    errordt = (err-prevError)/SampleTime
    u = 36*err - 8.7*errordt
    prevError = err
    
    
