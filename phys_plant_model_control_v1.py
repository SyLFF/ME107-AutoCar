import car_dir
car_dir.setup()
import motor
motor.setup()
motor.setSpeed(0)
import time
import serial
import numpy as np

# In command window, type ls /dev/tty* and make sure ttyACM0 is there.
# If it is not, unplug and plug the Teensy back into the RPi.
ser = serial.Serial('/dev/ttyACM0', 9600)

def Map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def volt2dist_right(voltage):
    if (voltage < 515):
        voltage = 1030 - voltage;
    voltage -= 515;
    if (voltage < 5):
        return 1.5
    right = np.log((voltage - 3.571) / 277.4) / -3.541 # This may need to casted to a float
    if (right < 0):
        right = 0
    return right

def volt2dist_left(voltage):
    if (voltage < 515):
        voltage = 1030 - voltage
    voltage -= 515
    if (voltage < 5):
        return 1.5;
    left = -np.log((voltage - .6978) / .04688) / 2.523 + 3 # This may need to be casted to a float
    if (left < 0):
        left = 0
    return left

def conv2err(x1, x2, tot_time):
    delta = (np.power(x1, 2) - np.power(x2,2)) / (2 * sens_dist)
    print(x1)
    print(x2)
##    if (tot_time % 5 == 0):
##        print("delta")
##        print(delta)
    if (np.power(sens_dist/2 + delta, 2) > np.power(x1, 2)):
        print("Hello")
        y = np.sqrt(-np.power(x1, 2) + np.power(sens_dist/2 + delta, 2))
    else :
        print("no hello")
        y = np.sqrt(np.power(x1, 2) - np.power(sens_dist/2 + delta, 2))
    if (delta < 0):
        des_alpha = np.arctan(y/-delta)
    else :
        des_alpha = np.pi - np.arctan(y/delta)
    return des_alpha

def saturate(x,ub,lb):
    if x>ub:
        return(ub)
    elif x<lb:
        return(lb)
    else:
        return(x)

# what is thiserror supposed to be?  Is it the erorr in the heading angle for the car?
# I'm supposing that it's the error in the heading angle for the car, and if it's not,
# then we can convert this function to be exactly that.


# Uncomment delay if you want to run the car on the ground.
car_dir.turn(25)
time.sleep(5)

tot_time = 0
start_time = 0
SampleTime = .01666667
lbase = -75    # numeric values for the turn limits of the car (lbase = left rbase = right)
rbase = 125
langle = np.pi/2-np.pi/6-0.25   # Angular limits for turning the car
rangle = np.pi/2+np.pi/6+0.25
c_alpha = Map(25,lbase,rbase,langle,rangle)  # Initial car-heading, set at straight ahead, 90 degrees
interror = 0
# Gain
Kp = 1.4 # used to be 1
Kd = 0  # used to be 120
Ki = 0
# Parameters that may end up being tuned
sens_dist = 3
prevError = 0
left_offset = -22
right_offset = -21
print('Starting Loop')

while(start_time<200):
    start_time += 1
    raw_vals = ser.readline()
    time.sleep(SampleTime)
motor.setSpeed(38)

while(tot_time < 600):
    tot_time += 1
    raw_vals = ser.readline()
    split_vals = raw_vals.split(',')
    left_val_raw = int(split_vals[0]) - left_offset # Make sure these two lines actually convert the string into an int
    right_val_raw = int(split_vals[1]) - right_offset
    des_alpha = conv2err(volt2dist_left(left_val_raw), volt2dist_right(right_val_raw), tot_time)
    error = des_alpha - c_alpha
    differror = (error-prevError)/SampleTime
    interror += error*SampleTime
    input_angle = saturate(Kp*error + Kd*differror + Ki*interror+c_alpha,rangle,langle)
#   if (abs(differror) < 16):
    c_alpha = input_angle
    prevError = error
    car_dir.turn(int(Map(input_angle,langle,rangle,lbase,rbase)))
    testData = open('testdata.txt','a')
    testData.write(str('\n \n current_time: {} \n des_alpha: {}, \n error: {}, \n differror: {}, \n interror: {}, \n input_angle: {}'.format(tot_time*SampleTime,des_alpha,error,differror,interror,c_alpha)))
    testData.close()
    print('\n des_alpha: {}, \n error: {}, \n differror: {}, \n interror: {}, \n input_angle: {} \n left_val_raw: {} \n right_val_raw: {}'.format(des_alpha,error,differror,interror,c_alpha,left_val_raw,right_val_raw))
    print(tot_time)
    
    time.sleep(SampleTime)

motor.setSpeed(0)
car_dir.turn(25)

