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

def conv2err(x1, x2, tot_time):
    delta = (np.power(x1, 2) - np.power(x2,2)) / (2 * sens_dist)
##    print('penis')
##    print(x1)
##    if (tot_time % 5 == 0):
##        print(delta)
    if (np.power(sens_dist/2 + delta, 2) > np.power(x1, 2)):
        y = np.sqrt(-np.power(x1, 2) + np.power(sens_dist/2 + delta, 2))
    else :
        y = np.sqrt(np.power(x1, 2) - np.power(sens_dist/2 + delta, 2))
    des_alpha = np.arctan2(y,delta)
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
time.sleep(1)

tot_time = 0
SampleTime = .01666667
lbase = -70    # numeric values for the turn limits of the car (lbase = left rbase = right)
rbase = 120
langle = np.pi/2-np.pi/6    # Angular limits for turning the car
rangle = np.pi/2+np.pi/6
c_alpha = Map(25,lbase,rbase,langle,rangle)  # Initial car-heading, set at straight ahead, 90 degrees
interror = 0
# Gain
Kp = 1 # used to be 36
Kd = 0  # used to be -8.7
Ki = 120
# Parameters that may end up being tuned
sens_dist = 3
prevError = 0
left_offset = 15
right_offset = 16
motor.setSpeed(55)
print('Starting Loop')

while(tot_time < 200):
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
    c_alpha = input_angle
    prevError = error
    testData = open('testdata.txt','a')
    testData.write(str('\n des_alpha: {}, \n error: {}, \n differror: {}, \n interror: {}, \n input_angle: {}'.format(des_alpha,error,differror,interror,c_alpha)))
    testData.close()
    print('\n des_alpha: {}, \n error: {}, \n differror: {}, \n interror: {}, \n input_angle: {}'.format(des_alpha,error,differror,interror,c_alpha))
    car_dir.turn(int(Map(input_angle,langle,rangle,lbase,rbase)))
    time.sleep(SampleTime)
#    print(left_val_raw)
#    print(',')
#    print(right_val_raw)
#    print(',')
#    print(des_alpha)
#    print(',')
#    print(err)
#    print(',')
#    prev_alpha = des_alpha
##   if (tot_time % 5 == 0):
##      print('left')
##       print(left_val_raw)
##       print('right')
##       print(right_val_raw)
#   prev_alpha = des_alpha
#     Unless I'm mistaken, we have some sample speed associated with the Teensy (I believe
#     you mentioned it was 60 Hz, I'm supposing that)
#     Note that the output is the required steering angle for the car, so on top of the input calculated
#     by the function, we need to have a control system that maps steering angle input to actual
#     steering angle for the car, as I think that you mentioned that was a problem prior.
#   print(SampleTime)
#   if (tot_time % 5 == 0):
#        print(err)
#    prevError = err
#    motor.forward()
###    angle = float(Map(tot_time,0,rbase-lbase,langle,rangle))
###    print(angle)
###    print('---')
###    print(tot_time)
###    print('---')
###    car_dir.turn(int(Map(satinput,langle,rangle,lbase,rbase)))
#    print(des_alpha)
#    time.sleep(SampleTime)
#    magData = open('magData.txt', 'a')
#    magData.write(str(tot_time))
#    magData.write(',')
#    magData.write(str(des_alpha))
#    magData.write(',')
#    magData.write(str(err))
#    magData.write(',')
#    magData.write(str(u))
#    magData.write(',')
#    magData.write(raw_vals)
#    magData.close()
#    time.sleep(SampleTime)

motor.setSpeed(0)

