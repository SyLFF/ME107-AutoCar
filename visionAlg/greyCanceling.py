""" Goal of this file is to test ways to see if there is any other part of the image
    present. IE the floor instead of the track background
"""


# Determine what range of colors are present in track and background
# import the necessary packages
import argparse
import cv2
import numpy as np


# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
refPt = []
cropping = False


def click_and_crop(event, x, y, flags, param):
    # grab references to the global variables
    global refPt, cropping

    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt = [(x, y)]
        cropping = True

    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        refPt.append((x, y))
        cropping = False

        # draw a rectangle around the region of interest
        cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
        cv2.imshow("image", image)



# construct the argument parser and parse the arguments
imToRead = "/Users/keenanrodewald/PycharmProjects/CV_tests/Keenan/Centroid Approaches/turn1.png"

# load the image, clone it, and setup the mouse callback function
image = cv2.imread(imToRead)
clone = image.copy()
cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)

# keep looping until the 'q' key is pressed
while True:
    # display the image and wait for a keypress
    cv2.imshow("image", image)
    key = cv2.waitKey(1) & 0xFF

    # if the 'r' key is pressed, reset the cropping region
    if key == ord("r"):
        image = clone.copy()

    # if the 'c' key is pressed, break from the loop
    elif key == ord("c"):
        break

# if there are two reference points, then crop the region of interest
# from teh image and display it
if len(refPt) == 2:
    roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]



    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    maxVal = np.max(gray)
    minVal = np.min(gray)

    print("Max greyscale val = " + str(maxVal))
    print("Min greyscale val = " + str(minVal))





# close all open windows
cv2.destroyAllWindows()

