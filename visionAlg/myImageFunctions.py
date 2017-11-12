import numpy as np
import cv2

def splitImageRows(image, numRows):
    # Find length and height of input image
    shape = image.shape
    height = shape[0]
    rowHeight = round(height / numRows)
    # Create imRows list and start a counter for y position
    imRows = []
    ycount = 0
    # Iterate through each row and append each new row image to imRows list
    for i in range(0, numRows ):
        imRows.append(image[ycount: ycount + rowHeight, :])
        ycount += rowHeight
    return imRows

def threshErode(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    eroded = cv2.erode(thresh, kernel)
    return eroded

def adaptiveThreshErode(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

    # Need to invert because original image was black line on white background
    thresh = (255 - thresh)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    eroded = cv2.erode(thresh, kernel)
    return eroded

# initializer for using webcam : not sure if this will work since cap may only be defined within this script.
def startWebcam(self):
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('frame')

# converts each border pixel to white
# TODO: Check if this works to close features for feature detection
def whiteBorderGreyIm(image):
    imHeight = image.shape[0]
    imWidth = image.shape[1]
    # top row
    image[0, 0:imWidth] = 255
    # right row
    image[0:imHeight, imWidth - 1,] = 255
    # bottom row
    image[imHeight-1, 0:imWidth-1,] = 255
    # left row
    image[0:imHeight-1, 0] = 255

# Finds x centroid of input image. Input image should be a binary thresholded image. Recomended to erode the thresholded image
# If no contours can be found, then centroid returns as -1
def findCentroid(threshIm):
    im2, contours, hierarchy = cv2.findContours(threshIm, 1, 2)

    # If contours is not empty
    if contours:

        M = cv2.moments(contours[0])

        #checks for error in Moment feedback.
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            return cx

        else:
            return -1

    else:
        return -1

def imToCentroidArray(image, numRows):
    threshedEroded = threshErode(image)
    # split into rows
    rows = splitImageRows(threshedEroded, numRows)

    # centroid array contains the centroid of each row. From top row to bottom row.
    centroidArr = []
    for i in range(0, len(rows)):
        xCentroid = findCentroid(rows[i])
        centroidArr.append(xCentroid)

    return centroidArr



def showRows(image, numRows):
    # Find length and height of input image
    Imshape = image.shape
    height = Imshape[0]
    rowHeight = round(height / numRows)
    # Iterate through each row and draw line at each rowDivision
    for i in range(0, numRows):
        image[rowHeight*i, :] = [0, 255, 0]

def showCentroids(image, numRows, centroidArr):
    # Find length and height of input image
    shape = image.shape
    height = shape[0]
    rowHeight = round(height / numRows)
    # Iterate through each row and draw line at each rowDivision
    for i in range(0, numRows ):
        y = rowHeight*i + round((rowHeight / 2))
        x = centroidArr[i]
        cv2.circle(image, (x,y), 10, [0, 255, 0], -1)

def centroidToError(centroidArr, originalImage):
    imShape = originalImage.shape
    imWidth = imShape[1]

    errorArr = []
    for i in range(0, len(centroidArr)):
        errorArr.append(centroidArr[i] - (imWidth /2))

return errorArr