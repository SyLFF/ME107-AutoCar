import numpy as np
import cv2

#Start webcam capture
cap = cv2.VideoCapture(0)
cv2.namedWindow('frame')

while(True):
    # get current frame frmo video capture
    ret, frame = cap.read()

    # first greyscale, then threhold and erode the image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    thresh = (255 - thresh)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    eroded = cv2.erode(thresh, kernel)

    # split into rows
    numRows = 10
    height, width = eroded.shape
    rowHeight = round(height/numRows)
    imRows = []
    ycount = 0
    for i in range(0, numRows):
        imRows.append(eroded[ycount: ycount + rowHeight, :])
        ycount += rowHeight

    # find centroids of each row
    centroidArr = []
    for i in range(0, len(imRows)):
        # im2, contours, hierarchy = cv2.findContours(imRows[i], 1, 2)
        img, contours = cv2.findContours(imRows[i],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        print contours

        if contours.any:
            M = cv2.moments(contours[0])
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                print cx
            else:
                break
        else:
            break

    # for i in range(0, len(imRows)):
        xCentroid = cx[i]
        centroidArr.append(xCentroid)

    shape = frame.shape
    height = shape[0]
    rowHeight = round(height / numRows)
    for i in range(0, numRows):
        frame[rowHeight*i, :] = [0, 255, 0]

    for i in range(0, numRows -1):
        y = rowHeight*i + round((rowHeight / 2))
        x = centroidArr[i]
    cv2.circle(frame, (x,y), 10, [0, 255, 0], -1)

    cv2.imshow('frame', frame)

    """ converts image back to color and draws found contours
    color = cv2.cvtColor(bottomRow, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(color, contours[0], -1, (0, 255, 0), 3)
    cv2.imshow('frame', color) """