# import the necessary packages
import numpy as np
import argparse
import imutils
import cv2
 
# construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", required=True,
#	help="path to the input image")
# args = vars(ap.parse_args())
 
# load the image, convert it to grayscale, blur it slightly,
# and threshold it

#Start webcam capture
cap = cv2.VideoCapture(0)
cv2.namedWindow('frame')

while(True):
	# image = cv2.imread(args["image"])
	ret, frame = cap.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
	eroded = cv2.erode(thresh, kernel)

	# split into rows
	numRows = 10
	shape = eroded.shape
	height = shape[0]
	rowHeight = round(height/numRows)
	imRows = []
	ycount = 0
	for i in range(0, numRows):
		imRows.append(eroded[ycount: ycount + rowHeight, :])
		ycount += rowHeight

	# find contours in the thresholded image
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]

	# loop over the contours
	for c in cnts:
		# compute the center of the contour
		M = cv2.moments(c)

		if M['m00'] != 0:
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
		else:
			break
 
		# draw the contour and center of the shape on the image
		cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
		cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
		cv2.putText(frame, "center", (cX - 20, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
 
		# show the image
		cv2.imshow("Image", frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()