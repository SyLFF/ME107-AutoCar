import numpy as np 
import cv2

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert to gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect edges 
    edges = cv2.Canny(gray,100,200)

    minLineLength = 80
    maxLineGap = 1
    lines = cv2.HoughLinesP(edges,1,np.pi/180,500,minLineLength,maxLineGap)
    if (lines != None):
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(edges,(x1,y1),(x2,y2),(0,255,0),2)

    # Display the resulting frame
    cv2.imshow('frame1', edges)
    cv2.imshow('frame2', gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()