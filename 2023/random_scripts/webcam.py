import cv2
import numpy as np

# import cv2.aruco

from cv2 import aruco


def draw_lines(lines, img):
    # The below for loop runs till r and theta values
    # are in the range of the 2d array
    if not ( lines is None ):
        for r_theta in lines:
            arr = np.array(r_theta[0], dtype=np.float64)
            r, theta = arr
            # Stores the value of cos(theta) in a
            a = np.cos(theta)

            # Stores the value of sin(theta) in b
            b = np.sin(theta)

            # x0 stores the value rcos(theta)
            x0 = a*r

            # y0 stores the value rsin(theta)
            y0 = b*r

            # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
            x1 = int(x0 + 1000*(-b))

            # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
            y1 = int(y0 + 1000*(a))

            # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
            x2 = int(x0 - 1000*(-b))

            # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
            y2 = int(y0 - 1000*(a))

            # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
            # (0,0,255) denotes the colour of the line to be
            # drawn. In this case, it is red.
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)





cap = cv2.VideoCapture("/dev/video0")
# cap = cv2.VideoCapture("https://128.179.184.234:8080/video")

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, frame = cap.read()
    # frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    edges = cv2.Canny(gray,100,200)
    lines = cv2.HoughLines(edges, 1, np.pi/180, 100)
    draw_lines(lines, frame)

    # print(lines)
    # concatenate image Horizontally
    # Hori = np.concatenate((edges, frame), axis=1)
    cv2.imshow('Lines', frame)
    cv2.imshow('edges', edges)

    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()