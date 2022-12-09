import cv2 as cv
from cv2 import aruco
import numpy as np







marker_dict_4 = aruco.Dictionary_get(aruco.DICT_4X4_50)
# marker_dict_4 = aruco.Dictionary_get(aruco.DICT_7X7_50)

#create detector parameters
param_markers = aruco.DetectorParameters_create()

cap_logi = cv.VideoCapture(2) # 1 Eylül

while True:
    ret, frame = cap_logi.read()
    if not ret:
        break
    #!!!
    frame = cv.resize(frame, (960, 540))

    gray_img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, rej = aruco.detectMarkers(gray_img, marker_dict_4, parameters=param_markers)

    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )

            ##aruco.drawDetectedMarkers(frame, corners)

            corners = corners.reshape(4, 2)
            corners = corners.astype(int)

            print(corners)

            top_right = corners[0]#.ravel()
            top_left = corners[1]#.ravel()
            bottom_right = corners[2]#.ravel()
            bottom_left = corners[3]#.ravel()
            cv.putText(
                frame,
                f"id: {ids[0]}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv.LINE_AA,
            )

            for x,y in corners:
                cv.circle(frame, (x,y), 4, (0,100, 255), 8)

            # print(ids, "  ", corners)
    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap_logi.release()
cv.destroyAllWindows()


