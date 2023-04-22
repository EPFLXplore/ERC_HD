from cv2 import aruco
import cv2 as cv


class ARTag:

        def __init__(self, marker_dict, marker_size, camera_matrix, dist_coeffs):
            self.marker_dict = marker_dict
            self.marker_size = marker_size
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs

        def detect(self, frame):
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            corners, ids, _rejectedImgPoints = aruco.detectMarkers(gray, self.marker_dict)
            if ids is not None:
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                return rvecs, tvecs, ids, corners
            return None, None, None, None
        
        def __str__(self) -> str:
             return f"ARTag(marker_dict={self.marker_dict},\
                        marker_size={self.marker_size},\
                        camera_matrix={self.camera_matrix},\
                        dist_coeffs={self.dist_coeffs})"