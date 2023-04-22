from cv2 import aruco
import cv2 as cv


class ARTag:

        def __init__(self, marker_dict, marker_size, marker_id, camera_matrix, dist_coeffs):
            self.marker_dict = marker_dict
            self.marker_size = marker_size
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs
            self.marker_id = marker_id

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
        
        # Draw the pose of the marker object
        def draw_pose(self, frame, rvec, tvec, length=4, thickness=4):
            cv.drawFrameAxes(frame, self.cam_matrix, self.coeffs, rvec, tvec, length, thickness)

        # Projects the given point onto this marker's coordinate frame, 
        def project_to_marker(self, points_real_world, rvec, tvec):
            [image_point, jacobian] = cv.projectPoints(points_real_world, rvec, tvec, self.camera_matrix, self.coeffs)
            cv.circle(self.frame, (int(image_point[0, 0]),int(image_point[0,1])), 2, (0, 100, 255), 8)
            return image_point


             

