import cv2
import cv2.aruco as aruco
import numpy as np

# Camera matrix parameters
ppx = 317.056182861328
ppy = 236.889175415039
fx = 607.182434082031
fy = 606.4169921875

# Camera matrix
camera_matrix = np.array([
    [fx, 0,  ppx],
    [0,  fy, ppy],
    [0,  0,  1]
])

# Distortion coefficients
distortion_coefficients = np.array([0,0,0,0,0])

# Define the ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
# Define the parameters for marker detection
parameters = aruco.DetectorParameters_create()


def dist_detection(frame, aruco_dict, parameters):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None and len(ids) >= 2:
        aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Physical size of the ArUco markers (in cm)
        marker_length = 3.0 
        
        # Estimate pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, distortion_coefficients)
        
        if len(ids) >= 2:
            # Get the translation vectors of the first two markers
            tvec1 = tvecs[0][0]
            tvec2 = tvecs[1][0]
            
            # Calculate the distance between the two markers in cm
            distance = np.linalg.norm(tvec1 - tvec2)
            
            # Draw a line between the two markers
            marker1_center = np.mean(corners[0][0], axis=0).astype(int)
            marker2_center = np.mean(corners[1][0], axis=0).astype(int)
            cv2.line(frame, tuple(marker1_center), tuple(marker2_center), (0, 255, 0), 2)
            
            # Display the distance
            cv2.putText(frame, f"Distance: {distance:.2f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    else:
        distance = -1.0


    return float(distance),frame

if __name__ == '__main__':



    # Initialize the webcam
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # detect aruco tags and draw distance on frame 
        distance,processed_img = dist_detection(frame, aruco_dict, parameters)

        # Display the resulting frame
        cv2.imshow('ArUco Marker Detection', processed_img)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()