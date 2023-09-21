#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Pose, Quaternion

#little code to get human pose estimation 
# then get angles between limbs 
#precision and stability are meeeh... 
 
#someones have already done the work for me
import cv2
import mediapipe as mp
import numpy as np
import math
 
#mediapipe weird spells
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
width = 200
black_mode = False
 
 
class MarkWrapper:
    def __init__(self, mark):
        self.mark = mark
        self.pos = None
 
    def update_pos(self, landmarks):
        pos = landmarks[self.mark.value]
        self.pos = [pos.x, pos.y]
 
 
class Joint:
    def __init__(self, name, marks, angle_transform=None):
        if angle_transform is None:
            angle_transform = lambda x: x
        self.name = name
        self.marks = marks
        self.angle_transform = angle_transform
 
    def get_pos(self):
        return self.angle_transform(int(calculate_angle(*(m.pos for m in self.marks))))
 
 
#angular maths for nerds
def calculate_angle(a,b,c):
    a = np.array(a) #first
    b = np.array(b) #middle
    c = np.array(c) #last
 
    ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
    return ang % 360



def main():
    rclpy.init()
    node = rclpy.create_node("limbs_control")

    #detected_element_pub = node.create_publisher(PanelObject, "/HD/vision/distance_topic", 10)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(30)

    #video capture settings
    #my camera is 720p because I am stuck in the 90's
    cap = cv2.VideoCapture(13, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 780)

    try:
        with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
            print("aaaaaaaa2")
            PL = mp_pose.PoseLandmark
            thumb = MarkWrapper(PL.LEFT_THUMB)
            pinky = MarkWrapper(PL.LEFT_PINKY)
            index = MarkWrapper(PL.LEFT_INDEX)
            wrist = MarkWrapper(PL.LEFT_WRIST)
            elbow = MarkWrapper(PL.LEFT_ELBOW)
            shoulder = MarkWrapper(PL.LEFT_SHOULDER)
            left_hip = MarkWrapper(PL.LEFT_HIP)
            right_hip = MarkWrapper(PL.RIGHT_HIP)
            knee = MarkWrapper(PL.LEFT_KNEE)
            marks = [thumb, pinky, index, wrist, elbow, shoulder, left_hip, right_hip, knee]
            print("aaaaaaaa3")
            joints = [
                Joint("J1", [right_hip, left_hip, shoulder]),
                Joint("J2", [left_hip,shoulder , elbow]),
                Joint("J3", [shoulder, elbow, wrist]),
                Joint("J4", [shoulder, left_hip, knee]),
                Joint("J5", [elbow, wrist, index]),
                Joint("J6", [index, wrist, pinky]),
                Joint("GRIPPER", [thumb, wrist, index]),
            ]
            print("aaaaaaaa4")
            print("rclpy ", rclpy.ok())
            print("cap ", cap.isOpened())
            while rclpy.ok() and cap.isOpened():
                print("aaaaaaaa5")
                ret, frame = cap.read()
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False
                results = pose.process(image)
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
                # Get coordinates of Chakra's points
                try:
                    for mark in marks:
                        mark.update_pos(results.pose_landmarks.landmark)
        
                    # Giving cool angle names for a cool robot arm            
                    #artistic part here
                    start_point = (0, 0)
                    end_point = (width, 720)
                    color = (0, 0, 0)
                    thickness = -1
                    image = cv2.rectangle(image, start_point, end_point, color, thickness)
        
                    # Printing angles on body
                    for i, joint in enumerate(joints):
                        cv2.putText(image, str(joint.get_pos()), 
                                tuple(np.multiply(joint.marks[1].pos, [1280, 720]).astype(int)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                        )
        
                        cv2.putText(image, f"{joint.name}:   {joint.get_pos()}", 
                                (20, 50+50*i), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                        )
        
                except Exception as e:
                    print(e)
                    raise
                    pass
        
        
                # Some more art
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                        mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2), 
                                        mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2) 
                                        )               
        
                cv2.imshow('Mediapipe Feed', image)
        
                # press Q to rage quit
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break
        
                #press B for night mode
                if cv2.waitKey(10) & 0xFF == ord('b'):
                    if black_mode == False:
                        width = 1280
                        black_mode = True
                    else:
                        width = 200
                        black_mode = False
            
                rate.sleep()
        
            cap.release()
            cv2.destroyAllWindows()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()