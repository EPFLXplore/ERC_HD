
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8

import pygame
import threading
import sys

import math

class LoadData(Node):
    def __init__(self):
        super().__init__('gui_subscriber')

        self.joint_angles = [0] * 6
        self.joint_speeds = [0] * 6
        self.joint_efforts = [0] * 6
        self.eef_position = [0]*3
        self.eef_quat = [0]*4
        #self.gripper_opening = 0

        self.create_subscription(
            JointState,
            '/HD/motor_control/joint_telemetry',
            self.joints_callback,
            10)

        self.create_subscription(
            Pose,
            '/HD/kinematics/eef_pose',
            self.eef_callback,
            10)
        
        '''
        self.create_subscription(
            Pose,
            '/gripper_opening',
            self.gripper_opening_callback,
            10)
        '''

    def joints_callback(self, msg):
        self.joint_angles = msg.position
        self.joint_speeds = msg.velocity
        self.joint_efforts = msg.effort
        
    
    def eef_callback(self,msg):
        self.eef_position = [msg.position.x,msg.position.y,msg.position.z]
        self.eef_quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]

    '''
    def gripper_opening_callback(self,msg):
        self.gripper_opening = msg
    '''
        

class Publisher(Node):
    def __init__(self):
        super().__init__("gui_publisher")
        self.mode_pub = self.create_publisher(Int8, '/ROVER/HD_mode', 10)

    def publish_mode(self, mode):
        msg = Int8()
        msg.data = mode
        self.mode_pub.publish(msg)
        print(f"Published mode: {mode}")


class DisplayData():
    def __init__(self, loader,publisher):

        # initialise subscriber and publisher classes
        self.loader = loader
        self.publisher = publisher

        # Size of window
        self.window_width = 2000
        self.window_height = 1200

        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)

        # Initialize modes
        self.modes = [-1, 1, 0,2, 3]
        self.current_mode = -1
        self.menu_open = False

    def setup(self):
        '''displays the pygame window'''
        self.window = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("GUI HD")
        self.font = pygame.font.SysFont("arial", 28, False, False)
        self.window.fill(self.BLACK)

    def draw_mode_button(self,x,y):
        '''displays the current mode, and all the options if the button is pressed
        Args:
            x,y: position of the top left corner of the button
        '''
        self.button_rect = pygame.Rect(x, y, 200, 50)
        pygame.draw.rect(self.window, self.WHITE, self.button_rect)
        self.window.blit(self.font.render(f"MODE: {self.current_mode}", True, self.BLACK), (self.button_rect.x + 20, self.button_rect.y + 10))

        if self.menu_open: 
            menu_rect = pygame.Rect(self.button_rect.x, self.button_rect.y + self.button_rect.height, self.button_rect.width, len(self.modes) * self.button_rect.height)
            pygame.draw.rect(self.window, self.WHITE, menu_rect)
            for i, mode in enumerate(self.modes):
                mode_rect = pygame.Rect(self.button_rect.x, self.button_rect.y + (i + 1) * self.button_rect.height, self.button_rect.width, self.button_rect.height)
                pygame.draw.rect(self.window, self.WHITE, mode_rect)
                self.window.blit(self.font.render(f"MODE: {mode}", True, self.BLACK), (mode_rect.x + 20, mode_rect.y + 10))

    def check_mode_button_press(self, pos):
        '''
        Checks if the button is pressed, and publish the new mode if one is selected

        Args:
            pos: point where the user clicked
        '''
        if self.button_rect.collidepoint(pos): # If point where clicked is within the button area
            self.menu_open = not self.menu_open 
        elif self.menu_open: 
            for i, mode in enumerate(self.modes):
                mode_rect = pygame.Rect(self.button_rect.x, self.button_rect.y + (i + 1) * self.button_rect.height, self.button_rect.width, self.button_rect.height)
                if mode_rect.collidepoint(pos):
                    self.current_mode = mode
                    self.menu_open = False
                    print(f"MODE changed to: {self.current_mode}")
                    self.publisher.publish_mode(self.current_mode)
                    break

    def display_joint_box(self, x, y):
        '''Displays the joints informations (n°,angle,rate angle,effort)
        Args:
            x,y: position of the top left corner of the joints box'''
        
        x_offset = 200
        y_offset = 50
        box_width, box_height = 800, 7 * y_offset
        pygame.draw.rect(self.window, self.WHITE, (x-10, y-10, box_width, box_height), 2)

        self.window.blit(self.font.render("Joint N°", True, self.WHITE), (x, y))
        self.window.blit(self.font.render("Angle", True, self.WHITE), (x + 200, y))
        self.window.blit(self.font.render("Speed", True, self.WHITE), (x + 2 * 200, y))
        self.window.blit(self.font.render("Effort ", True, self.WHITE), (x + 3 * 200, y))

        for i in range(6):
            self.window.blit(self.font.render(str(i + 1), True, self.WHITE), (x + x_offset / 4, y + y_offset * (i + 1)))
            self.window.blit(self.font.render(f"{math.degrees(self.loader.joint_angles[i]): .3f}°", True, self.WHITE), (x + 1* x_offset , y + y_offset * (i + 1)))
            self.window.blit(self.font.render(f"{math.degrees(self.loader.joint_speeds[i]): .3f}°/s", True, self.WHITE), (x + 2 * x_offset , y + y_offset * (i + 1)))
            self.window.blit(self.font.render(f"{self.loader.joint_efforts[i]:.3f}", True, self.WHITE), (x + 3 * x_offset , y + y_offset * (i + 1)))



    def quat_to_euler(self,quaternion):
        '''
        Transforms a quaternion (x,y,z,w) into corresponding euler angles (roll,pitch,yaw)
        '''
        x,y,z,w = quaternion[0], quaternion[1],quaternion[2],quaternion[3]
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        euler_angles= [roll,pitch,yaw]
        return euler_angles
    
    def display_eef_box(self, x, y):
        '''Diplays the end effector informations (position, euler angles)
        Args:
            x,y=position of the top left corner of the eef box'''

        x_offset = 200
        y_offset = 50
        box_width, box_height = 450, 4 * y_offset

        
        pygame.draw.rect(self.window, self.WHITE, (x-10, y-10, box_width, box_height), 2)

        self.window.blit(self.font.render("EEF position", True, self.WHITE), (x, y))
        self.window.blit(self.font.render("EEF euler angles", True, self.WHITE), (x + 200, y))

        eef_euler = self.quat_to_euler(self.loader.eef_quat)
        for i in range(3):
            self.window.blit(self.font.render(f"{self.loader.eef_position[i]: .3f}", True, self.WHITE), (x + x_offset / 4, y + y_offset * (i + 1)))
            self.window.blit(self.font.render(f'{math.degrees(eef_euler[i]):.3f}°', True, self.WHITE), (x + 5 * x_offset / 4, y + y_offset * (i + 1)))


    def display_gripper_opening(self,x,y):
        '''
        Display the gripper opening value and a visualization of the opening compared to the max and min openings
        Args: 
            x,y: position of the top left corner of the box
        '''
        scale = 300
        min_opening,max_opening = 0.3*scale,1.5*scale
        gripper_opening = 0.4 # PLACEHOLDER OF: self.loader.gripper_opening

        box_width, box_height = 490,200
        y_offset = 100
        
        self.window.blit(self.font.render(f"Gripper opening: {gripper_opening:.3f}", True, self.WHITE), (x, y))
        pygame.draw.line(self.window,self.RED,(x,y+y_offset),(x+(max_opening-min_opening)/2,y+y_offset),5)
        pygame.draw.line(self.window,self.RED,(x+ (max_opening+min_opening)/2 ,y+y_offset),(x+max_opening,y+y_offset),5)

        pygame.draw.line(self.window,self.GREEN,(x+(max_opening-gripper_opening*scale)/2,y+y_offset-50),(x+(max_opening-gripper_opening*scale)/2,y+y_offset+50),5)
        pygame.draw.line(self.window,self.GREEN,(x+(max_opening+gripper_opening*scale)/2,y+y_offset-50),(x+(max_opening+gripper_opening*scale)/2,y+y_offset+50),5)

        pygame.draw.rect(self.window, self.WHITE, (x-10, y-10, box_width, box_height), 2)


def ros_thread(loader, running_event,stop_event):
    '''Creates a parallel thread to subscribe and publish on ros2 topics'''
    while running_event.is_set() and not stop_event.is_set():
        rclpy.spin_once(loader, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    pygame.init()
    loader = LoadData()
    publisher = Publisher()
    
    
    running_event = threading.Event()
    stop_event = threading.Event()
    running_event.set()

    thread = threading.Thread(target=ros_thread, args=(loader, running_event,stop_event))
    thread.start()

    display = DisplayData(loader,publisher)
    display.setup()

    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                display.check_mode_button_press(event.pos)

        display.window.fill(display.BLACK)

        display.draw_mode_button(30,30)
        display.display_joint_box(30,150)
        display.display_eef_box(30,550)
        display.display_gripper_opening(30,800)
        pygame.display.flip()

    # A bunch of things to try to quit pygame and ros thread
    pygame.display.quit()
    stop_event.set()  # Signal the thread to stop immediately
    running_event.clear()
    #thread.join()
    loader.destroy_node()
    rclpy.shutdown()
    print("shutdown")
    sys.exit()
    
if __name__ == '__main__':
    main()
