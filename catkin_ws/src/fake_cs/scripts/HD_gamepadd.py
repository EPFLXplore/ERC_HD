'''
@File  :

@Author: Emma Gaia Poggiolini
'''
import evdev
from   evdev           import *

import rospy
import sys

from   threading       import Thread

#decomment
#from   CS_node         import CS
from   keyMap          import *

from geometry_msgs.msg import Twist
from std_msgs.msg      import Int8MultiArray, Int8


'''
Class Gamepad

@Description: Thread to

@Attributes:
    -

'''

## To Launch File Independently - with NAV commands
#     0. delete  from   CS_node         import CS
#     1. delete  cs  from the arguments of __init__( )  and comment  self.cs = cs
#     2. add in __init__( ):
#           rospy.init_node("CONTROL_STATION", anonymous=True)
#           self.Nav_Joystick_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#           self.HD_Angles_pub    = rospy.Publisher('HD_joints', Int8MultiArray, queue_size=1)
#           self.HD_voltmeter_pub = rospy.Publisher('HD_oltmeter', Int8, queue_size=1)
#           self.HD_mode_pub      = rospy.Publisher('HD_mode', Int8, queue_size=1)
#     3. add at the end of the file:
#           gamepad = Gamepad()
#           gamepad.run()


class Gamepad(Thread):

  def __init__(self):  #add cs
    #self.cs = cs      #decomment

    #delete
    rospy.init_node("CONTROL_STATION", anonymous=False)
    self.Nav_Joystick_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.HD_Angles_pub    = rospy.Publisher('HD_joints', Int8MultiArray, queue_size=1)
    self.HD_voltmeter_pub = rospy.Publisher('HD_voltmeter', Int8, queue_size=1)
    self.HD_mode_pub      = rospy.Publisher('HD_mode', Int8, queue_size=1)

    # ~ connection search gamepad
    for path in evdev.list_devices():
      self.control = evdev.InputDevice(path)
      # device.capabilities() returns all possible (KEY, value) pairs
      # representing the actions linked to device  
      # .EV_FF sends force feedback commands to an input device
      if 1 or evdev.ecodes.EV_FF in self.control.capabilities():
        self._running = True # variable to stop gamepad thread => when set to False
        break
      else: # error in connecting to gamepad
        sys.stderr.write('Failed to find the haptic motor.\n')
        self.control = None

    # start-up mode = NAV
    self.mode = 'NAV' #or 'HD'

    #------------------NAVIGATION------------------
    # declare and initialize msg_nav_dir
    self.msg_nav_dir = Twist()
    self.msg_nav_dir.linear.x  = 0
    self.msg_nav_dir.linear.y  = 0  # always zero
    self.msg_nav_dir.linear.z  = 0  # always zero
    self.msg_nav_dir.angular.x = 0  # always zero
    self.msg_nav_dir.angular.y = 0  # always zero
    self.msg_nav_dir.angular.z = 0
    self.rate = rospy.Rate(10)

    self.axe_NAV_old = [0., 0.]  # [.linear.x, .angular.z]
    self.axe_NAV_new = [0., 0.]

    #---------------HANDLING DEVICE----------------
    self.zero_HD = [0, 0, 0, 0, 0, 0, 0]
    self.axe_HD_old = [0, 0, 0, 0, 0, 0, 0]
    self.axe_HD_new = [0, 0, 0, 0, 0, 0, 0]
    self.joint = 0  # joint 1 as default joint
    self.voltmeter  = 0
   
    self.modeHD = 'DIR' #or 'INV' or 'DEBUG'
    self.modeHDmsg = 2  
      # DEBUG == 1
      # DIR   == 2
      # INV   == 3

    self.joint3 = 1
    self.joint4 = 1



  def run (self):
    advance = 0
    retreat = 0
    for event in self.control.read_loop():
      if event.type != 0:
        if (self._running) == 0: # when self._running == False  run() stops
          break
        # EV_KEY describes state changes of device
        if event.type == ecodes.EV_KEY:
          if event.value == 1:
            print(event.code) #TODO
          #---------------SWITCH NAV <=> HD------------------------------------------------------------------
            if event.code == Keymap.BTN_SHARE.value:  # Share Button => switch NAV <=> HD
              self.switchNAV_HD()

            # switching  DIR => INV => DEBUG => DIR  only when in HD    
            if event.code == Keymap.BTN_OPTIONS.value:  # Option Button
              if self.mode == 'HD':
                self.switchHDmode()



        #------------------NAVIGATION------------------------------------------------------------------------
        if (self.mode) == 'NAV':
          if event.type == ecodes.EV_ABS:
            absevent = categorize(event)
          #---------------ANGULAR.Z--------------------------------------------------------------------------
            if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RX":  # x-axis of R3
              #-----------increase/decrease-------------
              # push joystick to the right for positive value
              self.axe_NAV_new[1] = round(absevent.event.value/32767, 5) # Max value :32768

          #---------------LINEAR.X---------------------------------------------------------------------------
            #-------------advance--------------
            if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RZ":  # button R2
              if (retreat == 0):  # advance (only if not retreating)
                self.axe_NAV_new[0] = round(absevent.event.value/255, 5)
                advance = 1
              if absevent.event.value == 0:  # stay still
                advance = 0
            #-------------retreat--------------
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Z":  # button L2
              if (advance == 0):  # go backwards (only if not retreating)
                self.axe_NAV_new[0] = round(-absevent.event.value/255, 5)
                retreat = 1
              if absevent.event.value == 0:  # stay still
                retreat = 0

            # Publish variation with round 0.1 to NAV only when it was changed
            if (compare_list(self.axe_NAV_old, self.axe_NAV_new, 0) != 1):
              print("send NAV:\nold: ", self.axe_NAV_old , ' new: ', self.axe_NAV_new)  
              # update values of axe_NAV_old, linear.x and angular.z
              self.axe_NAV_old[0] = self.axe_NAV_new[0]
              self.axe_NAV_old[1] = self.axe_NAV_new[1]
              self.msg_nav_dir.linear.x = self.axe_NAV_new[0]
              self.msg_nav_dir.angular.z = self.axe_NAV_new[1]
             
              #     TODO
              print(self.axe_NAV_new)
             
              self.Nav_Joystick_pub.publish(self.msg_nav_dir)
             


        #-----------------HANDLING DEVICE--------------------------------------------------------------------
        elif (self.mode) == 'HD':
          #---------------INVERSE----------------------------------------------------------------------------
          if self.modeHD == 'INV':

            # sends new values if and only if changed from previous
            newAxeVal(self)

          #---------------DEBUG----------------------------------------------------------------------------
          elif self.modeHD == 'DEBUG':
            if event.type == ecodes.EV_KEY:
              if event.value == 1:
                #-----------joints 3, 4, 5, 6, gripper-------------
                if event.code == Keymap.BTN_L1.value:         # joint 3
                  self.joint = 2
                elif event.code == Keymap.BTN_R1.value:       # joint 4
                  self.joint = 3
                elif event.code == Keymap.BTN_TRIANGLE.value: # joint 5
                  self.joint = 4
                elif event.code == Keymap.BTN_CIRCLE.value:   # joint 6
                  self.joint = 5
                elif event.code == Keymap.BTN_CROSS.value:    # joint 7: gripper
                  self.joint = 6
                #----------voltmeter------------
                elif event.code == Keymap.BTN_PS.value:       # PS button
                  self.switchVoltmeter()
             
            if event.type == ecodes.EV_ABS:  
              absevent = categorize(event)
              #-----------joint 1-------------
              if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_HAT0X":  # d-pad x            
                self.joint = 0
              #-----------joint 2-------------  
              if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_HAT0Y":  # d-pad y
                self.joint = 1
              #-------------velocity-------------
              if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RY":  # joint R3 up & down
                #-----------increase/decrease-------------
                # push joystick up for positive velocity      
                self.axe_HD_new[self.joint] = 100*round(-absevent.event.value/32767, 5) # Max value: 32768
               
#             if absevent.event.value == 0:
#               print("hi")
#               self.axe_HD_new[self.joint] = 0
#               resetAxe(self)

            # sends new values if and only if changed from previous
            newAxeVal(self)
            resetAxe(self)

          #---------------DIRECT---------------------------------------------------------------------------
          elif self.modeHD == 'DIR':
            if event.type == ecodes.EV_KEY:
              if event.value == 1:
                #-----------gripper------------
                if event.code == Keymap.BTN_TRIANGLE.value:   # gripper = +100
                  self.axe_HD_new[6] = 100.00
                elif event.code == Keymap.BTN_CIRCLE.value:   # gripper = +1
                  self.axe_HD_new[6] = 2.10
                elif event.code == Keymap.BTN_CROSS.value:    # gripper = -100
                  self.axe_HD_new[6] = -100.00
                elif event.code == Keymap.BTN_SQUARE.value:   # gripper = -1
                  self.axe_HD_new[6] = -2.10
                #----------joint 3, 4 retreat-----------
                elif event.code == Keymap.BTN_R1.value:       # R1 - joint 3 retreat
                  self.joint3 = -1
                elif event.code == Keymap.BTN_R2.value:       # R2 - joint 3 
                  self.axe_HD_new[2] = 50
                elif event.code == Keymap.BTN_L1.value:       # L1 - joint 4 retreat
                  self.joint4 = -1
                elif event.code == Keymap.BTN_L2.value:       # L2 - joint 4 
                  self.axe_HD_new[3] = 50
                #----------voltmeter------------
                elif event.code == Keymap.BTN_PS.value:       # PS button
                  self.switchVoltmeter()
              #-----------joint 3, 4 advance------------
              elif event.value == 0:
                self.axe_HD_new[6] = 0
                if event.code == Keymap.BTN_R1.value:         # R1 - joint 3 retreat
                  self.joint3 = 1
                elif event.code == Keymap.BTN_L1.value:       # L1 - joint 4 retreat
                  self.joint4 = 1
            #---------------velocity---------------  
            if event.type == ecodes.EV_ABS:  
              absevent = categorize(event)    
              #-----------joint 2-------------
              if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RY":  # R3 up & down
                self.axe_HD_new[1] = 100*round(absevent.event.value/32767, 5) # Max value: 32768
              #-----------joint 1-------------
              if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RX":  # R3 left & right
                self.axe_HD_new[0] = 100*round(-absevent.event.value/32767, 5) # Max value: 32768
              #-----------joint 5-------------
              if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":  # L3 up & down
                self.axe_HD_new[4] = 100*round(-absevent.event.value/32767, 5) # Max value: 32768
              #-----------joint 6-------------
              if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":  # L3 left & right
                self.axe_HD_new[5] = 100*round(absevent.event.value/32767, 5) # Max value: 32768
              #-----------joint 3-------------
              if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RZ":  # R2
                self.axe_HD_new[2] = 100*round(self.joint3 * absevent.event.value/255, 5) # Max value: 32768
              #-----------joint 4-------------
              if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Z":  # L2
                self.axe_HD_new[3] = 100*round(self.joint4 * absevent.event.value/255, 5) # Max value: 32768

            # sends new values if and only if changed from previous
            newAxeVal(self)
            resetAxe(self)
       

  # Set self.mode
  def cmode(self, mode):
    self.mode = mode

  # Switching NAV <=> HD
  def switchNAV_HD(self):
    # switching mode  NAV <=> HD
    if self.mode == 'NAV':
      self.cmode('HD')
      self.modeHD = 'DIR'  # reset to DIR
      print(self.mode, ': ', self.modeHD)
    elif self.mode == 'HD':
      self.cmode('NAV')
      print(self.mode)
#     TODO publish!!!!!!  

  # Switching DIR => INV => DEBUG => DIR
  # when in HD mode
  def switchHDmode(self):  
    #------DIR => INV------
    if self.modeHD == 'DIR':
      self.modeHDmsg = 3
      print('INV')
      self.modeHD = 'INV'
    #-----INV => DEBUG-----
    elif self.modeHD == 'INV':
      self.modeHDmsg = 1
      print('DEBUG')
      self.modeHD = 'DEBUG'
    #-----DEBUG => DIR-----
    elif self.modeHD == 'DEBUG':
      self.modeHDmsg = 2
      print('DIR')
      self.modeHD = 'DIR'
     
     #     TODO
    print(self.modeHD)  

    self.axe_HD_old = clear_tab(self.axe_HD_old)
    self.axe_HD_new = clear_tab(self.axe_HD_new)

#     self.cs.HD_mode_pub.publish(Int8(data = self.modeHDmsg))
#     self.cs.HD_Angles_pub.publish(Int8MultiArray(data = self.axe_HD_new))
    self.HD_mode_pub.publish(Int8(data = self.modeHDmsg))
    self.HD_Angles_pub.publish(Int8MultiArray(data = list(map(int,self.axe_HD_new))))

  # Voltmeter:  Extend <=> Retreat
  def switchVoltmeter(self):
    if self.voltmeter == 0:
      self.voltmeter = 1
    else:
      self.voltmeter = 0
      #     TODO
    print(self.voltmeter)  
#     self.cs.HD_voltmeter_pub.publish(Int8(data = self.voltmeter))
    self.HD_voltmeter_pub.publish(Int8(data = self.voltmeter))


def eval_axe(axe_value): #Donne le sens de rotation bras robot 32768 est la valeur max renvoyé par la manette
  if axe_value <= -32700:
    return -1
  elif axe_value >= 32700:
    return 1
  elif axe_value > -32700 and axe_value < 32700:
    return 0


# Compare 2 arrays with a tolerance
def compare_list(list1, list2, tolerance):
  for k in range(len(list1)):
    if (list1[k]+tolerance >= list2[k] and list1[k]-tolerance <= list2[k])!=1:
      return 0
  return 1


# Values small enough to reset to 0
def compare_list_reset(list1):
  for k in range(len(list1)):
    if (list1[k] >= -2 and list1[k] <= 2 and list1[k] != 0):
      return 0
  return 1


# Clear array
def clear_tab(tab):
  for k in range(len(tab)):
    tab[k] = 0
  return tab
 

# Send new Joint velocities for HD
# if and only if changed from previous
def newAxeVal(self):
  if (compare_list(self.axe_HD_old, self.axe_HD_new, 1) != 1):
    print("send HD - angles:\nold:", self.axe_HD_old , ' new:', self.axe_HD_new)
    for k in range(len(self.axe_HD_new)):
      self.axe_HD_old[k] = self.axe_HD_new[k]

#     TODO
    print(self.axe_HD_new)
   
#     self.cs.HD_Angles_pub.publish(Int8MultiArray(data = self.axe_HD_new))
    self.HD_Angles_pub.publish(Int8MultiArray(data = list(map(int,self.axe_HD_new))))
   
   
# Send new Joint velocities for HD
# if and only if changed from previous
def resetAxe(self):
  if (compare_list_reset(self.axe_HD_old) != 1):
    self.axe_HD_new = clear_tab(self.axe_HD_new)
    print("send HD - angles:\nold:", self.axe_HD_old , ' new:', self.axe_HD_new)
    self.axe_HD_old = clear_tab(self.axe_HD_old)
#     TODO
    print(self.axe_HD_new)
#     self.cs.HD_Angles_pub.publish(Int8MultiArray(data = self.axe_HD_new))
    self.HD_Angles_pub.publish(Int8MultiArray(data = list(map(int, self.axe_HD_new))))


gamepad = Gamepad()
gamepad.run()