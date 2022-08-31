'''
@File  :

@Author: Emma Gaia Poggiolini
'''
import evdev
from   evdev           import *

import rospy
import sys

from   threading       import Thread
from   keyMap          import *
from std_msgs.msg      import Int8MultiArray, Int8


'''
Class Gamepad

@Description: Thread to 

@Attributes:
    -

'''

class Gamepad(Thread):

  def __init__(self):  

    rospy.init_node("CONTROL_STATION", anonymous=False)
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

    self.mode = 'HD' 
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
            # switching  DIR => INV => DEBUG => DIR  only when in HD     
            if event.code == Keymap.BTN_OPTIONS.value:  # Option Button
              if self.mode == 'HD':
                self.switchHDmode()

        #-----------------HANDLING DEVICE--------------------------------------------------------------------
        elif (self.mode) == 'HD': 
          #---------------INVERSE----------------------------------------------------------------------------
          if self.modeHD == 'INV':
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
                elif event.code == Keymap.BTN_L1.value:       # L1 - joint 4 retreat
                  self.joint4 = -1
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
                self.axe_HD_new[0] = 100*round(-absevent.event.value/32767, 15) # Max value: 32768
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

    self.axe_HD_old = clear_tab(self.axe_HD_old)
    self.axe_HD_new = clear_tab(self.axe_HD_new)
    self.HD_mode_pub.publish(Int8(data = self.modeHDmsg))
    self.HD_Angles_pub.publish(Int8MultiArray(data = list(map(int, self.axe_HD_new))))

  # Voltmeter:  Extend <=> Retreat
  def switchVoltmeter(self):
    if self.voltmeter == 0:
      self.voltmeter = 1
    else:
      self.voltmeter = 0
      #     TODO 
    print(self.voltmeter)  
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
    self.HD_Angles_pub.publish(Int8MultiArray(data = list(map(int, self.axe_HD_new))))
    
    
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