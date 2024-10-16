import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int8
from geometry_msgs.msg import Twist, TwistStamped
from control_msgs.msg import JointJog
from custom_msg.msg import Task
import array
import math


VERBOSE = False


def str_pad(x, length=5):
    s = str(x)
    if len(s) >= length:
        return s[:length]
    return s + " " * (length-len(s))


def str_pad_list(l, length=5):
    pad_fct = lambda x: str_pad(x, length)
    return "[ " + ", ".join(map(pad_fct, l)) + " ]"


def normalize(l):
    n = math.sqrt(sum(x**2 for x in l))
    if n == 0:
        return [x for x in l]
    return [x/n for x in l]

class FSM(Node):
    IDLE = -1
    MANUAL_INVERSE = 0
    MANUAL_DIRECT = 1
    SEMI_AUTONOMOUS = 2
    AUTONOMOUS = 3
    
    JOINTSPACE = 0
    CARTESIAN = 1

    def __init__(self):
        super().__init__("HD_fsm", allow_undeclared_parameters=True)
        self.create_ros_interfaces()
        self.velocity = 0
        self.command_expiration = .5   # seconds
        self.received_manual_direct_cmd_at = time.time() - self.command_expiration
        self.received_manual_inverse_cmd_at = time.time() - self.command_expiration
        motor_count = 8
        self.manual_direct_command = array.array('d', [0.0]*motor_count)
        #self.semi_autonomous_command_id = None
        self.semi_autonomous_command = None
        self.mode = self.MANUAL_INVERSE#self.MANUAL_DIRECT
        self.submode = self.JOINTSPACE
        self.target_mode = self.MANUAL_INVERSE
        self.mode_transitioning = False
        self.manual_inverse_axis = [0.0, 0.0, 0.0]
        self.manual_inverse_velocity_scaling = 0.0
        self.manual_inverse_twist = Twist()
        self.manual_inverse_joint = array.array('d', [0.0]*6)
        
        # moveit_servo = "moveit_servo"
        # self.declare_parameter(moveit_servo, "")    # TODO: figure out how to pass a dict parameter and not use eval!
        # self.moveit_servo = eval(self.get_parameter(moveit_servo).get_parameter_value().string_value)
        # self.moveit_servo_frame_id = self.moveit_servo["planning_frame"]
        # self.get_logger().info(str(self.moveit_servo_frame_id))
    
    def create_ros_interfaces(self):
        self.manual_direct_cmd_pub = self.create_publisher(Float64MultiArray, "/HD/fsm/joint_vel_cmd", 10)
        #self.manual_inverse_cmd_pub = self.create_publisher(Float64MultiArray, "/HD/fsm/man_inv_axis_cmd", 10)
        self.manual_inverse_twist_pub = self.create_publisher(TwistStamped, "/HD/fsm/man_inv_twist", 10)
        self.manual_inverse_joint_pub = self.create_publisher(JointJog, "/HD/fsm/man_inv_joint", 10)
        self.task_pub = self.create_publisher(Task, "/HD/fsm/task_assignment", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/HD/fsm/mode_change", 10)
        self.create_subscription(Float32MultiArray, "/CS/HD_gamepad", self.manual_cmd_callback, 10)
        # self.create_subscription(Float32MultiArray, "/ROVER/HD_man_inv_axis", self.manual_cmd_callback, 10)
        self.create_subscription(Twist, "/ROVER/HD_man_inv_twist", self.manual_cmd_callback, 10)
        self.create_subscription(Float64MultiArray, "/ROVER/HD_man_inv_joint", self.manual_cmd_callback, 10)
        self.create_subscription(Int8, "/ROVER/HD_mode", self.mode_callback, 10)
        self.create_subscription(Int8, "/ROVER/HD_sub_mode", self.sub_mode_callback, 10)
        self.create_subscription(Task, "/ROVER/semi_auto_task", self.task_cmd_callback, 10)
        self.create_subscription(Int8, "/ROVER/HD_element_id", self.task_cmd_callback2, 10)

    def deprecate_all_commands(self):
        self.received_manual_direct_cmd_at = time.time() - 2*self.command_expiration
        self.received_manual_inverse_cmd_at = time.time() - 2*self.command_expiration
    
    def mode_callback(self, msg: Int8):
        """listens to HD_mode topic published by CS"""
        self.deprecate_all_commands()
        self.target_mode = msg.data
        self.mode_transitioning = True
        
    def sub_mode_callback(self, msg: Int8):
        """listens to HD_sub_mode topic published by CS"""
        self.deprecate_all_commands()
        self.submode = msg.data

    def manual_cmd_callback(self, msg: Float32MultiArray):
        """listens to HD_joints topic"""
        if self.mode == self.MANUAL_DIRECT:
            self.manual_direct_command = msg.data[1:]
            self.manual_direct_velocity_scaling = msg.data[0]
            self.received_manual_direct_cmd_at = time.time()
        elif self.mode == self.MANUAL_INVERSE:  # TODO: standardize this
            if isinstance(msg, Float64MultiArray):
                self.manual_inverse_joint = msg
            else:
                self.manual_inverse_twist = msg
            # self.manual_inverse_axis = normalize(msg.data[1:4])
            # self.manual_inverse_velocity_scaling = msg.data[0]
            self.received_manual_inverse_cmd_at = time.time()

    def task_cmd_callback(self, msg: Task):
        self.semi_autonomous_command = msg

    def task_cmd_callback2(self, msg: Int8):        # TODO
        task_type = 0   # random default
        task_str = ""
        x = msg.data
        if 100 <= x <= 119 or x == 10 or x ==13:
            task_type = Task.BUTTON
        elif x == 20:
            task_type = Task.PLUG_VOLTMETER_ALIGN
        elif x == 21:
            task_type = Task.PLUG_VOLTMETER_APPROACH
        elif x == 11:
            task_type = Task.METAL_BAR_APPROACH
        elif x == 30:
            task_type = Task.ETHERNET_CABLE
        elif x == 41:
            task_type = Task.RASSOR_SAMPLE
        elif 50 <= x <= 55:
            task_type = Task.NAMED_TARGET
            task_str = ["home", "optimal_view", "zero", "face_ground", "science_drop", "back"][x-50]
        elif 70 <= x <= 72:
            task_type = Task.ALIGN_PANEL
        elif x == 80:
            task_type = Task.ROCK_SAMPLING_APPROACH
        elif x == 81:
            task_type = Task.ROCK_SAMPLING_DROP
        elif x == 82:
            task_type = Task.ROCK_SAMPLING_COMPLETE
        self.get_logger().info("AAAAAAAAAAAAAAAAAAA :   " + str(x) + "  ;  " + str(task_type))
        self.semi_autonomous_command = Task(
            type = task_type,
            str_slot = task_str
        )

    def send_task_cmd(self):
        """sends the last task command to the task executor and locks any other command until completion"""

    def send_manual_direct_cmd(self):
        """sends the last direct command to the motor control and locks any other command until completion"""
        if self.manual_direct_command_old(): 
            return
        msg = Float64MultiArray()
        msg.data = array.array('d', self.manual_direct_command)
        if VERBOSE:
            self.get_logger().info("FSM direct cmd :   " + str_pad_list(list(msg.data)))
        self.manual_direct_cmd_pub.publish(msg)

    def send_manual_inverse_cmd(self):
        if self.manual_inverse_command_old():
            return
        msg = Float64MultiArray()
        msg.data = array.array('d', self.manual_inverse_axis + [self.manual_inverse_velocity_scaling])
        if VERBOSE:
            self.get_logger().info("FSM manual inverse cmd :   " + str_pad_list(list(msg.data)))
        self.manual_inverse_cmd_pub.publish(msg)

    def send_manual_inverse_cmd2(self):
        if self.manual_inverse_command_old():
            return
        if self.submode == self.JOINTSPACE:
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"   # aaaaa
            #msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
            msg.joint_names = ["hd_joint1", "hd_joint2", "hd_joint3", "hd_joint4", "hd_joint5", "hd_joint6"]
            msg.velocities = self.manual_inverse_joint.data
            if VERBOSE:
                self.get_logger().info("FSM manual inverse cmd :   " + str_pad_list(list(msg.velocities)))
            self.manual_inverse_joint_pub.publish(msg)
        else:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "hd_link6"   # hd_link6    # Gripper_Finger_v1__1__1
            msg.twist = self.manual_inverse_twist
            if VERBOSE:
                self.get_logger().info("FSM manual inverse cmd :    " + str(msg))
            self.manual_inverse_twist_pub.publish(msg)
        
    def send_semi_autonomous_cmd(self):
        if self.semi_autonomous_command is not None:
            if VERBOSE:
                self.get_logger().info("SENDING SEMI AUTO CMD")
            # msg = Task()
            # msg.description = "btn"
            # msg.id = self.semi_autonomous_command_id
            # self.semi_autonomous_command_id = None
            # self.task_pub.publish(msg)
            self.task_pub.publish(self.semi_autonomous_command)
            self.semi_autonomous_command = None
        
    def updateWorld(self):
        """sends a world update to the trajectory planner"""
        # TODO

    def manual_direct_command_old(self):
        return time.time()-self.received_manual_direct_cmd_at > self.command_expiration
    
    def manual_inverse_command_old(self):
        return time.time()-self.received_manual_inverse_cmd_at > self.command_expiration

    def normal_loop_action(self):
        if VERBOSE:
            self.get_logger().info("MODE : " + str(self.mode))
            self.get_logger().info("SUBMODE : " + str(self.submode))
            
        if self.mode == self.IDLE:
            pass
        elif self.mode == self.AUTONOMOUS:
            pass
        elif self.mode == self.SEMI_AUTONOMOUS:
            self.send_semi_autonomous_cmd()
        elif self.mode == self.MANUAL_INVERSE:
            self.send_manual_inverse_cmd2()
        elif self.mode == self.MANUAL_DIRECT:
            self.send_manual_direct_cmd()

    def transition_loop_action(self):
        if self.mode == self.IDLE:
            pass
        elif self.mode == self.AUTONOMOUS:
            pass
        elif self.mode == self.SEMI_AUTONOMOUS:
            pass
        elif self.mode == self.MANUAL_INVERSE:
            pass
        elif self.mode == self.MANUAL_INVERSE:
            pass

        transition_condition = True
        if transition_condition:
            self.mode_transitioning = False
            self.mode = self.target_mode
            msg = Int8()
            msg.data = self.mode
            self.mode_change_pub.publish(msg)

    def loop(self):
        rate = self.create_rate(25)   # 25hz
        while rclpy.ok():
            if self.mode_transitioning:
                self.transition_loop_action()
            else:
                self.normal_loop_action()
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = FSM()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    try:
        node.loop()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
