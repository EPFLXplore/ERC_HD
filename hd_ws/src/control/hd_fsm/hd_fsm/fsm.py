import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int8
from kerby_interfaces.msg import Task
from interfaces.msg import PanelObject
import array


VERBOSE = True


def str_pad(x, length=5):
    s = str(x)
    if len(s) >= length:
        return s[:length]
    return s + " " * (length-len(s))


def str_pad_list(l, length=5):
    pad_fct = lambda x: str_pad(x, length)
    return "[ " + ", ".join(map(pad_fct, l)) + " ]"


class FSM(Node):
    MANUAL_INVERSE = 0
    MANUAL_DIRECT = 1
    SEMI_AUTONOMOUS = 2
    AUTONOMOUS = 3

    def __init__(self):
        super().__init__("HD_fsm")
        self.create_ros_interfaces()
        self.velocity = 0
        self.command_expiration = .5   # seconds
        self.received_manual_direct_cmd_at = time.time() - self.command_expiration
        self.received_manual_inverse_cmd_at = time.time() - self.command_expiration
        motor_count = 8
        self.manual_direct_command = array.array('d', [0.0]*motor_count)
        self.semi_autonomous_command_id = None
        self.mode = self.MANUAL_DIRECT
        self.target_mode = self.MANUAL_DIRECT
        self.mode_transitioning = False
        self.manual_inverse_axis = [0.0, 0.0, 0.0]
    
    def create_ros_interfaces(self):
        self.manual_direct_cmd_pub = self.create_publisher(Float64MultiArray, "/HD/fsm/joint_vel_cmd", 10)
        self.manual_inverse_cmd_pub = self.create_publisher(Float64MultiArray, "/HD/fsm/man_inv_axis_cmd", 10)
        self.task_pub = self.create_publisher(Task, "/HD/fsm/task_assignment", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/HD/fsm/mode_change", 10)
        self.create_subscription(Float32MultiArray, "/ROVER/HD_gamepad", self.manual_cmd_callback, 10)
        self.create_subscription(Float32MultiArray, "/ROVER/HD_man_inv_axis", self.manual_cmd_callback, 10)
        self.create_subscription(Int8, "/ROVER/HD_mode", self.mode_callback, 10)
        self.create_subscription(Int8, "/ROVER/element_id", self.task_cmd_callback, 10)

    def mode_callback(self, msg: Int8):
        """listens to HD_mode topic published by CS"""
        self.target_mode = msg.data
        self.mode_transitioning = True

    def manual_cmd_callback(self, msg: Float32MultiArray):
        """listens to HD_joints topic"""
        if self.mode == self.MANUAL_DIRECT:
            self.manual_direct_command = msg.data
            self.received_manual_direct_cmd_at = time.time()
        elif self.mode == self.MANUAL_INVERSE:
            self.manual_inverse_axis = msg.data
            self.received_manual_inverse_cmd_at = time.time()

    def task_cmd_callback(self, msg: Int8):
        if self.mode != self.SEMI_AUTONOMOUS: return
        
        self.semi_autonomous_command_id = msg.data

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
        msg.data = array.array('d', self.manual_inverse_axis)
        if VERBOSE:
            self.get_logger().info("FSM manual inverse cmd :   " + str_pad_list(list(msg.data)))
        self.manual_inverse_cmd_pub.publish(msg)

    def send_semi_autonomous_cmd(self):
        if self.semi_autonomous_command_id is not None:
            if VERBOSE:
                self.get_logger().info("SENDING SEMI AUTO CMD")
            msg = Task()
            msg.description = "btn"
            msg.id = self.semi_autonomous_command_id
            #msg.pose = self.semi_autonomous_command_id.pose
            self.semi_autonomous_command_id = None
            self.task_pub.publish(msg)
        
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
            
        if self.mode == self.AUTONOMOUS:
            pass
        elif self.mode == self.SEMI_AUTONOMOUS:
            self.send_semi_autonomous_cmd()
        elif self.mode == self.MANUAL_INVERSE:
            self.send_manual_inverse_cmd()
        elif self.mode == self.MANUAL_DIRECT:
            self.send_manual_direct_cmd()

    def transition_loop_action(self):
        if self.mode == self.AUTONOMOUS:
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
