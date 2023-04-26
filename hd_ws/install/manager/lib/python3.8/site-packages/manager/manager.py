import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, Int8, Bool, Int16
from kerby_interfaces.msg import Task


VERBOSE = True


class Manager(Node):
    AUTONOMOUS = 0
    SEMI_AUTONOMOUS = 1
    MANUAL_INVERSE = 2
    MANUAL_DIRECT = 3

    def __init__(self):
        super().__init__("HD_control_manager")
        self.create_ros_interfaces()
        self.velocity = 0
        self.command_expiration = .5   # seconds
        self.received_manual_direct_cmd_at = time.time() - self.command_expiration
        self.received_manual_inverse_cmd_at = time.time() - self.command_expiration
        motor_count = 8
        self.manual_direct_command = [0.0]*motor_count
        self.manual_inverse_command = [0.0]*motor_count
        self.semi_autonomous_command = None
        self.mode = self.MANUAL_DIRECT
        self.target_mode = self.MANUAL_DIRECT
        self.mode_transitioning = False
        self.reset_arm_pos = False
    
    def create_ros_interfaces(self):
        self.manual_direct_cmd_pub = self.create_publisher(Float32MultiArray, '/arm_control/manual_direct_cmd', 10)
        self.pos_manual_inverse_cmd_pub = self.create_publisher(Float32MultiArray, '/arm_control/pos_manual_inverse_cmd', 10)
        self.orient_manual_inverse_cmd_pub = self.create_publisher(Float32MultiArray, '/arm_control/orient_manual_inverse_cmd', 10)
        self.task_pub = self.create_publisher(Task, '/arm_control/task_assignment', 10)
        self.create_subscription(Int8MultiArray, "HD_Angles", self.manual_cmd_callback, 10)
        self.create_subscription(Int8, "CS_HD_mode", self.mode_callback, 10)
        self.create_subscription(Int16, "fsm_state", self.semi_autonomous_callback, 10)

    def mode_callback(self, msg):# Int8):
        """listens to HD_mode topic published by CS"""
        self.target_mode = msg.data
        self.mode_transitioning = True

    def taskCmdCallback(self, msg):# Task):
        """listens to task assignement topic published by detection"""

    def manual_cmd_callback(self, msg):# Int8MultiArray):
        """listens to HD_joints topic"""
        if self.mode == self.MANUAL_DIRECT:
            max_speed = 100
            self.manual_direct_command = [float(x)/max_speed for x in msg.data]
            self.received_manual_direct_cmd_at = time.time()
        elif self.mode == self.MANUAL_INVERSE:
            max_speed = 100
            self.manual_inverse_command = [float(x)/max_speed for x in msg.data]
            self.received_manual_inverse_cmd_at = time.time()

    def semi_autonomous_callback(self, msg):
        self.semi_autonomous_command = msg.data

    def send_task_cmd(self):
        """sends the last task command to the task executor and locks any other command until completion"""

    def send_manual_direct_cmd(self):
        """sends the last direct command to the motor control and locks any other command until completion"""
        if self.manual_direct_command_old(): 
            return
        msg = Float32MultiArray()
        msg.data = self.manual_direct_command
        if VERBOSE:
            self.get_logger().info("manager direct cmd :   " + str(msg.data))
        self.manual_direct_cmd_pub.publish(msg)

    def send_manual_inverse_cmd(self):
        """sends the last manual command to the manual control and locks any other command until completion"""
        if self.manual_inverse_command_old():
            return
        cmd = self.manual_inverse_command
        if cmd[0] != 0 or cmd[1] != 0 or cmd[2] != 0:
            msg = Float32MultiArray()
            msg.data = cmd[:3]
            msg.data.append(max(cmd[:3]))
            if VERBOSE:
                self.get_logger().info("manager pos man inv cmd :   " + str(msg.data))
            self.pos_manual_inverse_cmd_pub.publish(msg)
        elif cmd[3] != 0 or cmd[4] != 0 or cmd[5] != 0:
            msg = Float32MultiArray()
            msg.data = cmd[3:6]
            msg.data.append(max(cmd[3:6]))
            if VERBOSE:
                self.get_logger().info("manager orient man inv cmd :   " + str(msg.data))
            self.orient_manual_inverse_cmd_pub.publish(msg)

    def send_semi_autonomous_cmd(self):
        if self.semi_autonomous_command is not None:
            self.get_logger().info("SENDING SEMI AUTO CMD")
            task = Task()
            task.description = "btn"
            task.id = self.semi_autonomous_command
            self.semi_autonomous_command = None
            self.task_pub.publish(task)

    def updateWorld(self):
        """sends a world update to the trajectory planner"""
        # TODO

    def manual_direct_command_old(self):
        return time.time()-self.received_manual_direct_cmd_at > self.command_expiration
    
    def manual_inverse_command_old(self):
        return time.time()-self.received_manual_inverse_cmd_at > self.command_expiration

    def normal_loop_action(self):
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

    def run(self):
        """main"""
        spin = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin.start()
        rate = self.create_rate(2)   # 25hz
        print("ZEBI")

        """if VERBOSE:
            rospy.logwarn("manager started")"""
        try:
            while rclpy.ok():
                if self.mode_transitioning:
                    self.transition_loop_action()
                else:
                    self.normal_loop_action()
                rate.sleep()
        except KeyboardInterrupt:
            pass
        
        self.destroy_node()
        rclpy.shutdown()
        spin.join()


def main(args=None):
    rclpy.init(args=args)
    Manager().run()


if __name__ == '__main__':
    main()

