import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.client import Client
import rclpy.parameter
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int8
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from control_msgs.msg import JointJog
from custom_msg.msg import Task, HDGoal
from custom_msg.srv import HDMode, RequestHDGoal
import array
import math
from std_srvs.srv import Trigger
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

VERBOSE = True


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


class DoneFlag:
    def __init__(self):
        self.done = False
        self.future: Future = None
    
    def trigger(self, future: Future):
        self.done = True
        self.future = future
    
    def __bool__(self) -> bool:
        return self.done


class FSM(Node):
    IDLE = -1
    MANUAL_INVERSE = 0
    MANUAL_DIRECT = 1
    SEMI_AUTONOMOUS = 2
    COMPLIANT_MOTION = 3

    def __init__(self):
        super().__init__("HD_fsm")
        self.mutually_exclusive_callback_group = MutuallyExclusiveCallbackGroup()
        self.retrant_callback_group = ReentrantCallbackGroup()
        self.create_ros_interfaces()
        self.velocity = 0
        self.command_expiration = .5   # seconds
        self.received_manual_direct_cmd_at = time.time() - self.command_expiration
        self.received_manual_inverse_cmd_at = time.time() - self.command_expiration
        motor_count = 8
        self.manual_direct_command = array.array('d', [0.0]*motor_count)
        #self.semi_autonomous_command_id = None
        self.semi_autonomous_command = None
        self.mode = self.IDLE
        self.target_mode = self.mode
        self.mode_transitioning = False
        #self.manual_inverse_axis = [0.0, 0.0, 0.0]
        #self.manual_inverse_velocity_scaling = 0.0
        self.manual_inverse_twist = Twist()
    
    def get_str_param(self, name: str, default: str = "") -> str:
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value().string_value
        
    def create_ros_interfaces(self):
        # publishers
        self.manual_direct_cmd_pub = self.create_publisher(Float64MultiArray, self.get_str_param("hd_fsm_joint_vel_cmd_topic"), 10)
        self.manual_inverse_twist_pub = self.create_publisher(TwistStamped, self.get_str_param("hd_fsm_man_inv_twist_topic"), 10)
        self.task_pub = self.create_publisher(Task, self.get_str_param("hd_fsm_task_assignment_topic"), 10)
        self.mode_change_pub = self.create_publisher(Int8, self.get_str_param("hd_fsm_mode_transmission_topic"), 10)
        self.abort_pub = self.create_publisher(Int8, self.get_str_param("hd_fsm_abort_topic"), 10)
        # old
        self.manual_inverse_cmd_pub = self.create_publisher(Float64MultiArray, "/HD/fsm/man_inv_axis_cmd", 10)
        
        # servers
        self.hd_mode_srv = self.create_service(HDMode, self.get_str_param("hd_fsm_mode_srv"), self.new_mode_callback)
        self.goal_assignment_srv = self.create_service(RequestHDGoal, self.get_str_param("hd_fsm_goal_srv"), self.goal_assignement_callback, callback_group=self.retrant_callback_group)

        # subscriptions
        self.create_subscription(Float32MultiArray, self.get_str_param("rover_hd_man_dir_topic"), self.manual_direct_cmd_callback, 10)
        self.create_subscription(Float32MultiArray, self.get_str_param("rover_hd_man_inv_topic"), self.manual_inverse_cmd_callback, 10)
        # self.create_subscription(Int8, self.get_str_param("rover_hd_man_inv_topic"), self.mode_callback, 10)
        self.create_subscription(Task, "/ROVER/semi_auto_task", self.task_cmd_callback, 10)
        self.create_subscription(Int8, "/ROVER/HD_element_id", self.task_cmd_callback2, 10)
        
        # clients
        self.servo_start_cli = self.create_client(Trigger, "/servo_node/start_servo")
        self.task_executor_goal_assignment_cli = self.create_client(RequestHDGoal, self.get_str_param("hd_task_executor_goal_srv"))
        self.perception_goal_assignment_cli = self.create_client(RequestHDGoal, self.get_str_param("hd_model_set_goal_srv"))

    def deprecate_all_commands(self):
        self.received_manual_direct_cmd_at = time.time() - 2*self.command_expiration
        self.received_manual_inverse_cmd_at = time.time() - 2*self.command_expiration
    
    def new_mode_callback(self, request: HDMode.Request, response: HDMode.Response) -> HDMode.Response:
        temp_mode_map = {
            HDMode.Request.OFF: FSM.IDLE,
            HDMode.Request.MANUAL_DIRECT: FSM.MANUAL_DIRECT,
            HDMode.Request.MANUAL_INVERSE: FSM.MANUAL_INVERSE,
            HDMode.Request.AUTO: FSM.SEMI_AUTONOMOUS,
            HDMode.Request.COMPLIANT_MOTION: FSM.COMPLIANT_MOTION,
        }
        self.target_mode = temp_mode_map[request.mode]
        self.mode_transitioning = True
        # while self.mode_transitioning:
        #     pass
        # TODO:
        response.system_mode = request.mode
        return response
    
    def mode_callback(self, msg: Int8):
        """listens to HD_mode topic published by CS"""
        self.deprecate_all_commands()
        self.target_mode = msg.data
        self.mode_transitioning = True

    def manual_direct_cmd_callback(self, msg: Float32MultiArray):
        # TODO: add mode check in all callbacks
        if self.mode != self.MANUAL_DIRECT:
            return
        scalings = [-1, 1, -1, -1, 0.13/0.2, -1]
        self.manual_direct_command = msg.data
        for i in range(min(len(self.manual_direct_command), len(scalings))):
            self.manual_direct_command[i] *= scalings[i]
        self.received_manual_direct_cmd_at = time.time()
    
    def manual_inverse_cmd_callback(self, msg: Float32MultiArray):
        x = 1; y = 2; z = 3
        axis_mapping = {    # from urdf to convention (with Ugo) (negative index means that direction should be reversed)
            x: -z,
            y: y,
            z: x
        }
        ang_mapping = {
            x: x,
            y: y,
            z: z
        }
        sign = lambda x: 1 if x >= 0 else -1
        get_lin = lambda coord: sign(axis_mapping[coord]) * msg.data[abs(axis_mapping[coord])-1]
        get_ang = lambda coord: sign(ang_mapping[coord]) * msg.data[3 + abs(ang_mapping[coord])-1]
        self.manual_inverse_twist = Twist(
            linear=Vector3(
                x=get_lin(x),
                y=get_lin(y),
                z=get_lin(z)
            ),
            angular=Vector3(
                x=get_ang(x),
                y=get_ang(y),
                z=get_ang(z)
            )
        )
        # TODO: deal with msg.data[6] containing gripper command
        self.received_manual_inverse_cmd_at = time.time()
        
    def manual_cmd_callback(self, msg: Float32MultiArray):
        """old"""
        if self.mode == self.MANUAL_DIRECT:
            self.manual_direct_command = msg.data[1:]
            self.manual_direct_velocity_scaling = msg.data[0]
            self.received_manual_direct_cmd_at = time.time()
        elif self.mode == self.MANUAL_INVERSE:  # TODO: standardize this
            self.manual_inverse_twist = msg
            # self.manual_inverse_axis = normalize(msg.data[1:4])
            # self.manual_inverse_velocity_scaling = msg.data[0]
            self.received_manual_inverse_cmd_at = time.time()

    def goal_assignement_callback(self, request: RequestHDGoal.Request, response: RequestHDGoal.Response) -> RequestHDGoal.Response:
        goal = request.goal
        
        if goal.target == HDGoal.ABORT:
            threading.Thread(target=self.abort).start()
            response.success = True
            response.message = HDGoal.OK
            return response
        
        query_perception = False
        
        if query_perception:
            perception_response = self.send_request(self.perception_goal_assignment_cli, goal)
            if not perception_response.success:
                response.success = False
                response.message = perception_response.message
                return response
        
        task_exec_response = self.send_request(self.task_executor_goal_assignment_cli, goal)
        if not task_exec_response.success:
            response.success = False
            response.message = task_exec_response.message
            return response
        
        response.success = True
        response.message = HDGoal.OK
        return response
    
    def send_request(self, client: Client, goal: HDGoal) -> RequestHDGoal.Response:
        req = RequestHDGoal.Request()
        req.goal = goal
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        future = client.call_async(req)
        done_flag = DoneFlag()
        future.add_done_callback(done_flag.trigger)
        
        # rate = self.create_rate(100)
        while not done_flag:
            self.get_logger().warn("call not done")
            # rate.sleep()
            time.sleep(0.05)
        
        return done_flag.future.result()

    def abort(self):
        for _ in range(5):
            self.abort_pub.publish(Int8())
            time.sleep(0.1)
    
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
        self.get_logger().info("AAAAAAAAAAAAAAAAAAA :   " + str(x) + "  ;  " + str(task_type), throttle_duration_sec=1)
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
            self.get_logger().info("FSM direct cmd :   " + str_pad_list(list(msg.data)), throttle_duration_sec=1)
        self.manual_direct_cmd_pub.publish(msg)

    def send_manual_inverse_cmd_old(self):
        if self.manual_inverse_command_old():
            return
        #msg = Float64MultiArray()
        lin = self.manual_inverse_twist.linear
        ang = self.manual_inverse_twist.angular
        data = [lin.x, lin.y, lin.z, 1.0]     # 1 for velocity scaling
        #msg.data = array.array('d', data)
        msg = Float64MultiArray(data=data)
        if VERBOSE:
            self.get_logger().info("FSM manual inverse cmd :   " + str_pad_list(list(msg.data)), throttle_duration_sec=1)
        self.manual_inverse_cmd_pub.publish(msg)
        
    def send_manual_inverse_cmd(self):
        if self.manual_inverse_command_old():
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "base_link"     #hd_link6   #Gripper_Finger_v1__1__1
        msg.twist = self.manual_inverse_twist
        if VERBOSE:
            self.get_logger().info("FSM manual inverse cmd :    " + str(msg), throttle_duration_sec=1)
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

    def send_trigger_request(self, client: Client):
        return
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        req = Trigger.Request()
        future = client.call_async(req)
        # TODO: wait for result (callback or other)
        return
        # rclpy.spin_until_future_complete(self, future)
        # return future.result()

    def normal_loop_action(self):
        if VERBOSE:
            self.get_logger().info("MODE : " + str(self.mode), throttle_duration_sec=1)
        
        if self.mode == self.IDLE:
            pass
        elif self.mode == self.SEMI_AUTONOMOUS:
            #self.send_semi_autonomous_cmd()
            pass
        elif self.mode == self.MANUAL_INVERSE:
            self.send_manual_inverse_cmd_old()
        elif self.mode == self.MANUAL_DIRECT:
            self.send_manual_direct_cmd()

    def transition_loop_action(self):
        if self.target_mode == self.MANUAL_INVERSE:
            self.send_trigger_request(self.servo_start_cli)
            
        if self.mode == self.IDLE:
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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    # thread.start()
    thread = threading.Thread(target=spin, args=(executor, ), daemon=True)
    thread.start()

    try:
        node.loop()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


def spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    
    
if __name__ == '__main__':
    main()