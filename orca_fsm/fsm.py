import time
import cv2
from cv_bridge import CvBridge
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Float64, Int8
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Point2D
from sensor_msgs.msg import Image

obj_name = {
    '0': 'blue_drum',
    '1': 'blue_flare',
    '2': 'gate',
    '3': 'metal_ball',
    '4': 'orange_flare',
    '5': 'red_flare',
    '6': 'yellow_flare',
}

flare_order = {
    0: ['red_flare', 'yellow_flare', 'blue_flare'],
    1: ['yellow_flare', 'blue_flare', 'red_flare'],
    2: ['blue_flare', 'red_flare', 'yellow_flare'],
    3: ['red_flare', 'blue_flare', 'yellow_flare'],
    4: ['yellow_flare', 'red_flare', 'blue_flare'],
    5: ['blue_flare', 'yellow_flare', 'red_flare']
}

class FSM(Node):

    def __init__(self):
        super().__init__('orca_fsm')

        # default mode is real
        self.declare_parameter('mode', 'real')
        self.mode = self.get_parameter('mode').value

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            'detections_output',
            self.detection_cb,
            10)
        self.detection_sub  # prevent unused variable warning

        self.cap = None
        self.cur_frame_msg = Image()

        if self.mode == 'real':
            self.front_cam_sub = self.create_subscription(
                Image,
                '/color/image_raw', # realsense
                self.front_cam_cb,
                10)
            self.front_cam_sub

            self.init_bottom_cam()
            self.cv_bridge = CvBridge()

            self.cam_pub = self.create_publisher(
                Image,
                'image',
                10)
        else:
            self.front_cam_sub = self.create_subscription(
                Image,
                'stereo_camera/left/image',
                self.front_cam_cb,
                10)
            self.front_cam_sub

            self.bottom_cam_sub = self.create_subscription(
                Image,
                '/sauvc_sim/bottom_camera/image_raw',
                self.bottom_cam_cb,
                10)
            self.bottom_cam_sub

            self.cam_pub = self.create_publisher(
                Image,
                'image',
                10)

        self.pub = self.create_publisher(
            Float32MultiArray,
            'fsm_output',
            10)
        
        # Varaibles
        self.cur_task = 'TEST_CAM_SWITCH'
        self.cur_state = 'NONE'
        self.nxt_state = 'START'

        self.dx = 0.0 # front > 0
        self.dy = 0.0 # right > 0
        self.dz = 0.0 # up > 0
        self.d_yaw = 0.0 # cw > 0
        # arm control 0: idle, 1: drop, 2: stretch out, 3: stretch in
        self.arm_state = 0.0

        self.speed_x = 1.0
        self.speed_y = 1.0
        self.speed_z = 1.0
        self.speed_yaw = 1.0

        self.pose = {
            'blue_drum': Point2D(),
            'blue_flare': Point2D(),
            'gate': Point2D(),
            'metal_ball': Point2D(),
            'orange_flare': Point2D(),
            'red_flare': Point2D(),
            'yellow_flare': Point2D()
        }
        self.detected = {
            'blue_drum': False,
            'blue_flare': False,
            'gate': False,
            'metal_ball': False,
            'orange_flare': False,
            'red_flare': False,
            'yellow_flare': False
        }
        self.gate_width = 0.0
        self.prev_time = time.time()
        self.gate_direction = 1.0
        self.avoid_flare_time = 0.0
        self.got_water_com = False
        self.cur_aim_flare_id = 0
        self.flare_down_check_cnt = 0
        self.use_bottom_cam = False

        # Parameters
        self.cruise_init_interval = 5.0 # seconds
        self.cruise_interval = 1.0
        self.cruise_direction = 1.0
        self.center_tolerance = 30 # pixels
        self.aim_gate_1_tol = 30 # pixels
        self.aim_gate_2_tol = 30 # pixels
        self.aim_flare_tol = 30 # pixels
        self.aim_drum_tol = 30 # pixels
        self.aim_drum_tol_bottom = 30 # pixels
        self.aim_metal_ball_tol = 30 # pixels
        self.forward_1m_time = 4.6 # seconds
        self.pass_gate_finish_time = 15.0 # seconds
        self.flare_avoid_dist = 100 # pixels
        self.pass_flare_time = 35.0 # seconds
        self.wait_for_water_com_time = 3.0 # seconds
        self.flare_order_com = 0 # default order
        self.bump_back_timeout = 20.0 # seconds
        self.drop_ball_time = 3.0 # seconds
        self.arm_stretch_out_time = 3.0 # seconds
        self.arm_stretch_in_time = 3.0
        self.leave_area_time = 5.0 # seconds


    def task_pass_gate(self):
        self.cur_state = "NONE"
        while self.cur_state != self.nxt_state:
            self.cur_state = self.nxt_state
            if self.cur_state == 'START':
                self.nxt_state = 'CRUISE'
                self.prev_time = time.time()
            elif self.cur_state == 'CRUISE':
                if time.time() - self.prev_time > self.cruise_interval:
                    self.prev_time = time.time()
                    self.cruise_direction *= -1.0
                    self.cruise_interval *= 2.0
                self.dx = 0.0
                self.dy = self.speed_y * self.cruise_direction
                self.dz = 0.0
                self.d_yaw = 0.0
                if self.detected['gate']:
                    self.nxt_state = 'AIM_GATE_1'
            elif self.cur_state == 'AIM_GATE_1':
                self.dz = 0.0
                self.d_yaw = 0.0
                if not self.detected['gate']:
                    pass # keep previous dx and dy
                elif abs(self.pose['gate'].x - 320.0) < self.aim_gate_1_tol:
                    self.prev_time = time.time()
                    self.nxt_state = 'FORWARD_3M'
                else:
                    self.dx = 0.0
                    if self.pose['gate'].x < 320.0:
                        self.dy = self.speed_y * -1.0
                    else:
                        self.dy = self.speed_y
            elif self.cur_state == "FORWARD_3M":
                self.dx = self.speed_x
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if time.time() - self.prev_time > self.forward_1m_time * 3.0:
                    flare_inside_gate = self.pose['orange_flare'].x < self.pose['gate'].x + self.gate_width / 2 and self.pose['orange_flare'].x > self.pose['gate'].x - self.gate_width / 2
                    if self.detected['orange_flare'] and flare_inside_gate:
                        self.prev_time = time.time()
                        self.nxt_state = 'AVOID_FLARE'
                    else:
                        self.prev_time = time.time()
                        self.nxt_state = 'FORWARD_6M'
            elif self.cur_state == 'AVOID_FLARE':
                self.dx = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if self.pose['orange_flare'].x < self.pose['gate'].x:
                    self.gate_direction = -1.0
                else:
                    self.gate_direction = 1.0
                if (self.pose['orange_flare'].x - 320.0) > 0.0:
                    self.dy = -self.speed_y
                else:
                    self.dy = self.speed_y
                if abs(self.pose['orange_flare'].x - 320.0) > self.flare_avoid_dist:
                    self.avoid_flare_time = time.time() - self.prev_time
                    self.prev_time = time.time()
                    self.nxt_state = 'FORWARD_6M'
            elif self.cur_state == 'FORWARD_6M':
                self.dx = self.speed_x
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if time.time() - self.prev_time > self.forward_1m_time * 5.5:
                    self.prev_time = time.time()
                    self.nxt_state = 'BACK_TO_GATE'
            elif self.cur_state == 'BACK_TO_GATE':
                self.dx = 0.0
                self.dy = self.speed_y * self.gate_direction
                self.dz = 0.0
                self.d_yaw = 0.0
                if self.detected['gate']:
                    self.nxt_state = 'AIM_GATE_2'
                elif time.time() - self.prev_time > self.avoid_flare_time:
                    self.prev_time = time.time()
                    self.nxt_state = 'FORWARD_5M'
            elif self.cur_state == 'AIM_GATE_2':
                self.dz = 0.0
                self.d_yaw = 0.0
                if not self.detected['gate']:
                    pass
                elif abs(self.pose['gate'].x - 320.0) < self.aim_gate_2_tol:
                    self.prev_time = time.time()
                    self.nxt_state = 'FORWARD_5M'
            elif self.cur_state == 'FORWARD_5M':
                self.dx = self.speed_x
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if time.time() - self.prev_time > self.forward_1m_time * 5.0:
                    self.nxt_state = 'FINISH'
            elif self.cur_state == 'FINISH':
                self.dx = self.speed_x
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if time.time() - self.prev_time > self.pass_gate_finish_time:
                    self.cur_task = 'DONE'
                    self.dx = 0.0
            else:
                raise ValueError('Invalid state - ' + self.cur_state)
            if self.cur_task != 'PASS_GATE':
                self.nxt_state = 'START'

    def task_bump_flare(self):
        self.cur_state = "NONE"
        cur_aim_flare = flare_order[self.flare_order_com][self.cur_aim_flare_id]
        while self.cur_state != self.nxt_state:
            self.cur_state = self.nxt_state
            if self.cur_state == 'START':
                # self.nxt_state = 'WAIT' # Real
                self.nxt_state = 'AIM' # For testing
                self.prev_time = time.time()
            elif self.cur_state == 'FIND_FLARE':
                # TODO: Implement find flare 
                self.dx = 0.0
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
            elif self.cur_state == 'WAIT':
                if self.got_water_com or time.time() - self.prev_time > 10.0:
                    self.nxt_state = 'AIM'
                else:
                    self.dx = 0.0
                    self.dy = 0.0
                    self.dz = 0.0
                    self.d_yaw = 0.0
            elif self.cur_state == 'AIM':
                self.dx = 0.0
                if self.detected[cur_aim_flare]:
                    if abs(self.pose[cur_aim_flare].x - 320.0) < self.aim_flare_tol:
                        self.prev_time = time.time()
                        self.nxt_state = 'BUMP_FLARE_FORWARD'
                    elif self.pose[cur_aim_flare].x < 320.0:
                        self.dy = -self.speed_y * 0.7
                    else:
                        self.dy = self.speed_y * 0.7
            elif self.cur_state == 'BUMP_FLARE_FORWARD':
                self.dx = self.speed_x * 1.5
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if not self.detected[cur_aim_flare]:
                    self.flare_down_check_cnt = 0
                    self.nxt_state = 'CHECK_FLARE_DOWN'
                elif abs(self.pose[cur_aim_flare].x - 320.0) > self.aim_flare_tol:
                    self.nxt_state = 'AIM'
            elif self.cur_state == 'CHECK_FLARE_DOWN':
                if self.detected[cur_aim_flare]:
                    self.nxt_state = 'BUMP_FLARE_FORWARD'
                else:
                    self.flare_down_check_cnt += 1
                    if self.flare_down_check_cnt > 6:
                        if self.cur_aim_flare_id == 2:
                            self.cur_task = 'DONE'
                        else:
                            self.prev_time = time.time()
                            self.cur_aim_flare_id += 1
                            self.nxt_state = 'BUMP_FLARE_BACK'
            elif self.cur_state == 'BUMP_FLARE_BACK':
                self.dx = -self.speed_x * 1.5
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if self.detected[cur_aim_flare] or time.time() - self.prev_time > self.bump_back_timeout:
                    self.nxt_state = 'AIM'
            else:
                raise ValueError('Invalid state - ' + self.cur_state)
                
        

    def task_drop_ball(self):
        self.cur_state = 'NONE'
        while self.cur_state != self.nxt_state:
            self.cur_state = self.nxt_state
            if self.cur_state == 'START':
                self.nxt_state = 'CRUISE'
                self.prev_time = time.time()
                self.cruise_direction = 1.0
                self.cruise_interval = self.cruise_init_interval
            elif self.cur_state == 'CRUISE':
                if time.time() - self.prev_time > self.cruise_interval:
                    self.prev_time = time.time()
                    self.cruise_direction *= -1.0
                    self.cruise_interval *= 2.0
                self.dx = 0.0
                self.dy = self.speed_y * self.cruise_direction
                self.dz = 0.0
                self.d_yaw = 0.0
                if self.detected['blue_drum']:
                    self.nxt_state = 'AIM_DRUM_FRONT'
            elif self.cur_state == 'AIM_DRUM_FRONT':
                self.dz = 0.0
                self.d_yaw = 0.0
                if not self.detected['blue_drum']:
                    pass # keep previous dx and dy
                elif abs(self.pose['blue_drum'].x - 320.0) < self.aim_drum_tol:
                    self.prev_time = time.time()
                    self.cur_state = 'FORWARD'
                    self.nxt_state = 'FORWARD'
                else:
                    self.dx = 0.0
                    if self.pose['blue_drum'].x < 320.0:
                        self.dy = self.speed_y * -1.0
                    else:
                        self.dy = self.speed_y
            elif self.cur_state == "FORWARD":
                self.dx = self.speed_x
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if not self.detected['blue_drum']:
                    self.cur_state = 'BOTTOM_AIM' # to escape from loop
                    self.nxt_state = 'BOTTOM_AIM'
                    self.use_bottom_cam = True
                elif abs(self.pose['blue_drum'].y - 320.0) > self.aim_drum_tol:
                    self.nxt_state = 'AIM_DRUM_FRONT'
            elif self.cur_state == 'BOTTOM_AIM':
                self.use_bottom_cam = True
                if not self.detected['blue_drum']:
                    pass # keep previous dx and dy
                elif abs(self.pose['blue_drum'].y - 320.0) < self.aim_drum_tol_bottom:
                    self.prev_time = time.time()
                    self.nxt_state = 'DROP'
                    self.use_bottom_cam = False
                else:
                    self.dy = 0.0
                    if self.pose['blue_drum'].y < 320.0:
                        self.dx = self.speed_x * -1.0
                    else:
                        self.dx = self.speed_x
            elif self.cur_state == 'DROP':
                self.dx = 0.0
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                self.arm_state = 1.0
                if time.time() - self.prev_time > self.drop_ball_time:
                    self.cur_task = 'PICK_BALL'
                    self.cur_state = 'START'
                    self.nxt_state = 'START'
            else:
                raise ValueError('Invalid state - ' + self.cur_state)


    def task_pick_ball(self):
        self.cur_state = 'NONE'
        while self.cur_state != self.nxt_state:
            self.cur_state = self.nxt_state
            if self.cur_state == 'START':
                self.nxt_state = 'LEAVE_AREA'
                self.prev_time = time.time()
            elif self.cur_state == 'LEAVE_AREA':
                self.dx = self.speed_x * -1.0
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if time.time() - self.prev_time > self.leave_area_time:
                    self.cruise_direction = 10.0
                    self.cruise_interval = self.cruise_init_interval
                    self.prev_time = time.time()
                    self.nxt_state = 'CRUISE'
            elif self.cur_state == 'CRUISE':
                if time.time() - self.prev_time > self.cruise_interval:
                    self.prev_time = time.time()
                    self.cruise_direction *= -1.0
                    self.cruise_interval *= 2.0
                self.dx = 0.0
                self.dy = self.speed_y * self.cruise_direction
                self.dz = 0.0
                self.d_yaw = 0.0
                if self.detected['blue_drum']:
                    self.nxt_state = 'AIM_DRUM_FRONT'
            elif self.cur_state == 'AIM_DRUM_FRONT':
                self.dz = 0.0
                self.d_yaw = 0.0
                if not self.detected['blue_drum']:
                    pass # keep previous dx and dy
                elif abs(self.pose['blue_drum'].x - 320.0) < self.aim_drum_tol:
                    self.prev_time = time.time()
                    self.nxt_state = 'FORWARD'
                else:
                    self.dx = 0.0
                    if self.pose['blue_drum'].x < 320.0:
                        self.dy = self.speed_y * -1.0
                    else:
                        self.dy = self.speed_y
            elif self.cur_state == "FORWARD":
                self.dx = self.speed_x
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                if not detected['blue_drum']:
                    self.cur_state = 'BOTTOM_AIM_DRUM' # to escape from loop
                    self.nxt_state = 'BOTTOM_AIM_DRUM'
                    self.use_bottom_cam = True
                elif abs(self.pose['blue_drum'].y - 320.0) > self.aim_drum_tol:
                    self.nxt_state = 'AIM_DRUM_FRONT'
            elif self.cur_state == 'BOTTOM_AIM_DRUM':
                self.use_bottom_cam = True
                if not self.detected['blue_drum']:
                    pass # keep previous dx and dy
                elif abs(self.pose['blue_drum'].y - 320.0) < self.aim_drum_tol_bottom:
                    self.prev_time = time.time()
                    self.nxt_state = 'BOTTOM_AIM_BALL'
                    self.use_bottom_cam = False
                else:
                    self.dy = 0.0
                    if self.pose['blue_drum'].y < 320.0:
                        self.dx = self.speed_x * -1.0
                    else:
                        self.dx = self.speed_x
            elif self.cur_state == 'BOTTOM_AIM_BALL':
                self.use_bottom_cam = True
                if not self.detected['metal_ball']:
                    pass # keep previous dx and dy
                elif abs(self.pose['metal_ball'].y - 320.0) < self.aim_metal_ball_tol and abs(self.pose['metal_ball'].x - 320.0) < self.aim_metal_ball_tol:
                    self.prev_time = time.time()
                    self.nxt_state = 'STRETCH_OUT'
                    self.use_bottom_cam = False
                else:
                    self.dy = 0.0
                    if self.pose['blue_drum'].y < 320.0:
                        self.dx = self.speed_x * -1.0
                    else:
                        self.dx = self.speed_x
            elif self.cur_state == 'STRETCH_OUT':
                self.dx = 0.0
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
                self.arm_state = 2.0
                if time.time() - self.prev_time > self.arm_stretch_out_time:
                    self.prev_time = time.time()
                    self.nxt_state = 'STRETCH_IN'
            elif self.cur_state == 'STRETCH_IN':
                self.arm_state = 3.0
                if time.time() - self.prev_time > self.arm_stretch_in_time:
                    self.cur_task = 'CHECK_BALL'
            elif self.cur_state == 'CHECK_BALL':
                if not self.detected['metal_ball']:
                    self.cur_task = 'DONE'
                else: # keep trying to pick the ball
                    self.nxt_state = 'BOTTOM_AIM_BALL'
            else:
                raise ValueError('Invalid state - ' + self.cur_state)

    def task_test_cam_switch(self):
        self.cur_state = 'NONE'
        while self.cur_state != self.nxt_state:
            self.cur_state = self.nxt_state
            if self.cur_state == 'START':
                self.prev_time = time.time()
                self.nxt_state = 'FRONT_AIM'
            elif self.cur_state == 'FRONT_AIM':
                if time.time() - self.prev_time > 5.0:
                    self.prev_time = time.time()
                    self.use_bottom_cam = True
                    self.nxt_state = 'BOTTOM_AIM'
            elif self.cur_state == 'BOTTOM_AIM':
                if not self.detected['blue_drum']:
                    pass

    def main_sequence(self):
        if self.cur_task == 'DONE':
            self.get_logger().info('DONE')
        else:
            self.get_logger().info(self.cur_task + ' -> ' + self.cur_state)
        self.show_detection()
        if self.cur_task == 'PASS_GATE':
            self.task_pass_gate()
        elif self.cur_task == 'BUMP_FLARE':
            self.task_bump_flare()
        elif self.cur_task == 'DROP_BALL':
            self.task_drop_ball()
        elif self.cur_task == 'PICK_BALL':
            self.task_pick_ball()
        elif self.cur_task == 'TEST_CAM_SWITCH':
            self.task_test_cam_switch()
        elif self.cur_task == 'DONE':
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
        else:
            self.get_logger().info('Error: Invalid task')

    def detection_cb(self, detection_msg):
        for key in self.detected.keys():
            self.detected[key] = False
        score_dict = {}
        for detection in detection_msg.detections:
            result = detection.results[0]
            # get the detection with the highest score
            # self.get_logger().info('Class ID: ' + result.hypothesis.class_id)
            id = detection.results[0].hypothesis.class_id
            score = detection.results[0].hypothesis.score
            result_pose = detection.bbox.center.position
            if id in score_dict:
                if result.hypothesis.score < score_dict[id]:
                    continue
                score_dict[id] = result.hypothesis.score
            if id in obj_name:
                hyp_threshold = 0.5
                if id == '1' or id == '5' or id == '6': # flares
                    hyp_threshold = 0.2
                if result.hypothesis.score > hyp_threshold:
                    self.pose[obj_name[id]] = result_pose
                    self.detected[obj_name[id]] = True
                    if obj_name[id] == 'gate':
                        self.gate_width = detection.bbox.size_x
            else:
                raise ValueError('Invalid class id')
        self.main_sequence()
        msg = Float32MultiArray()
        msg.data = [self.dx, self.dy, self.dz, self.d_yaw, self.arm_state]
        self.pub.publish(msg)

    def show_detection(self):
        for key in self.detected.keys():
            if self.detected[key]:
                self.get_logger().info(key + ' detected at (' + str(self.pose[key].x) + ', ' + str(self.pose[key].y) + ')')
            else:
                self.get_logger().info(key + ' not detected')

    def front_cam_cb(self, img_msg):
        if not self.use_bottom_cam:
            self.cur_frame_msg = img_msg

    def bottom_cam_cb(self, img_msg):
        if self.use_bottom_cam:
            self.cur_frame_msg = img_msg

    def init_bottom_cam(self):
        self.cap = cv2.VideoCapture(6)
        if not self.cap.isOpened():
            self.get_logger().info('Error: Cannot open bottom camera')
            return
        self.get_logger().info('Bottom camera opened')

    def pub_image(self):
        if self.mode == 'real':
            if self.use_bottom_cam:
                ret, frame = self.cap.read()
                if ret: 
                    frame_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    self.cam_pub.publish(frame_msg)
            else:
                self.cam_pub.publish(self.cur_frame_msg)
        else:
            self.cam_pub.publish(self.cur_frame_msg)


def main(args=None):
    rclpy.init(args=args)
    fsm = FSM()
    executor = MultiThreadedExecutor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(fsm, executor))
    spin_thread.start()

    rate = fsm.create_rate(5)
    while rclpy.ok():
        fsm.pub_image()
        rate.sleep()

    fsm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
