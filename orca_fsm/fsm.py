import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Point2D

class FSM(Node):

    def __init__(self):
        super().__init__('orca_fsm')

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            'detections_output',
            self.detection_cb,
            10)
        self.detection_sub  # prevent unused variable warning

        self.pub = self.create_publisher(
            Float64MultiArray,
            'fsm_output',
            10)

        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.d_yaw = 0.0

        self.speed_x = 1.0
        self.speed_y = 1.0
        self.speed_z = 1.0
        self.speed_yaw = 1.0

        self.cruise_init_interval = 5.0 # seconds
        self.cruise_interval = 5.0
        self.cruise_direction = 1.0

        self.gate_pose = Point2D()
        self.gate_detected = False
        self.gate_center_tolerance = 30 # pixels
        self.blue_drum_pose = Point2D()
        self.blue_drum_detected = False
        self.metal_ball_pose = Point2D()
        self.metal_ball_detected = False
        self.orange_flare_pose = Point2D()
        self.orange_flare_detected = False
        self.yellow_flare_pose = Point2D()
        self.yellow_flare_detected = False
        self.red_flare_pose = Point2D()
        self.red_flare_detected = False
        self.blue_flare_pose = Point2D()
        self.blue_flare_detected = False

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.cur_task = 'PASS_GATE'
        self.cur_state = 'CRUISE'


    def task_pass_gate(self):
        if self.cur_state == 'CRUISE':
            # TODO: Implement cruise interval logic
            self.dx = self.speed_x * self.cruise_direction
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
            if self.gate_detected:
                self.cur_state = 'AIM_GATE'
        elif self.cur_state == 'AIM_GATE':
            if not self.gate_detected:
                self.cur_state = 'CRUISE'
            elif abs(self.gate_pose.x - 320) < self.gate_center_tolerance:
            # check if gate is centered
                if self.orange_flare_detected:
                    self.cur_state = 'AVOID_FLARE'
                else:
                    self.cur_state = 'AIM_GATE_FORWARD'
            else:
                if self.gate_pose.x < 320:
                    self.dx = self.speed_x
                else:
                    self.dx = self.speed_x * -1.0
                self.dy = 0.0
                self.dz = 0.0
                self.d_yaw = 0.0
        elif self.cur_state == 'AIM_GATE_FORWARD':
            self.dx = 0.0
            self.dy = self.speed_y
            self.dz = 0.0
            self.d_yaw = 0.0
            if not self.gate_detected:
                self.cur_state = 'FINISH'
            elif abs(self.gate_pose.x - 320) > self.gate_center_tolerance:
                self.cur_state = 'AIM_GATE'
        elif self.cur_state == 'AVOID_FLARE':
            # TODO: Implement avoid flare logic
            self.dx = self.speed_x
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
            if True:
                self.cur_state = 'AVOID_FLARE_FORWARD'
        elif self.cur_state == 'AVOID_FLARE_FORWARD':
            # TODO: Implement avoid flare forward logic
            self.dx = 0.0
            self.dy = self.speed_y
            self.dz = 0.0
            self.d_yaw = 0.0
        elif self.cur_state == 'BACT_TO_GATE':
            self.dx = self.speed_x * -1.0
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
            if self.gate_detected:
                self.cur_state = 'AIM_GATE'
        elif self.cur_state == 'FINISH':
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
            self.cur_task = 'BUMP_FLARE'
            self.cur_state = 'DONE'

    def task_bump_flare(self):
        pass
        # TODO: Implement bump flare task

    # def task_drop_ball(self):
    #     # TODO: Implement drop ball task
    #
    # def task_pick_ball(self):
    #     # TODO: Implement pick ball task

    def main_sequence(self):
        if self.cur_state == 'DONE':
            self.get_logger().info('DONE')
        else:
            self.get_logger().info(self.cur_task + ' -> ' + self.cur_state)
        if self.cur_task == 'PASS_GATE':
            self.task_pass_gate()
        elif self.cur_task == 'BUMP_FLARE':
            self.task_bump_flare()
        # elif self.cur_task == 'DROP_BALL':
        #     self.task_drop_ball()
        # elif self.cur_task == 'PICK_BALL':
        #     self.task_pick_ball()
        elif self.cur_task == 'DONE':
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
        else:
            self.get_logger().info('Error: Invalid task')

    def detection_cb(self, detection_msg):
        self.gate_detected = False
        self.blue_drum_detected = False
        self.metal_ball_detected = False
        self.orange_flare_detected = False
        self.yellow_flare_detected = False
        self.red_flare_detected = False
        self.blue_flare_detected = False
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
            # TODO: match object id
            if id == '0':
                if result.hypothesis.score > 0.5:
                    self.gate_pose = result_pose
                    self.gate_detected = True
            elif id == '1':
                if result.hypothesis.score > 0.5:
                    self.blue_drum_pose = result_pose
                    self.blue_drum_detected = True
            elif id == '2':
                if result.hypothesis.score > 0.5:
                    self.metal_ball_pose = result_pose
                    self.metal_ball_detected = True
            elif id == '3':
                if result.hypothesis.score > 0.5:
                    self.orange_flare_pose = result_pose
                    self.orange_flare_detected = True
            elif id == '4':
                if result.hypothesis.score > 0.5:
                    self.yellow_flare_pose = result_pose
                    self.yellow_flare_detected = True
            elif id == '5':
                if result.hypothesis.score > 0.5:
                    self.red_flare_pose = result_pose
                    self.red_flare_detected = True
            elif id == '6':
                if result.hypothesis.score > 0.5:
                    self.blue_flare_pose = result_pose
                    self.blue_flare_detected = True
            else:
                raise ValueError('Invalid class id')

        self.main_sequence()

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [self.dx, self.dy, self.dz, self.d_yaw]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    fsm = FSM()
    # executor = MultiThreadedExecutor()
    # rclpy.spin(fsm, executor)
    rclpy.spin(fsm)
    fsm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
