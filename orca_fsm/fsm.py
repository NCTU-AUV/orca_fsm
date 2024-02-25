import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Point2D

obj_name = {
     '0': 'blue_drum',
     '1': 'blue_flare',
     '2': 'gate',
     '3': 'metal_ball',
     '4': 'orange_flare',
     '5': 'red_flare',
     '6': 'yellow_flare',
}

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

        # Varaibles
        self.cur_task = 'PASS_GATE'
        self.cur_state = 'NONE'
        self.nxt_state = 'START'

        self.dx = 0.0 # front > 0
        self.dy = 0.0 # right > 0
        self.dz = 0.0 # up > 0
        self.d_yaw = 0.0 # cw > 0

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

        # Parameters
        self.cruise_init_interval = 5.0 # seconds
        self.cruise_interval = 1.0
        self.cruise_direction = 1.0
        self.center_tolerance = 30 # pixels
        self.aim_gate_1_tol = 30 # pixels
        self.aim_gate_2_tol = 30 # pixels
        self.forward_1m_time = 4.6 # seconds
        self.pass_gate_finish_time = 15.0 # seconds
        self.flare_avoid_dist = 100 # pixels
        self.pass_flare_time = 35.0 # seconds

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
                    self.nxt_state = 'CRUISE'
                    self.dx = 0.0
            else:
                raise ValueError('Invalid state - ' + self.cur_state)

    def task_bump_flare(self):
        pass
        # TODO: Implement bump flare task

    # def task_drop_ball(self):
    #     # TODO: Implement drop ball task
    #
    # def task_pick_ball(self):
    #     # TODO: Implement pick ball task

    def main_sequence(self):
        if self.cur_task == 'DONE':
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
                if result.hypothesis.score > 0.5:
                    self.pose[obj_name[id]] = result_pose
                    self.detected[obj_name[id]] = True
                    if obj_name[id] == 'gate':
                        self.gate_width = detection.bbox.size_x
            else:
                raise ValueError('Invalid class id')
        self.main_sequence()
        msg = Float64MultiArray()
        msg.data = [self.dx, self.dy, self.dz, self.d_yaw]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    fsm = FSM()
    rclpy.spin(fsm)
    fsm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
