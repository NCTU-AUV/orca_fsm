import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Pose2D

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

        self.gate_pose = Pose2D()
        self.gate_detected = False
        self.blue_drum_pose = Pose2D()
        self.blue_drum_detected = False
        self.metal_ball_pose = Pose2D()
        self.metal_ball_detected = False
        self.orange_flare_pose = Pose2D()
        self.orange_flare_detected = False
        self.yellow_flare_pose = Pose2D()
        self.yellow_flare_detected = False
        self.red_flare_pose = Pose2D()
        self.red_flare_detected = False
        self.blue_flare_pose = Pose2D()
        self.blue_flare_detected = False

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.cur_task = 'PASS_DOOR'


    def task_pass_door(self):
        self.get_logger().info('PASS DOOR')
        # TODO: Implement pass door task
        return True

    def task_bump_flare(self):
        self.get_logger().info('BUMP FLARE')
        # TODO: Implement bump flare task
        return True

    def task_drop_ball(self):
        self.get_logger().info('DROP BALL')
        # TODO: Implement drop ball task
        return True

    def task_pick_ball(self):
        self.get_logger().info('PICK BALL')
        # TODO: Implement pick ball task
        return True

    def main_sequence(self):
        if self.cur_task == 'PASS_DOOR':
            if self.task_pass_door():
                self.cur_task = 'BUMP_FLARE'
        elif self.cur_task == 'BUMP_FLARE':
            if self.task_bump_flare():
                self.cur_task = 'DROP_BALL'
        elif self.cur_task == 'DROP_BALL':
            if self.task_drop_ball():
                self.cur_task = 'PICK_BALL'
        elif self.cur_task == 'PICK_BALL':
            if self.task_pick_ball():
                self.cur_task = 'DONE'
        elif self.cur_task == 'DONE':
            self.get_logger().info('DONE')
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
        else:
            self.get_logger().info('Error: Invalid task')

    def detection_cb(self, detection_msg):
        self.get_logger().info('Detected')
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
            if result.hypothesis.class_id in score_dict:
                if result.hypothesis.score < score_dict[result.hypothesis.class_id]:
                    continue
                else:
                    score_dict[result.hypothesis.class_id] = result.hypothesis.score
            # TODO: match object id
            if result.hypothesis.class_id == '0':
                if result.hypothesis.score > 0.5:
                    self.get_logger().info('Gate detected')
                    self.gate_pose = result.pose
                    self.gate_detected = True
            elif result.hypothesis.class_id == '1':
                if result.hypothesis.score > 0.5:
                    self.get_logger().info('Blue drum detected')
                    self.blue_drum_pose = result.pose
                    self.blue_drum_detected = True
            elif result.hypothesis.class_id == '2':
                if result.hypothesis.score > 0.5:
                    self.get_logger().info('Metal ball detected')
                    self.metal_ball_pose = result.pose
                    self.metal_ball_detected = True
            elif result.hypothesis.class_id == '3':
                if result.hypothesis.score > 0.5:
                    self.get_logger().info('Orange flare detected')
                    self.orange_flare_pose = result.pose
                    self.orange_flare_detected = True
            elif result.hypothesis.class_id == '4':
                if result.hypothesis.score > 0.5:
                    self.get_logger().info('Yellow flare detected')
                    self.yellow_flare_pose = result.pose
                    self.yellow_flare_detected = True
            elif result.hypothesis.class_id == '5':
                if result.hypothesis.score > 0.5:
                    self.get_logger().info('Red flare detected')
                    self.red_flare_pose = result.pose
                    self.red_flare_detected = True
            elif result.hypothesis.class_id == '6':
                if result.hypothesis.score > 0.5:
                    self.get_logger().info('Blue flare detected')
                    self.blue_flare_pose = result.pose
                    self.blue_flare_detected = True
            else:
                self.get_logger().info('Error: Invalid class_id')

        self.main_sequence()

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [self.dx, self.dy, self.dz, self.d_yaw]
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


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
