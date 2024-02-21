import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray

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
        if self.cur_task == 'DONE':
            self.get_logger().info('DONE')
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
        elif self.cur_task == 'PASS_DOOR':
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
        else:
            self.get_logger().info('Error: Invalid task')

    def detection_cb(self, detection_msg):
        self.get_logger().info('Detected')
        # TODO: Implement detection callback
        self.main_sequence()

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [self.dx, self.dy, self.dz, self.d_yaw]
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    fsm = FSM()
    executor = MultiThreadedExecutor()
    rclpy.spin(fsm, executor)
    fsm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
