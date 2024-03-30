import time
import sys
import rclpy
from rclpy.node import Node
from select import select
import termios
import tty
import threading
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor


class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_ctrl')

        self.dx = 0.0
        self.dy = 0.0
        self.depth = 0.1
        self.target_yaw = 0.0
        self.drop = 0.0
        self.arm_mode = 0.0

        self.LIN_VEL_STEP = 1.0
        self.DEP_VEL_STEP = 0.1
        self.ANG_VEL_STEP = 10.0

        self.settings = termios.tcgetattr(sys.stdin)
        self.key_timeout = 0.5

        self.pub = self.create_publisher(
            Float32MultiArray,
            'target',
            10)

        self.real_yaw = 0.0
        self.real_arm_done = 0.0
        self.real_depth = 0.0

        # 0: yaw, 1: arm, 2: depth
        self.sub = self.create_subscription(
            Float32MultiArray,
            'rpi_to_oring',
            self.callback,
            10)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], self.key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        key = self.get_key()
        if key == 'w':
            if (not self.dx == 0.0):
                self.dx += self.LIN_VEL_STEP
            else:
                self.dx = self.LIN_VEL_STEP
            self.dy = 0.0
            self.get_logger().info("front")
        elif key == 'a':
            self.dx = 0.0
            self.dy = -self.LIN_VEL_STEP
            self.get_logger().info("left")
        elif key == 's':
            if (not self.dx == 0.0):
                self.dx -= self.LIN_VEL_STEP
            else:
                self.dx = -self.LIN_VEL_STEP
            self.dy = 0.0
            self.get_logger().info("back")
        elif key == 'd':
            self.dx = 0.0
            self.dy = self.LIN_VEL_STEP
            self.get_logger().info("right")
        elif key == 'k':
            if (self.depth - self.DEP_VEL_STEP) >= 0.0:
                self.depth -= self.DEP_VEL_STEP
                self.get_logger().info("up: depth = %f" % self.depth)
        elif key == 'j':
            self.depth += self.DEP_VEL_STEP
            self.get_logger().info("down: depth = %f" % self.depth)
        elif key == 'h':
            self.target_yaw -= self.ANG_VEL_STEP
            self.get_logger().info("turn_left: yaw = %f" % self.target_yaw)
        elif key == 'l':
            self.target_yaw += self.ANG_VEL_STEP
            self.get_logger().info("turn_right: yaw = %f" % self.target_yaw)
        elif key == 'q':
            self.dx = 0.0
            self.dy = 0.0
            self.get_logger().info("stop")
        elif key == 'u':
            if (self.drop == 1.0):
                self.drop = 0.0
                self.get_logger().info("drop close")
            else:
                self.drop = 1.0
                self.get_logger().info("drop open")
        elif key == 'i':
            self.arm_mode = 0.0
            self.get_logger().info("arm idle")
        elif key == 'o':
            self.arm_mode = 1.0
            self.get_logger().info("arm grab")
        elif key == 'p':
            self.arm_mode = 2.0
            self.get_logger().info("arm release")

        msg = Float32MultiArray()
        msg.data = [self.dx, self.dy, self.depth, self.target_yaw, self.drop, self.arm_mode]
        self.pub.publish(msg)

    def callback(self, msg):
        self.real_yaw = msg.data[0]
        self.real_arm_done = msg.data[1]
        self.real_depth = msg.data[2]
        self.water_com = msg.data[3]

def main():
    rclpy.init()
    node = KeyboardController()
    executor = MultiThreadedExecutor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node, executor,))
    spin_thread.start()

    rate = node.create_rate(5)
    while rclpy.ok():
        node.run()
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
