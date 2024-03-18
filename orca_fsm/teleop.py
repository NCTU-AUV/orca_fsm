import time
import sys
import rclpy
from rclpy.node import Node
from select import select
import termios
import tty
import threading
from std_msgs.msg import Float64MultiArray


class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_ctrl')

        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.d_yaw = 0.0
        # arm control 0: idle, 1: drop, 2: stretch out, 3: stretch in
        self.arm_state = 0

        self.LIN_VEL_STEP = 1.0
        self.ANG_VEL_STEP = 1.0

        self.settings = termios.tcgetattr(sys.stdin)
        self.key_timeout = 0.5

        self.pub = self.create_publisher(
            Float64MultiArray,
            'fsm_output',
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
            self.dx = self.LIN_VEL_STEP
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
            self.get_logger().info("front")
        elif key == 'a':
            self.dx = 0.0
            self.dy = -self.LIN_VEL_STEP
            self.dz = 0.0
            self.d_yaw = 0.0
            self.get_logger().info("left")
        elif key == 's':
            self.dx = -self.LIN_VEL_STEP
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
            self.get_logger().info("back")
        elif key == 'd':
            self.dx = 0.0
            self.dy = self.LIN_VEL_STEP
            self.dz = 0.0
            self.d_yaw = 0.0
            self.get_logger().info("right")
        elif key == 'k':
            self.dx = 0.0
            self.dy = 0.0
            self.dz = self.LIN_VEL_STEP
            self.d_yaw = 0.0
            self.get_logger().info("up")
        elif key == 'j':
            self.dx = 0.0
            self.dy = 0.0
            self.dz = -self.LIN_VEL_STEP
            self.d_yaw = 0.0
            self.get_logger().info("down")
        elif key == 'h':
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = self.ANG_VEL_STEP
            self.get_logger().info("yaw_left")
        elif key == 'l':
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = -self.ANG_VEL_STEP
            self.get_logger().info("yaw_right")
        elif key == 'q':
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.d_yaw = 0.0
            self.get_logger().info("stop")
        elif key == 'u':
            self.arm_state = 0
            self.get_logger().info("idle")
        elif key == 'i':
            self.arm_state = 1
            self.get_logger().info("drop")
        elif key == 'o':
            self.arm_state = 2
            self.get_logger().info("stretch out")
        elif key == 'p':
            self.arm_state = 3
            self.get_logger().info("stretch in")

        msg = Float64MultiArray()
        msg.data = [self.dx, self.dy, self.dz, self.d_yaw]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = KeyboardController()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    rate = node.create_rate(5)
    while rclpy.ok():
        node.run()
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
