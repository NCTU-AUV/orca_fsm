import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
# from gazebo_msgs.srv import SetEntityState
# from gazebo_msgs.msg import ModelStates


class Bridge(Node):

    def __init__(self):
        super().__init__('fsm_bridge')

        # default mode is real
        self.declare_parameter('mode', 'real')
        self.mode = self.get_parameter('mode').value

        if self.mode == 'real':
            self.real_x_scale = 1.0
            self.real_y_scale = 1.0
            self.real_z_scale = 1.0
            self.real_yaw_scale = 1.0
            # subscribe to fsm output
            self.sub = self.create_subscription(
                Float64MultiArray,
                'fsm_output',
                self.real_callback,
                10)
            # publish target pose to controller
            self.pub = self.create_publisher(
                Float64MultiArray,
                'target',
                10,
            )

        elif self.mode == 'sim':
            self.sim_scale = 0.2
            # subscribe to fsm output
            self.sub = self.create_subscription(
                Float64MultiArray,
                'fsm_output',
                self.sim_callback,
                10)
            # set gazebo entity state
            self.set_cli = self.create_client(
                SetEntityState, 'sauvc_sim/set_entity_state')
            while not self.set_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('sauvc_sim/set_entity_state not available, waiting again...')
            self.get_logger().info('ready')
            # subscribe to model states (get current pose of orca)
            self.current_pose = None
            self.model_state_sub = self.create_subscription(
                ModelStates,
                'sauvc_sim/model_states',
                self.model_states_callback,
                10,
            )

        else:
            raise ValueError(f'unknown mode: {self.mode}')

    def real_callback(self, msg):
        msg.data[0] *= self.real_x_scale # x
        msg.data[1] *= self.real_y_scale # y
        msg.data[2] *= self.real_z_scale # z
        msg.data[3] *= self.real_yaw_scale # yaw
        self.get_logger().info('msg: "%s"' % msg.data)
        self.pub.publish(msg)

    def sim_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().info('current pose is None')
            return

        self.get_logger().info(f'msg: {msg.data}')

        # switch x and y
        y, x, z, yaw = msg.data
        x *= self.sim_scale
        y *= self.sim_scale

        # TODO: control z and yaw

        # set velocity
        req = SetEntityState.Request()
        req.state.name = 'orca'
        req.state.pose.position = self.current_pose.position
        req.state.twist.linear = Vector3(x=x, y=y, z=0.0)
        self.set_cli.call_async(req)

    def model_states_callback(self, msg):
        self.current_pose = msg.pose[0]


def main(args=None):
    rclpy.init(args=args)

    node = Bridge()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
