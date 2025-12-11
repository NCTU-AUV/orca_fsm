#!/usr/bin/env python3
"""
AUV Decision Making Node
Supports both Gazebo simulation and real hardware with topic remapping
Uses modular basic blocks for clean decision programming
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, Optional, Callable
import time

from geometry_msgs.msg import Twist, PoseArray, Pose
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32MultiArray, Bool
from cv_bridge import CvBridge


class TaskState(Enum):
    """Define all possible task states"""
    IDLE = "idle"
    SEARCHING = "searching"
    APPROACHING = "approaching"
    ALIGNING = "aligning"
    EXECUTING = "executing"
    COMPLETING = "completing"
    DONE = "done"


@dataclass
class DetectionData:
    """Store detection information for a single object"""
    detected: bool = False
    center_x: float = 0.0  # normalized [0, 1]
    center_y: float = 0.0  # normalized [0, 1]
    width: float = 0.0     # normalized [0, 1]
    height: float = 0.0    # normalized [0, 1]
    confidence: float = 0.0
    depth: Optional[float] = None  # meters from depth image
    # 3D pose from depth_estimator
    pose_x: Optional[float] = None  # meters in camera frame
    pose_y: Optional[float] = None
    pose_z: Optional[float] = None
    timestamp: float = 0.0


@dataclass
class VehicleState:
    """Store vehicle feedback data"""
    yaw: float = 0.0           # degrees
    depth: float = 0.0         # meters
    arm_status: float = 0.0    # 0: idle, 1: grabbing, 2: releasing
    has_object: bool = False   # gripper status
    timestamp: float = 0.0


@dataclass
class DecisionContext:
    """Context passed to decision rules"""
    detections: Dict[str, DetectionData] = field(default_factory=dict)
    vehicle_state: VehicleState = field(default_factory=VehicleState)
    rgb_available: bool = False
    depth_available: bool = False
    pose_available: bool = False  # from depth_estimator
    camera_calibrated: bool = False
    dt: float = 0.0  # time since last update


class AUVDecisionNode(Node):
    """
    Main decision-making node for AUV control
    """
    
    # Class names mapping
    CLASS_NAMES = {
        0: 'blue_drum',
        1: 'blue_flare',
        2: 'gate',
        3: 'orange_flare',
        4: 'red_flare',
        5: 'yellow_flare',
    }
    
    def __init__(self):
        super().__init__('auv_decision_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize variables
        self.bridge = CvBridge()
        self.decision_context = DecisionContext()
        self.current_state = TaskState.IDLE
        self.state_start_time = time.time()
        self.last_update_time = time.time()
        
        # Velocity limits
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        
        # Initialize detection storage
        for name in self.CLASS_NAMES.values():
            self.decision_context.detections[name] = DetectionData()
        
        # Setup QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers with remapping support
        self._setup_subscribers(sensor_qos)
        
        # Publishers
        self._setup_publishers()
        
        # Timer for decision loop
        control_rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / control_rate, self.decision_loop)
        
        # Import modular decision system
        from orca_fsm.modular_blocks import ModularDecisionRule, ExecutionContext
        self.modular_decision = ModularDecisionRule()
        self.ExecutionContext = ExecutionContext
        
        # Task queue - define your mission here
        self.task_queue = ['pass_gate']  # Add more tasks as needed
        self.task_index = 0
        
        self.get_logger().info(f'AUV Decision Node initialized in {self.get_parameter("mode").value} mode')
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        self.get_logger().info(f'Mission tasks: {self.task_queue}')

    def _declare_parameters(self):
        """Declare all ROS parameters"""
        # Mode selection
        self.declare_parameter('mode', 'sim')  # 'sim' or 'real'
        
        # Topic names (with defaults for simulation)
        self.declare_parameter('rgb_topic', '/realsense/image')
        self.declare_parameter('depth_topic', '/realsense/depth_image')
        self.declare_parameter('camera_info_topic', '/realsense/camera_info')
        self.declare_parameter('detection_topic', 'detections_output')
        self.declare_parameter('object_pose_topic', '/object_pose')
        self.declare_parameter('feedback_topic', 'rpi_to_oring')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('gripper_topic', 'gripper_command')
        
        # Control parameters
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 0.5)
        
        # Detection parameters
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_scale', 0.001)  # mm to meters
        
        # Camera calibration
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)

    def _setup_subscribers(self, sensor_qos):
        """Setup all subscribers with topic remapping"""
        # RGB camera
        self.rgb_sub = self.create_subscription(
            Image,
            self.get_parameter('rgb_topic').value,
            self.rgb_callback,
            sensor_qos
        )
        
        # Depth camera
        if self.get_parameter('use_depth').value:
            self.depth_sub = self.create_subscription(
                Image,
                self.get_parameter('depth_topic').value,
                self.depth_callback,
                sensor_qos
            )
            
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                self.get_parameter('camera_info_topic').value,
                self.camera_info_callback,
                10
            )
        
        # Object poses from depth_estimator
        self.object_pose_sub = self.create_subscription(
            PoseArray,
            self.get_parameter('object_pose_topic').value,
            self.object_pose_callback,
            10
        )
        
        # Detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.get_parameter('detection_topic').value,
            self.detection_callback,
            10
        )
        
        # Vehicle feedback
        self.feedback_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('feedback_topic').value,
            self.feedback_callback,
            10
        )

    def _setup_publishers(self):
        """Setup all publishers"""
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10
        )
        
        self.gripper_pub = self.create_publisher(
            Bool,
            self.get_parameter('gripper_topic').value,
            10
        )

    # ==================== CALLBACKS ====================
    
    def rgb_callback(self, msg: Image):
        """Process RGB image"""
        self.decision_context.rgb_available = True
        # Store or process RGB if needed for advanced vision
        # For now, we rely on detection results

    def depth_callback(self, msg: Image):
        """Process depth image"""
        self.decision_context.depth_available = True
        # Convert depth image to numpy for processing
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Store for depth lookup in detections
            self._current_depth_image = depth_image
        except Exception as e:
            self.get_logger().warn(f'Failed to process depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """Process camera calibration info"""
        self.decision_context.camera_calibrated = True
        self._camera_info = msg

    def object_pose_callback(self, msg: PoseArray):
        """
        Process 3D object poses from depth_estimator node
        Assumes poses are ordered matching detection order or includes identifier
        """
        self.decision_context.pose_available = True
        
        # Store poses temporarily - will be matched with detections
        self._latest_poses = {}
        
        # If your depth_estimator includes a way to identify which pose belongs to which object
        # (e.g., via frame_id or a custom message), parse it here
        # For now, we'll store by index and match in detection_callback
        for idx, pose in enumerate(msg.poses):
            self._latest_poses[idx] = {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'timestamp': time.time()
            }

    def detection_callback(self, msg: Detection2DArray):
        """Process object detections"""
        # Reset all detections
        for det in self.decision_context.detections.values():
            det.detected = False
        
        # Get image dimensions
        img_width = self.get_parameter('camera_width').value
        img_height = self.get_parameter('camera_height').value
        
        # Process each detection
        min_conf = self.get_parameter('min_confidence').value
        
        detection_idx = 0  # Track detection index for pose matching
        for detection in msg.detections:
            if not detection.results:
                continue
                
            result = detection.results[0]
            class_id = int(result.hypothesis.class_id)
            confidence = result.hypothesis.score
            
            # Check if this is a known class and meets confidence threshold
            if class_id not in self.CLASS_NAMES:
                continue
            if confidence < min_conf:
                detection_idx += 1
                continue
            
            obj_name = self.CLASS_NAMES[class_id]
            
            # Extract normalized coordinates
            center_x = detection.bbox.center.position.x / img_width
            center_y = detection.bbox.center.position.y / img_height
            width = detection.bbox.size_x / img_width
            height = detection.bbox.size_y / img_height
            
            # Get depth at detection center if available
            depth_val = None
            if hasattr(self, '_current_depth_image') and self.decision_context.depth_available:
                try:
                    pixel_x = int(detection.bbox.center.position.x)
                    pixel_y = int(detection.bbox.center.position.y)
                    
                    if (0 <= pixel_y < self._current_depth_image.shape[0] and 
                        0 <= pixel_x < self._current_depth_image.shape[1]):
                        depth_raw = self._current_depth_image[pixel_y, pixel_x]
                        depth_scale = self.get_parameter('depth_scale').value
                        depth_val = float(depth_raw) * depth_scale
                except Exception as e:
                    self.get_logger().debug(f'Failed to get depth for {obj_name}: {e}')
            
            # Get 3D pose from depth_estimator if available
            pose_x, pose_y, pose_z = None, None, None
            if hasattr(self, '_latest_poses') and detection_idx in self._latest_poses:
                pose_data = self._latest_poses[detection_idx]
                pose_x = pose_data['x']
                pose_y = pose_data['y']
                pose_z = pose_data['z']
            
            # Update detection data
            self.decision_context.detections[obj_name] = DetectionData(
                detected=True,
                center_x=center_x,
                center_y=center_y,
                width=width,
                height=height,
                confidence=confidence,
                depth=depth_val,
                pose_x=pose_x,
                pose_y=pose_y,
                pose_z=pose_z,
                timestamp=time.time()
            )
            
            detection_idx += 1

    def feedback_callback(self, msg: Float32MultiArray):
        """Process vehicle feedback"""
        if len(msg.data) >= 4:
            self.decision_context.vehicle_state = VehicleState(
                yaw=msg.data[0],
                arm_status=msg.data[1],
                depth=msg.data[2],
                has_object=msg.data[3] > 0.5,
                timestamp=time.time()
            )

    # ==================== DECISION LOGIC ====================
    
    def decision_loop(self):
        """Main decision loop - called at control rate"""
        # Update timing
        current_time = time.time()
        self.decision_context.dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Create execution context for blocks
        exec_ctx = self.ExecutionContext(
            detections=self.decision_context.detections,
            vehicle_state=self.decision_context.vehicle_state,
            current_time=current_time,
            dt=self.decision_context.dt
        )
        
        # Start next task if needed
        if self.modular_decision.current_sequence is None:
            if self.task_index < len(self.task_queue):
                task = self.task_queue[self.task_index]
                success = self.modular_decision.start_task(task, exec_ctx)
                if success:
                    self.get_logger().info(f'Starting task {self.task_index + 1}/{len(self.task_queue)}: {task}')
                else:
                    self.get_logger().error(f'Unknown task: {task}')
                    self.task_index += 1
            else:
                # All tasks complete
                self.get_logger().info('All tasks completed!', throttle_duration_sec=5.0)
        
        # Execute current task
        if self.modular_decision.current_sequence is not None:
            cmd_vel, gripper_cmd, is_complete = self.modular_decision.execute(exec_ctx)
            
            # Log current block
            if hasattr(self.modular_decision.current_sequence, 'blocks'):
                current_idx = self.modular_decision.current_sequence.current_index
                if current_idx < len(self.modular_decision.current_sequence.blocks):
                    block = self.modular_decision.current_sequence.blocks[current_idx]
                    self.get_logger().info(
                        f'Executing: {block.name}',
                        throttle_duration_sec=1.0
                    )
            
            # Move to next task when complete
            if is_complete:
                task_name = self.task_queue[self.task_index] if self.task_index < len(self.task_queue) else 'unknown'
                self.get_logger().info(f'Task complete: {task_name}')
                self.task_index += 1
                self.modular_decision.current_sequence = None
        else:
            # No active task
            cmd_vel = Twist()
            gripper_cmd = False
        
        # Publish commands
        self.publish_commands(cmd_vel, gripper_cmd)

    def publish_commands(self, cmd_vel: Twist, gripper_cmd: bool):
        """Publish velocity and gripper commands"""
        # Clamp velocities to limits
        cmd_vel.linear.x = np.clip(cmd_vel.linear.x, -self.max_linear, self.max_linear)
        cmd_vel.linear.y = np.clip(cmd_vel.linear.y, -self.max_linear, self.max_linear)
        cmd_vel.linear.z = np.clip(cmd_vel.linear.z, -self.max_linear, self.max_linear)
        cmd_vel.angular.z = np.clip(cmd_vel.angular.z, -self.max_angular, self.max_angular)
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        gripper_msg = Bool()
        gripper_msg.data = gripper_cmd
        self.gripper_pub.publish(gripper_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AUVDecisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()