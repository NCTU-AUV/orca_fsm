#!/usr/bin/env python3
"""
Modular Decision System with Configurable Basic Blocks
Clean, composable building blocks for AUV control

Save this file as: sauvc_sim/modular_blocks.py (or orca_fsm/modular_blocks.py)
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Callable, Dict, Any
from enum import Enum
from geometry_msgs.msg import Twist


# ============================================
# BASIC MOTION BLOCKS
# ============================================

class BasicBlock:
    """Base class for all basic motion blocks"""
    
    def __init__(self, name: str):
        self.name = name
        self.start_time: Optional[float] = None
        self.is_complete = False
        
    def start(self, ctx):
        """Called when block starts executing"""
        self.start_time = ctx.current_time
        self.is_complete = False
        
    def execute(self, ctx) -> Twist:
        """Execute block and return velocity command"""
        raise NotImplementedError
        
    def check_complete(self, ctx) -> bool:
        """Check if block execution is complete"""
        raise NotImplementedError
        
    def reset(self):
        """Reset block state"""
        self.start_time = None
        self.is_complete = False


@dataclass
class ExecutionContext:
    """Context available to all blocks during execution"""
    detections: Dict
    vehicle_state: Any
    current_time: float
    dt: float


class MoveForward(BasicBlock):
    """Move forward for specified distance or time"""
    
    def __init__(self, distance: Optional[float] = None, 
                 duration: Optional[float] = None,
                 speed: float = 0.3):
        """
        Args:
            distance: Target distance in meters (requires pose feedback)
            duration: Target duration in seconds
            speed: Forward speed in m/s
        """
        super().__init__(f"MoveForward({distance}m, {duration}s)")
        self.distance = distance
        self.duration = duration
        self.speed = speed
        self.start_position: Optional[float] = None
        
    def start(self, ctx):
        super().start(ctx)
        if self.distance is not None:
            # Store starting position (would need odometry/position tracking)
            self.start_position = 0.0  # Placeholder
            
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        cmd.linear.x = self.speed
        return cmd
        
    def check_complete(self, ctx) -> bool:
        if self.start_time is None:
            return False
            
        elapsed = ctx.current_time - self.start_time
        
        # Time-based completion
        if self.duration is not None:
            return elapsed >= self.duration
            
        # Distance-based completion (would need odometry)
        if self.distance is not None:
            # Placeholder: assume constant speed
            traveled = self.speed * elapsed
            return traveled >= self.distance
            
        return False


class MoveBackward(BasicBlock):
    """Move backward for specified distance or time"""
    
    def __init__(self, distance: Optional[float] = None,
                 duration: Optional[float] = None,
                 speed: float = 0.3):
        super().__init__(f"MoveBackward({distance}m, {duration}s)")
        self.distance = distance
        self.duration = duration
        self.speed = speed
        
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        cmd.linear.x = -self.speed
        return cmd
        
    def check_complete(self, ctx) -> bool:
        if self.start_time is None:
            return False
        elapsed = ctx.current_time - self.start_time
        
        if self.duration is not None:
            return elapsed >= self.duration
        if self.distance is not None:
            traveled = self.speed * elapsed
            return traveled >= self.distance
        return False


class TurnToHeading(BasicBlock):
    """Turn to absolute heading"""
    
    def __init__(self, target_heading: float, tolerance: float = 5.0, 
                 max_angular_speed: float = 0.3):
        """
        Args:
            target_heading: Target heading in degrees [0, 360)
            tolerance: Acceptable error in degrees
            max_angular_speed: Maximum angular velocity in rad/s
        """
        super().__init__(f"TurnToHeading({target_heading}°)")
        self.target_heading = target_heading
        self.tolerance = tolerance
        self.max_angular_speed = max_angular_speed
        
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        current_yaw = ctx.vehicle_state.yaw
        
        # Calculate shortest angular distance
        error = self.target_heading - current_yaw
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        # Proportional control with saturation
        kp = 0.02  # Proportional gain
        cmd.angular.z = np.clip(kp * error, 
                               -self.max_angular_speed, 
                               self.max_angular_speed)
        return cmd
        
    def check_complete(self, ctx) -> bool:
        current_yaw = ctx.vehicle_state.yaw
        error = abs(self.target_heading - current_yaw)
        if error > 180:
            error = 360 - error
        return error < self.tolerance


class TurnRelative(BasicBlock):
    """Turn relative to current heading"""
    
    def __init__(self, angle: float, tolerance: float = 5.0,
                 max_angular_speed: float = 0.3):
        """
        Args:
            angle: Relative angle in degrees (positive = left/CCW)
            tolerance: Acceptable error in degrees
        """
        super().__init__(f"TurnRelative({angle}°)")
        self.angle = angle
        self.tolerance = tolerance
        self.max_angular_speed = max_angular_speed
        self.target_heading: Optional[float] = None
        
    def start(self, ctx):
        super().start(ctx)
        # Calculate target heading when starting
        self.target_heading = (ctx.vehicle_state.yaw + self.angle) % 360
        
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        if self.target_heading is None:
            return cmd
            
        current_yaw = ctx.vehicle_state.yaw
        error = self.target_heading - current_yaw
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        kp = 0.02
        cmd.angular.z = np.clip(kp * error,
                               -self.max_angular_speed,
                               self.max_angular_speed)
        return cmd
        
    def check_complete(self, ctx) -> bool:
        if self.target_heading is None:
            return False
        current_yaw = ctx.vehicle_state.yaw
        error = abs(self.target_heading - current_yaw)
        if error > 180:
            error = 360 - error
        return error < self.tolerance


class DiveToDepth(BasicBlock):
    """Dive or surface to target depth"""
    
    def __init__(self, target_depth: float, tolerance: float = 0.2,
                 max_speed: float = 0.3):
        """
        Args:
            target_depth: Target depth in meters (positive = deeper)
            tolerance: Acceptable error in meters
            max_speed: Maximum vertical speed in m/s
        """
        super().__init__(f"DiveToDepth({target_depth}m)")
        self.target_depth = target_depth
        self.tolerance = tolerance
        self.max_speed = max_speed
        
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        current_depth = ctx.vehicle_state.depth
        error = self.target_depth - current_depth
        
        # Proportional control
        kp = 0.5
        cmd.linear.z = np.clip(kp * error, -self.max_speed, self.max_speed)
        return cmd
        
    def check_complete(self, ctx) -> bool:
        current_depth = ctx.vehicle_state.depth
        return abs(self.target_depth - current_depth) < self.tolerance


class MaintainDepth(BasicBlock):
    """Maintain current depth (useful during lateral movement)"""
    
    def __init__(self, duration: float, target_depth: Optional[float] = None,
                 max_speed: float = 0.2):
        super().__init__(f"MaintainDepth({duration}s)")
        self.duration = duration
        self.target_depth = target_depth
        self.max_speed = max_speed
        
    def start(self, ctx):
        super().start(ctx)
        if self.target_depth is None:
            self.target_depth = ctx.vehicle_state.depth
            
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        error = self.target_depth - ctx.vehicle_state.depth
        kp = 0.5
        cmd.linear.z = np.clip(kp * error, -self.max_speed, self.max_speed)
        return cmd
        
    def check_complete(self, ctx) -> bool:
        if self.start_time is None:
            return False
        return (ctx.current_time - self.start_time) >= self.duration


class Strafe(BasicBlock):
    """Move sideways (sway) for distance or time"""
    
    def __init__(self, direction: str, distance: Optional[float] = None,
                 duration: Optional[float] = None, speed: float = 0.3):
        """
        Args:
            direction: 'left' or 'right'
            distance: Target distance in meters
            duration: Target duration in seconds
            speed: Lateral speed in m/s
        """
        super().__init__(f"Strafe({direction}, {distance}m, {duration}s)")
        self.direction = direction
        self.distance = distance
        self.duration = duration
        self.speed = speed if direction == 'right' else -speed
        
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        cmd.linear.y = self.speed
        return cmd
        
    def check_complete(self, ctx) -> bool:
        if self.start_time is None:
            return False
        elapsed = ctx.current_time - self.start_time
        
        if self.duration is not None:
            return elapsed >= self.duration
        if self.distance is not None:
            traveled = abs(self.speed) * elapsed
            return traveled >= self.distance
        return False


class Hover(BasicBlock):
    """Hover in place for duration"""
    
    def __init__(self, duration: float):
        super().__init__(f"Hover({duration}s)")
        self.duration = duration
        
    def execute(self, ctx) -> Twist:
        return Twist()  # Zero velocity
        
    def check_complete(self, ctx) -> bool:
        if self.start_time is None:
            return False
        return (ctx.current_time - self.start_time) >= self.duration


# ============================================
# DETECTION-BASED BLOCKS
# ============================================

class SearchForObject(BasicBlock):
    """Search for object by rotating"""
    
    def __init__(self, object_name: str, timeout: float = 10.0,
                 angular_speed: float = 0.2, min_confidence: float = 0.5):
        super().__init__(f"SearchFor({object_name})")
        self.object_name = object_name
        self.timeout = timeout
        self.angular_speed = angular_speed
        self.min_confidence = min_confidence
        self.found = False
        
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        
        # Check if object is detected
        det = ctx.detections.get(self.object_name)
        if det and det.detected and det.confidence >= self.min_confidence:
            self.found = True
            return cmd  # Stop rotating
            
        # Keep rotating to search
        cmd.angular.z = self.angular_speed
        return cmd
        
    def check_complete(self, ctx) -> bool:
        if self.start_time is None:
            return False
            
        # Complete if found or timeout
        if self.found:
            return True
            
        elapsed = ctx.current_time - self.start_time
        return elapsed >= self.timeout


class AlignToObject(BasicBlock):
    """Align to object center (without moving forward)"""
    
    def __init__(self, object_name: str, 
                 tolerance_x: float = 0.1, 
                 tolerance_y: float = 0.1,
                 timeout: float = 5.0,
                 kp_x: float = 0.5,
                 kp_y: float = 0.3):
        super().__init__(f"AlignTo({object_name})")
        self.object_name = object_name
        self.tolerance_x = tolerance_x
        self.tolerance_y = tolerance_y
        self.timeout = timeout
        self.kp_x = kp_x
        self.kp_y = kp_y
        
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        det = ctx.detections.get(self.object_name)
        
        if not det or not det.detected:
            return cmd
            
        # Calculate errors from center (0.5, 0.5)
        error_x = det.center_x - 0.5
        error_y = det.center_y - 0.5
        
        # Control lateral and vertical movement
        cmd.linear.y = -self.kp_x * error_x
        cmd.linear.z = -self.kp_y * error_y
        
        return cmd
        
    def check_complete(self, ctx) -> bool:
        if self.start_time is None:
            return False
            
        det = ctx.detections.get(self.object_name)
        if not det or not det.detected:
            # Timeout if object lost
            elapsed = ctx.current_time - self.start_time
            return elapsed >= self.timeout
            
        # Check alignment
        error_x = abs(det.center_x - 0.5)
        error_y = abs(det.center_y - 0.5)
        return error_x < self.tolerance_x and error_y < self.tolerance_y


class ApproachObject(BasicBlock):
    """Approach object to target distance"""
    
    def __init__(self, object_name: str,
                 target_distance: float = 1.5,
                 tolerance: float = 0.3,
                 timeout: float = 10.0,
                 kp_forward: float = 0.5,
                 kp_lateral: float = 0.5,
                 kp_vertical: float = 0.3):
        super().__init__(f"Approach({object_name}, {target_distance}m)")
        self.object_name = object_name
        self.target_distance = target_distance
        self.tolerance = tolerance
        self.timeout = timeout
        self.kp_forward = kp_forward
        self.kp_lateral = kp_lateral
        self.kp_vertical = kp_vertical
        
    def execute(self, ctx) -> Twist:
        cmd = Twist()
        det = ctx.detections.get(self.object_name)
        
        if not det or not det.detected:
            return cmd
            
        # Alignment
        error_x = det.center_x - 0.5
        error_y = det.center_y - 0.5
        cmd.linear.y = -self.kp_lateral * error_x
        cmd.linear.z = -self.kp_vertical * error_y
        
        # Forward control based on depth or pose
        if det.pose_z is not None:
            # Use 3D pose if available
            depth_error = det.pose_z - self.target_distance
            cmd.linear.x = self.kp_forward * depth_error
        elif det.depth is not None:
            # Use depth image
            depth_error = det.depth - self.target_distance
            cmd.linear.x = self.kp_forward * depth_error
        else:
            # Use size as proxy for distance
            target_size = 0.3  # Target 30% of frame width
            size_error = target_size - det.width
            cmd.linear.x = self.kp_forward * size_error * 2.0
            
        return cmd
        
    def check_complete(self, ctx) -> bool:
        if self.start_time is None:
            return False
            
        det = ctx.detections.get(self.object_name)
        
        # Timeout check
        elapsed = ctx.current_time - self.start_time
        if elapsed >= self.timeout:
            return True
            
        if not det or not det.detected:
            return False
            
        # Check if at target distance
        current_dist = det.depth or det.pose_z
        if current_dist is None:
            return False
            
        return abs(current_dist - self.target_distance) < self.tolerance


class WaitForDetection(BasicBlock):
    """Wait until object is detected"""
    
    def __init__(self, object_name: str, timeout: float = 5.0,
                 min_confidence: float = 0.5):
        super().__init__(f"WaitFor({object_name})")
        self.object_name = object_name
        self.timeout = timeout
        self.min_confidence = min_confidence
        
    def execute(self, ctx) -> Twist:
        return Twist()  # Hover in place
        
    def check_complete(self, ctx) -> bool:
        if self.start_time is None:
            return False
            
        det = ctx.detections.get(self.object_name)
        if det and det.detected and det.confidence >= self.min_confidence:
            return True
            
        # Timeout
        elapsed = ctx.current_time - self.start_time
        return elapsed >= self.timeout


# ============================================
# BLOCK SEQUENCER
# ============================================

class BlockSequence:
    """Execute a sequence of basic blocks"""
    
    def __init__(self, name: str, blocks: list):
        self.name = name
        self.blocks = blocks
        self.current_index = 0
        self.is_complete = False
        
    def start(self, ctx):
        """Start the sequence"""
        self.current_index = 0
        self.is_complete = False
        if self.blocks:
            self.blocks[0].start(ctx)
            
    def execute(self, ctx) -> Twist:
        """Execute current block"""
        if self.current_index >= len(self.blocks):
            self.is_complete = True
            return Twist()
            
        current_block = self.blocks[self.current_index]
        cmd = current_block.execute(ctx)
        
        # Check if current block is complete
        if current_block.check_complete(ctx):
            self.current_index += 1
            if self.current_index < len(self.blocks):
                # Start next block
                self.blocks[self.current_index].start(ctx)
            else:
                self.is_complete = True
                
        return cmd
        
    def reset(self):
        """Reset the sequence"""
        self.current_index = 0
        self.is_complete = False
        for block in self.blocks:
            block.reset()


# ============================================
# TASK DEFINITIONS USING BLOCKS
# ============================================

def create_gate_passing_sequence():
    """Example: Pass through gate using basic blocks"""
    return BlockSequence("PassGate", [
        SearchForObject("gate", timeout=10.0),
        AlignToObject("gate", tolerance_x=0.1, tolerance_y=0.1),
        ApproachObject("gate", target_distance=2.0, tolerance=0.3),
        AlignToObject("gate", tolerance_x=0.08, tolerance_y=0.08),
        MoveForward(distance=3.0, speed=0.4),
        Hover(duration=1.0)
    ])


def create_waypoint_navigation_sequence():
    """Example: Navigate through waypoints"""
    return BlockSequence("Waypoints", [
        DiveToDepth(target_depth=1.5),
        MoveForward(distance=5.0),
        TurnRelative(angle=90),
        MoveForward(distance=3.0),
        TurnRelative(angle=90),
        MoveForward(distance=5.0),
        TurnToHeading(target_heading=0),
        DiveToDepth(target_depth=0.5)
    ])


def create_object_inspection_sequence(object_name: str):
    """Example: Find and inspect object"""
    return BlockSequence(f"Inspect_{object_name}", [
        SearchForObject(object_name, timeout=15.0),
        AlignToObject(object_name),
        ApproachObject(object_name, target_distance=1.0),
        Hover(duration=2.0),  # Inspect
        MoveBackward(duration=2.0)
    ])


def create_search_pattern_sequence():
    """Example: Systematic search pattern"""
    return BlockSequence("SearchPattern", [
        DiveToDepth(1.5),
        MoveForward(distance=10.0),
        TurnRelative(90),
        MoveForward(distance=2.0),
        TurnRelative(90),
        MoveForward(distance=10.0),
        TurnRelative(-90),
        MoveForward(distance=2.0),
        TurnRelative(-90)
    ])


# ============================================
# INTEGRATION WITH DECISION NODE
# ============================================

class ModularDecisionRule:
    """Clean decision rule using block sequences"""
    
    def __init__(self):
        self.current_sequence: Optional[BlockSequence] = None
        self.sequences = {
            'pass_gate': create_gate_passing_sequence(),
            'waypoints': create_waypoint_navigation_sequence(),
            'inspect_drum': create_object_inspection_sequence('blue_drum'),
            'search': create_search_pattern_sequence()
        }
        self.active_task = None
        
    def start_task(self, task_name: str, ctx):
        """Start a named task sequence"""
        if task_name in self.sequences:
            self.current_sequence = self.sequences[task_name]
            self.current_sequence.reset()
            self.current_sequence.start(ctx)
            self.active_task = task_name
            return True
        return False
        
    def execute(self, ctx) -> tuple:
        """
        Execute current sequence
        Returns: (cmd_vel, gripper_cmd, is_complete)
        """
        if self.current_sequence is None:
            return Twist(), False, True
            
        cmd_vel = self.current_sequence.execute(ctx)
        gripper_cmd = False  # Can be extended
        is_complete = self.current_sequence.is_complete
        
        return cmd_vel, gripper_cmd, is_complete


# ============================================
# USAGE EXAMPLE
# ============================================

"""
# In your AUVDecisionNode, replace decision_rule with:

class AUVDecisionNode(Node):
    def __init__(self):
        # ... existing init ...
        self.modular_decision = ModularDecisionRule()
        self.task_queue = ['pass_gate', 'inspect_drum', 'waypoints']
        self.task_index = 0
        
    def decision_loop(self):
        # Create execution context
        exec_ctx = ExecutionContext(
            detections=self.decision_context.detections,
            vehicle_state=self.decision_context.vehicle_state,
            current_time=time.time(),
            dt=self.decision_context.dt
        )
        
        # Start next task if needed
        if self.modular_decision.current_sequence is None:
            if self.task_index < len(self.task_queue):
                task = self.task_queue[self.task_index]
                self.modular_decision.start_task(task, exec_ctx)
                self.get_logger().info(f'Starting task: {task}')
        
        # Execute current task
        cmd_vel, gripper_cmd, is_complete = self.modular_decision.execute(exec_ctx)
        
        # Move to next task when complete
        if is_complete:
            self.get_logger().info(f'Task {self.task_queue[self.task_index]} complete')
            self.task_index += 1
            self.modular_decision.current_sequence = None
        
        # Publish commands
        self.publish_commands(cmd_vel, gripper_cmd)
"""