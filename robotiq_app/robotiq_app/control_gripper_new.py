#!/usr/bin/env python3
"""
Python script to control the Robotiq gripper using ROS2 action interface.

This script provides a simple interface to:
- Open/close the gripper to specific positions
- Control speed and effort (force)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand
from std_srvs.srv import Trigger
import sys
import argparse


class RobotiqGripperController(Node):
    """ROS2 node for controlling Robotiq gripper."""

    def __init__(self):
        super().__init__('robotiq_gripper_controller')
        
        # Action client for gripper control
        self._action_client = ActionClient(
            self,
            ParallelGripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )
        
        
        self.get_logger().info('Robotiq Gripper Controller initialized')



    def send_gripper_command(self, position, max_effort=50.0, timeout_sec=10.0):
        """
        Send a position command to the gripper.
        
        Args:
            position: Target position in meters
                     - 0.0 = fully closed
                     - 0.085 = fully open (for 2F-85)
                     - 0.140 = fully open (for 2F-140)
            max_effort: Maximum effort in Newtons (default: 50.0)
            timeout_sec: Timeout for action
            
        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info('Waiting for gripper action server...')
        
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Gripper action server not available')
            return False
        
        # Create goal for ParallelGripperCommand
        # ParallelGripperCommand uses JointState which requires arrays
        goal_msg = ParallelGripperCommand.Goal()
        goal_msg.command.position = [position]
        goal_msg.command.effort = [max_effort]
        
        self.get_logger().info(f'Sending gripper command: position={position:.4f}m, effort={max_effort}N')
        
        # Send goal
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        
        result = result_future.result()
        if result:
            # ParallelGripperCommand result has state as JointState with arrays
            state = result.result.state
            pos = state.position[0] if state.position else 0.0
            eff = state.effort[0] if state.effort else 0.0
            self.get_logger().info(
                f'Gripper command completed: '
                f'position={pos:.4f}m, '
                f'effort={eff:.2f}N, '
                f'stalled={result.result.stalled}, '
                f'reached_goal={result.result.reached_goal}'
            )
            return result.result.reached_goal
        else:
            self.get_logger().error('Failed to get result')
            return False

    def open_gripper(self, max_effort=50.0):
        """
        Fully open the gripper.
        
        Args:
            max_effort: Maximum effort in Newtons
            
        Returns:
            bool: True if successful
        """
        
        self.get_logger().info('Opening gripper...')
        # For 2F-85: 0.085m, for 2F-140: 0.140m
        # Using 0.085 as default (most common model)
        return self.send_gripper_command(position=0.085, max_effort=max_effort)

    def close_gripper(self, max_effort=50.0):
        """
        Fully close the gripper.
        
        Args:
            max_effort: Maximum effort in Newtons
            
        Returns:
            bool: True if successful
        """
        self.get_logger().info('Closing gripper...')
        return self.send_gripper_command(position=0.0, max_effort=max_effort)

    def set_gripper_position(self, position, max_effort=50.0):
        """
        Move gripper to a specific position.
        
        Args:
            position: Position in meters (0.0 to 0.085 for 2F-85)
            max_effort: Maximum effort in Newtons
            
        Returns:
            bool: True if successful
        """
        return self.send_gripper_command(position=position, max_effort=max_effort)


def main(args=None):
    """Main function for gripper control."""
    
    parser = argparse.ArgumentParser(
        description='Control Robotiq gripper',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:

  
  # Open the gripper
  python3 control_gripper.py --open
  
  # Close the gripper
  python3 control_gripper.py --close
  
  # Set gripper to specific position (in meters)
  python3 control_gripper.py --position 0.04
  
  # Set position with custom effort
  python3 control_gripper.py --position 0.02 --effort 100.0
        """
    )
    

    parser.add_argument('--open', action='store_true',
                       help='Open the gripper')
    parser.add_argument('--close', action='store_true',
                       help='Close the gripper')
    parser.add_argument('--position', type=float, metavar='METERS',
                       help='Move to specific position (0.0=closed, 0.085=open for 2F-85)')
    parser.add_argument('--effort', type=float, default=50.0, metavar='NEWTONS',
                       help='Maximum effort in Newtons (default: 50.0)')
    
    # Parse known args to allow ROS args
    parsed_args, ros_args = parser.parse_known_args()
    
    # Initialize ROS
    rclpy.init(args=ros_args)
    
    controller = RobotiqGripperController()
    
    try:
        if parsed_args.open:
            success = controller.open_gripper(max_effort=parsed_args.effort)
        elif parsed_args.close:
            success = controller.close_gripper(max_effort=parsed_args.effort)
        elif parsed_args.position is not None:
            success = controller.set_gripper_position(
                position=parsed_args.position,
                max_effort=parsed_args.effort
            )
        
        # If no command specified, show help
        if not (parsed_args.open or 
                parsed_args.close or parsed_args.position is not None):
            parser.print_help()
            controller.get_logger().info(
                'No command specified. Use --help for usage information.'
            )
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
        sys.exit(0)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
