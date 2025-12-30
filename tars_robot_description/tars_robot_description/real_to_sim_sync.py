#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class TarsSync(Node):
    def __init__(self):
        super().__init__('tars_real_to_sim_sync')
        
        # Subscriber for Real Robot Feedback
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/servo_feedback',
            self.feedback_callback,
            10)
            
        # Publisher for Simulation Control
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
            
        self.joint_names = ['joint_center_linear', 'joint_left_leg', 'joint_right_leg']
        
        # Constants for mapping
        self.CENTER_NEUTRAL = 105
        self.LEG_NEUTRAL = 110
        
        # Max linear travel 0.046m for approx 30 deg servo delta (75 to 105)
        self.LINEAR_SCALE = 0.046 / 30.0 
        
        self.get_logger().info('TARS Sync Node Started: Listening to /servo_feedback...')

    def feedback_callback(self, msg):
        if len(msg.data) < 3:
            return

        # Raw Servo Values
        raw_center = msg.data[0]
        raw_left = msg.data[1]
        raw_right = msg.data[2]
        
        # --- MAPPING LOGIC ---
        
        # 1. Center Linear
        # 105 -> 0
        # 75 (DOWN) -> negative, 135 (UP) -> positive
        sim_center = (raw_center - self.CENTER_NEUTRAL) * self.LINEAR_SCALE
        
        # 2. Left Leg
        # 110 -> 0
        # FORWARD_L = 125 (+15) -> Positive rotation for forward motion
        sim_left = math.radians(raw_left - self.LEG_NEUTRAL)
        
        # 3. Right Leg
        # 110 -> 0
        # FORWARD_R = 95 (-15) -> Same direction as left for forward motion
        # INVERTED to match simulation coordinate frame
        sim_right = math.radians(raw_right - self.LEG_NEUTRAL) * -1.0

        # Create Trajectory Message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [sim_center, sim_left, sim_right]
        point.time_from_start = Duration(sec=0, nanosec=50000000) # 50ms (20Hz update)
        
        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)
        
        # Debug occasionally
        # self.get_logger().info(f'In: [{raw_center},{raw_left},{raw_right}] -> Out: [{sim_center:.3f}, {sim_left:.3f}, {sim_right:.3f}]')

def main(args=None):
    rclpy.init(args=args)
    sync_node = TarsSync()
    try:
        rclpy.spin(sync_node)
    except KeyboardInterrupt:
        pass
    sync_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
