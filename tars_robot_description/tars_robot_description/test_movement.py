#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class TarsMover(Node):
    def __init__(self):
        super().__init__('tars_mover')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_names = ['joint_center_linear', 'joint_left_leg', 'joint_right_leg']
        self.get_logger().info('TARS Mover Node Started')

    def send_command(self, linear, left, right, duration_sec=1.0):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(linear), float(left), float(right)]
        point.time_from_start = Duration(sec=0, nanosec=int(duration_sec * 1e9))
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Command: Linear={linear:.3f}, Left={left:.3f}, Right={right:.3f}')
        time.sleep(duration_sec)

    def step_forward(self):
        self.get_logger().info('=== STEP FORWARD ===')
        

        self.send_command(-0.04, 0.0, 0.0, 0.5)
        

        self.send_command(-0.04, 0.26, 0.26, 0.5)
        

        self.send_command(0.04, 0.26, 0.26, 0.5)
        

        self.send_command(0.04, 0.0, 0.0, 0.5)
        

        self.send_command(0.0, 0.0, 0.0, 0.5)
        
        self.get_logger().info('=== COMPLETE ===')

def main(args=None):
    rclpy.init(args=args)
    mover = TarsMover()

    time.sleep(2)
    
    try:
        while rclpy.ok():
            mover.step_forward()
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
