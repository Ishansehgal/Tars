import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Duration
import threading
import time
import math

class TarsEnv(gym.Env):
    def __init__(self):
        super(TarsEnv, self).__init__()
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('tars_gym_env')

        # Action Space: 3 joints (Center Linear, Left Leg, Right Leg)
        # Assuming normalized action [-1, 1] mapped to joint limits
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Observation Space:
        # 3 Joint Pos + 3 Joint Vel
        # 4 Quat (Orientation) + 3 Ang Vel + 3 Lin Acc
        # Total = 6 + 10 = 16
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(16,), dtype=np.float32)

        # ROS Topics
        self.pub_action = self.node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.sub_imu = self.node.create_subscription(Imu, '/imu', self._imu_callback, 10)
        self.sub_joint = self.node.create_subscription(JointState, '/joint_states', self._joint_callback, 10)
        self.sub_pose = self.node.create_subscription(PoseWithCovarianceStamped, '/model/tars_robot/pose', self._pose_callback, 10)

        # State Variables
        self.latest_imu = None
        self.latest_joint = None
        self.latest_pose = None
        self.prev_pose_x = None
        self.lock = threading.Lock()

        # Joint Limits
        # 0: joint_center_linear (Prismatic) - Limits [-0.046, 0.046]
        # 1: joint_left_leg (Revolute) - Limits [-3.14, 3.14]
        # 2: joint_right_leg (Revolute) - Limits [-3.14, 3.14]
        
        self.joint_limits_low = np.array([-0.046, -3.14, -3.14])
        self.joint_limits_high = np.array([0.046, 3.14, 3.14])
        self.joint_names = ['joint_center_linear', 'joint_left_leg', 'joint_right_leg']

        # Start ROS Spinner in background thread
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        
        # Wait for connections
        time.sleep(1.0)

    def _imu_callback(self, msg):
        with self.lock:
            self.latest_imu = msg

    def _joint_callback(self, msg):
        with self.lock:
            # We need to map joint names to our order
            # This is simple if names match, but msg might be unordered
            # For simplicity, we assume robust mapping later
            self.latest_joint = msg

    def _pose_callback(self, msg):
        with self.lock:
            self.latest_pose = msg

    def _get_obs(self):
        # Default zero obs
        obs = np.zeros(16, dtype=np.float32)
        
        with self.lock:
            if self.latest_joint:
                # Map joints
                # Need to find index of each joint in msg
                try:
                    name_map = {name: i for i, name in enumerate(self.latest_joint.name)}
                    indices = [name_map[n] for n in self.joint_names if n in name_map]
                    
                    if len(indices) == 3:
                        pos = np.array(self.latest_joint.position)[indices]
                        vel = np.array(self.latest_joint.velocity)[indices]
                        obs[0:3] = pos
                        obs[3:6] = vel
                except:
                    pass

            if self.latest_imu:
                # Quat
                obs[6] = self.latest_imu.orientation.x
                obs[7] = self.latest_imu.orientation.y
                obs[8] = self.latest_imu.orientation.z
                obs[9] = self.latest_imu.orientation.w
                # Ang Vel
                obs[10] = self.latest_imu.angular_velocity.x
                obs[11] = self.latest_imu.angular_velocity.y
                obs[12] = self.latest_imu.angular_velocity.z
                # Lin Acc
                obs[13] = self.latest_imu.linear_acceleration.x
                obs[14] = self.latest_imu.linear_acceleration.y
                obs[15] = self.latest_imu.linear_acceleration.z
        
        return obs

    def step(self, action):
        # Clip action
        action = np.clip(action, -1.0, 1.0)
        
        # Scale to limits
        target_pos = self.joint_limits_low + (action + 1.0) * 0.5 * (self.joint_limits_high - self.joint_limits_low)
        
        # Send Command
        msg = JointTrajectory()
        msg.header.frame_id = 'base_link'
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = target_pos.tolist()
        point.time_from_start = Duration(sec=0, nanosec=100000000) # 0.1s target time
        msg.points.append(point)
        self.pub_action.publish(msg)
        
        # Wait for simulation to step (Control Frequency)
        # Using sleep for now, ideally synchronize with Sim Time
        time.sleep(0.1)
        
        # Get Obs
        obs = self._get_obs()
        
        # Calculate Reward
        reward = 0.0
        terminated = False
        truncated = False
        
        # 1. Orientation/Fall Detection
        if self.latest_imu:
            qx, qy, qz, qw = self.latest_imu.orientation.x, self.latest_imu.orientation.y, self.latest_imu.orientation.z, self.latest_imu.orientation.w
            
            # Roll (x-axis rotation)
            sinr_cosp = 2 * (qw * qx + qy * qz)
            cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
            roll = np.arctan2(sinr_cosp, cosr_cosp)
            
            # Pitch (y-axis rotation)
            sinp = 2 * (qw * qy - qz * qx)
            if abs(sinp) >= 1:
                pitch = np.copysign(np.pi / 2, sinp)
            else:
                pitch = np.arcsin(sinp)
                
            # Fall Penalty (High Pitch/Roll)
            if abs(roll) > 0.8 or abs(pitch) > 0.8:
                terminated = True
                reward -= 20.0 # Heavy Fall penalty

        # 2. Forward Velocity Reward
        # Calculate dx
        velocity_x = 0.0
        current_pos_x = 0.0
        
        if self.latest_pose:
            current_pos_x = self.latest_pose.pose.pose.position.x
            
            if self.prev_pose_x is not None:
                # dt is approx 0.1s
                velocity_x = (current_pos_x - self.prev_pose_x) / 0.1
            
            self.prev_pose_x = current_pos_x
            
        # Reward for moving forward
        # TARS is big, 0.1 m/s is decent.
        reward += velocity_x * 50.0 
        
        # Survival Reward (Keep it small so it doesn't just stand still)
        if not terminated:
            reward += 0.05
            
        # 3. Stall Penalty (Encourage movement)
        if abs(velocity_x) < 0.01:
            reward -= 0.1

        return obs, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Reset Logic
        # Send joints to 0
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)
        self.pub_action.publish(msg)
        
        time.sleep(1.0)
        
        return self._get_obs(), {}
