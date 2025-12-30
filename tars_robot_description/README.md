# TARS Digital Twin - Complete Technical Documentation

## Overview

This project creates a **Digital Twin** of your TARS robot - a bipedal walking robot with:
- 2 legs with rotating servos
- 1 center body with a linear servo
- Real-time feedback from the physical robot to the simulation

---

## Part 1: URDF Robot Model

### 1.1 What is URDF?

URDF (Unified Robot Description Format) is an XML format that describes a robot's physical structure:
- **Links**: The rigid bodies (legs, body)
- **Joints**: The connections between links (how they move)
- **Inertial**: Mass, center of mass, and inertia tensors

### 1.2 Defining Links (Robot Parts)

Each link has three key sections:

```xml
<link name="base_link">
  <!-- INERTIAL: Physics properties -->
  <inertial>
    <mass value="1.5"/>  <!-- Mass in kg -->
    <origin xyz="0 0 -0.08"/>  <!-- Center of Mass offset -->
    <inertia ixx="..." iyy="..." izz="..."/>  <!-- Rotation resistance -->
  </inertial>
  
  <!-- VISUAL: What you see in simulation -->
  <visual>
    <geometry><box size="0.065 0.11 0.255"/></geometry>
  </visual>
  
  <!-- COLLISION: What physics engine uses -->
  <collision>
    <geometry><box size="0.065 0.11 0.255"/></geometry>
  </collision>
</link>
```

### 1.3 Defining Joints (How Parts Move)

**Prismatic Joint** (Linear motion - center servo):
```xml
<joint name="joint_center_linear" type="prismatic">
  <parent link="base_link"/>
  <child link="leg_carriage"/>
  <axis xyz="0 0 1"/>  <!-- Moves along Z (up/down) -->
  <limit lower="-0.046" upper="0.046"/>  <!-- Travel range in meters -->
  <dynamics damping="10.0" friction="5.0"/>  <!-- Resistance -->
</joint>
```

**Revolute Joint** (Rotation - leg servos):
```xml
<joint name="joint_left_leg" type="revolute">
  <parent link="leg_carriage"/>
  <child link="left_leg"/>
  <axis xyz="0 1 0"/>  <!-- Rotates around Y axis -->
  <limit lower="-3.14" upper="3.14"/>  <!-- Range in radians -->
  <dynamics damping="5.0" friction="2.0"/>
</joint>
```

### 1.4 Center of Mass (COM) Tuning

The COM determines where the robot's weight is concentrated. If it's too high, the robot tips easily.

**Original Problem**: Robot kept tipping over.

**Solution**: Lower the COM by offsetting the inertial origin:
```xml
<inertial>
  <origin xyz="0 0 -0.08"/>  <!-- 8cm below geometric center -->
</inertial>
```

**Why -0.08?** The body is 25.5cm tall. Lowering COM by 8cm puts the weight in the lower third, simulating a heavy bottom.

### 1.5 Damping & Friction

- **Damping**: Resistance to motion (like shock absorbers). Higher = slower, smoother movement
- **Friction**: Resistance at rest. Higher = less sliding

We increased these to stop "shaking" after landing:
```xml
<dynamics damping="10.0" friction="5.0"/>
```

---

## Part 2: Real-to-Sim Synchronization

### 2.1 The Problem

Real robot servos use angles (0-180°). Simulation uses:
- **Linear joints**: meters
- **Revolute joints**: radians

### 2.2 The Mapping (real_to_sim_sync.py)

```python
# Constants from real robot code
CENTER_NEUTRAL = 105  # Servo angle when centered
LEG_NEUTRAL = 110
LINEAR_SCALE = 0.046 / 30.0  # 0.046m travel per 30° servo movement

def feedback_callback(self, msg):
    raw_center = msg.data[0]  # e.g., 75, 105, 135
    raw_left = msg.data[1]
    raw_right = msg.data[2]
    
    # CENTER: Convert angle delta to meters
    # 105 -> 0m, 75 -> -0.046m, 135 -> +0.046m
    sim_center = (raw_center - CENTER_NEUTRAL) * LINEAR_SCALE
    
    # LEGS: Convert angle delta to radians
    sim_left = math.radians(raw_left - LEG_NEUTRAL)
    
    # RIGHT LEG: Inverted because servos are mirrored
    sim_right = math.radians(raw_right - LEG_NEUTRAL) * -1.0
```

---

## Part 3: Real Robot Code (realcode.cpp) - Line by Line

### 3.1 Includes & Configuration

```cpp
#include <micro_ros_arduino.h>      // micro-ROS for ESP32
#include <Adafruit_PWMServoDriver.h> // PCA9685 servo controller
#include <WiFi.h>                    // ESP32 WiFi

#define CENTER_NEUTRAL  105  // Servo position when centered
#define MOTOR_NEUTRAL   110  // Leg neutral position
#define FEEDBACK_RATE    50  // Publish every 50ms (20Hz)
```

### 3.2 micro-ROS Entities

```cpp
rcl_subscription_t twist_subscriber;   // Receives /cmd_vel
rcl_publisher_t feedback_publisher;    // Publishes /servo_feedback
rclc_executor_t executor;              // Processes callbacks
```

**What is an Executor?** It's the micro-ROS event loop. When you call `spin_some()`, it checks for new messages and calls your callbacks.

### 3.3 The State Machine (Connection Management)

```cpp
enum ros_state_t {
  WAITING_AGENT,      // WiFi not connected
  AGENT_AVAILABLE,    // WiFi connected, checking for ROS agent
  AGENT_CONNECTED,    // Full communication active
  AGENT_DISCONNECTED  // Lost connection, cleanup
} state;
```

This prevents the robot from crashing if the connection drops.

### 3.4 Servo Control

```cpp
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int angleToPulse(int angle) {
  // Servos use PWM pulses, not angles
  // 0° = 130 pulse, 180° = 600 pulse
  return map(angle, 0, 180, 130, 600);
}

void setServo(int channel, int angle) {
  pwm.setPWM(channel, 0, angleToPulse(angle));
  current_positions[channel] = angle;  // Track for feedback
}
```

### 3.5 Smooth Movement

```cpp
void smoothServo(int channel, int target_angle, int step_delay) {
  int current = current_positions[channel];
  int step = (target_angle > current) ? 1 : -1;  // Direction
  
  while (current != target_angle) {
    current += step;  // Move 1 degree
    pwm.setPWM(channel, 0, angleToPulse(current));
    current_positions[channel] = current;
    smartDelay(step_delay);  // Wait, but also handle feedback
  }
}
```

### 3.6 The Key: smartDelay()

**Problem**: Regular `delay()` blocks everything - no feedback during movement.

**Solution**: `smartDelay()` waits but also publishes feedback:

```cpp
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    // Check if it's time to publish feedback (every 50ms)
    if (millis() - last_feedback_time >= FEEDBACK_RATE) {
      publishFeedback();
      last_feedback_time = millis();
    }
    delay(1);  // Small yield to prevent watchdog reset
  }
}
```

This is how we get **both** smooth servo movement AND real-time feedback.

### 3.7 Main Loop

```cpp
void loop() {
  switch (state) {
    case WAITING_AGENT:
      // Try to connect WiFi
      set_microros_wifi_transports(...);
      state = AGENT_AVAILABLE;
      break;
      
    case AGENT_AVAILABLE:
      // Check if ROS agent is reachable
      if (rmw_uros_ping_agent(1000, 1) == RMW_RET_OK) {
        create_entities();  // Create publishers/subscribers
        state = AGENT_CONNECTED;
      }
      break;
      
    case AGENT_CONNECTED:
      // Normal operation
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      
      // Publish feedback at 20Hz
      if (millis() - last_feedback_time >= FEEDBACK_RATE) {
        publishFeedback();
        last_feedback_time = millis();
      }
      break;
      
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
```

### 3.8 Callbacks (Receiving Commands)

```cpp
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = ...;
  
  if (msg->linear.x > 0.1) stepForward();
  else if (msg->linear.x < -0.1) stepBackward();
  else if (msg->angular.z > 0.1) turnLeft();
  else if (msg->angular.z < -0.1) turnRight();
  else stopMovement();
}
```

When you publish to `/cmd_vel`, this function is called by the executor.

---

## Part 4: How It All Works Together

```
┌─────────────────────────────────────────────────────────────┐
│                      YOUR COMPUTER                           │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐   │
│  │   Gazebo    │◄────│  Sync Node  │◄────│ micro-ROS   │   │
│  │ (Simulation)│     │  (Python)   │     │   Agent     │   │
│  └─────────────┘     └─────────────┘     └─────────────┘   │
│        ▲                   ▲                    ▲          │
│        │                   │                    │          │
│  /joint_trajectory    /servo_feedback      WiFi/UDP        │
│                                                 │          │
└─────────────────────────────────────────────────│──────────┘
                                                  │
                                                  ▼
                                        ┌─────────────────┐
                                        │      ESP32      │
                                        │  (Real Robot)   │
                                        │                 │
                                        │ publishes       │
                                        │ /servo_feedback │
                                        │ subscribes      │
                                        │ /cmd_vel        │
                                        └─────────────────┘
                                                  │
                                                  ▼
                                        ┌─────────────────┐
                                        │  PCA9685 PWM    │
                                        │  Servo Driver   │
                                        └─────────────────┘
                                                  │
                                                  ▼
                                           3 Servos
```

---

## Quick Reference

| Component | File | Purpose |
|-----------|------|---------|
| Robot Model | `tars_robot.xacro` | Defines physical robot |
| Gazebo Config | `tars_robot.gazebo` | Simulation physics |
| Controllers | `controllers2.yaml` | Joint control config |
| Sync Script | `real_to_sim_sync.py` | Real → Sim data |
| ESP32 Code | `realcode.cpp` | Hardware control |

---

## Commands Cheat Sheet

```bash
# Build
cd ~/Tars_ws && colcon build --packages-select tars_robot_description

# Launch Simulation
ros2 launch tars_robot_description gazebo.launch.py

# Run Sync Node
python3 src/tars_robot_description/tars_robot_description/real_to_sim_sync.py

# Test Movement (without real robot)
python3 src/tars_robot_description/tars_robot_description/test_movement.py

# Monitor Feedback
ros2 topic echo /servo_feedback
```
