# TARS ESP32 MicroROS Controller

This folder contains the firmware for the ESP32-S3 microcontroller that controls the real TARS robot.

## Hardware Requirements

- **ESP32-S3 DevKit** (or compatible)
- **PCA9685 PWM Driver** (I2C address 0x40)
- **3x Servo Motors**:
  - Channel 0: Center lift servo
  - Channel 1: Left leg drive servo
  - Channel 2: Right leg drive servo
- **5V Power Supply** for servos

## Wiring

| ESP32 Pin | PCA9685 Pin |
|-----------|-------------|
| GPIO 21   | SDA         |
| GPIO 19   | SCL         |
| 3.3V      | VCC         |
| GND       | GND         |

## Setup

1. Install PlatformIO or Arduino IDE
2. Install required libraries:
   - `micro_ros_arduino`
   - `Adafruit_PWMServoDriver`
   - `Adafruit_NeoPixel`

3. Update WiFi credentials in `tars_controller.cpp`:
   ```cpp
   char wifi_ssid[] = "YOUR_WIFI_SSID";
   char wifi_password[] = "YOUR_WIFI_PASSWORD";
   char agent_ip[] = "YOUR_PC_IP_ADDRESS";
   ```

4. Flash to ESP32

## Usage

1. Start MicroROS agent on your PC:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

2. Power on ESP32 - it will connect automatically

3. Control via ROS2:
   ```bash
   # Move forward
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
   
   # Turn left
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"
   ```

## Servo Tuning

Adjust these values at the top of `tars_controller.cpp`:

```cpp
#define CENTER_NEUTRAL  105
#define CENTER_UP      135
#define CENTER_DOWN     75

#define MOTOR_NEUTRAL   110
#define MOTOR_FORWARD_L 125
#define MOTOR_FORWARD_R  95
```
