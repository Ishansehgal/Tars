#include <Adafruit_NeoPixel.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>

// ==================== EASY TO CHANGE MOTOR VALUES ====================
#define CENTER_NEUTRAL  105
#define CENTER_UP      135
#define CENTER_DOWN     75

#define MOTOR_NEUTRAL   110
#define MOTOR_FORWARD_L 125
#define MOTOR_FORWARD_R  95
#define MOTOR_BACK_L     95
#define MOTOR_BACK_R    125
#define MOTOR_TURN_L     80
#define MOTOR_TURN_R    140

#define UP_DELAY        400
#define STEP_DELAY       20
#define TURN_HOLD_TIME  500
#define FEEDBACK_RATE    50   // Publish feedback every 50ms (20Hz)
// =====================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// WiFi credentials
char wifi_ssid[] = "ACT-ai_123744637760";
char wifi_password[] = "59959412";
char agent_ip[] = "192.168.1.37";
const int agent_port = 8888;

// Connection state tracking
enum ros_state_t {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Micro-ROS entities
rcl_subscription_t twist_subscriber;
rcl_subscription_t servo_subscriber;
rcl_publisher_t feedback_publisher;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32MultiArray servo_msg;
std_msgs__msg__Int32MultiArray feedback_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long last_ping = 0;
bool micro_ros_init_successful = false;
unsigned long last_feedback_time = 0;

// Current servo positions for tracking
int current_positions[3] = {CENTER_NEUTRAL, MOTOR_NEUTRAL, MOTOR_NEUTRAL};

// Convert angle to PWM pulse
int angleToPulse(int angle) {
  return map(angle, 0, 180, 130, 600);
}

// Publish feedback - called from loop() and smartDelay()
void publishFeedback() {
  if (micro_ros_init_successful) {
    feedback_msg.data.data[0] = current_positions[0];
    feedback_msg.data.data[1] = current_positions[1];
    feedback_msg.data.data[2] = current_positions[2];
    rcl_publish(&feedback_publisher, &feedback_msg, NULL);
  }
}

// Smart delay that publishes feedback during wait
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    // Publish feedback at 20Hz during delay
    if (millis() - last_feedback_time >= FEEDBACK_RATE) {
      publishFeedback();
      last_feedback_time = millis();
    }
    delay(1);  // Small yield
  }
}

// Set servo position immediately - NO feedback call here
void setServo(int channel, int angle) {
  if (angle >= 0 && angle <= 180 && channel >= 0 && channel <= 2) {
    pwm.setPWM(channel, 0, angleToPulse(angle));
    current_positions[channel] = angle;
  }
}

// Smooth servo movement with feedback
void smoothServo(int channel, int target_angle, int step_delay = STEP_DELAY) {
  if (target_angle < 0 || target_angle > 180 || channel < 0 || channel > 2) return;
  
  int current = current_positions[channel];
  int step = (target_angle > current) ? 1 : -1;
  
  while (current != target_angle) {
    current += step;
    pwm.setPWM(channel, 0, angleToPulse(current));
    current_positions[channel] = current;
    smartDelay(step_delay);  // Use smartDelay instead of delay
  }
}

// Move CENTER motor UP with delay
void moveCenterUp(int target_angle) {
  smoothServo(0, target_angle, STEP_DELAY);
  smartDelay(UP_DELAY);  // Use smartDelay
}

// Move CENTER motor DOWN
void moveCenterDown(int target_angle) {
  smoothServo(0, target_angle, STEP_DELAY);
}

// Move BOTH left and right motors TOGETHER
void moveBothArmsTogether(int left_angle, int right_angle) {
  int max_steps = max(abs(left_angle - current_positions[1]), 
                     abs(right_angle - current_positions[2]));
  
  for (int i = 0; i <= max_steps; i++) {
    int left_pos = map(i, 0, max_steps, current_positions[1], left_angle);
    int right_pos = map(i, 0, max_steps, current_positions[2], right_angle);
    
    pwm.setPWM(1, 0, angleToPulse(left_pos));
    pwm.setPWM(2, 0, angleToPulse(right_pos));
    current_positions[1] = left_pos;
    current_positions[2] = right_pos;
    
    smartDelay(STEP_DELAY);  // Use smartDelay
  }
}

// WiFi functions
bool connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(wifi_ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);
  
  unsigned long wifi_timeout = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifi_timeout) < 15000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("");
    Serial.println("WiFi connection failed!");
    return false;
  }
}

bool checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    return connectWiFi();
  }
  return true;
}

// Destroy micro-ROS entities
void destroy_entities() {
  if (micro_ros_init_successful) {
    rcl_publisher_fini(&feedback_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_subscription_fini(&servo_subscriber, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    micro_ros_init_successful = false;
  }
}

// Create micro-ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "tars_controller", "", &support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(&twist_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(&servo_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "servo_control") != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(&feedback_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "servo_feedback") != RCL_RET_OK) return false;

  feedback_msg.data.data = (int32_t*) malloc(3 * sizeof(int32_t));
  feedback_msg.data.size = 3;
  feedback_msg.data.capacity = 3;

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  micro_ros_init_successful = true;
  return true;
}

// Movement functions
void stepForward() {
  if (!micro_ros_init_successful) return;
  moveCenterUp(CENTER_DOWN);
  moveBothArmsTogether(MOTOR_FORWARD_L, MOTOR_FORWARD_R);
  moveCenterUp(CENTER_UP);
  moveBothArmsTogether(MOTOR_NEUTRAL, MOTOR_NEUTRAL);
  moveCenterDown(CENTER_NEUTRAL);
}

void stepBackward() {
  if (!micro_ros_init_successful) return;
  moveCenterUp(CENTER_DOWN);
  moveBothArmsTogether(MOTOR_BACK_L, MOTOR_BACK_R);
  moveCenterDown(CENTER_NEUTRAL);
  moveBothArmsTogether(MOTOR_NEUTRAL, MOTOR_NEUTRAL);
}

void turnRight() {
  if (!micro_ros_init_successful) return;
  moveCenterDown(CENTER_DOWN);
  moveBothArmsTogether(MOTOR_TURN_R, MOTOR_TURN_R);
  delay(TURN_HOLD_TIME);
  smoothServo(0, CENTER_NEUTRAL, STEP_DELAY);
  moveBothArmsTogether(MOTOR_NEUTRAL, MOTOR_NEUTRAL);
}

void turnLeft() {
  if (!micro_ros_init_successful) return;
  moveCenterDown(CENTER_DOWN);
  moveBothArmsTogether(MOTOR_TURN_L, MOTOR_TURN_L);
  delay(TURN_HOLD_TIME);
  smoothServo(0, CENTER_NEUTRAL, STEP_DELAY);
  moveBothArmsTogether(MOTOR_NEUTRAL, MOTOR_NEUTRAL);
}

void stopMovement() {
  moveCenterDown(CENTER_NEUTRAL);
  moveBothArmsTogether(MOTOR_NEUTRAL, MOTOR_NEUTRAL);
}

// Servo control callback
void servo_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  if (msg->data.size >= 2) {
    int servo_channel = msg->data.data[0];
    int angle = msg->data.data[1];
    if (servo_channel >= 0 && servo_channel <= 2 && angle >= 0 && angle <= 180) {
      setServo(servo_channel, angle);
    }
  }
}

// Twist callback
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  
  if (linear_x > 0.1) stepForward();
  else if (linear_x < -0.1) stepBackward();
  else if (angular_z > 0.1) turnLeft();
  else if (angular_z < -0.1) turnRight();
  else stopMovement();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 19);
  
  pwm.begin();
  pwm.setPWMFreq(60);

  setServo(0, CENTER_NEUTRAL);
  delay(500);
  setServo(1, MOTOR_NEUTRAL);  
  delay(500);
  setServo(2, MOTOR_NEUTRAL);
  delay(1000);
  
  state = WAITING_AGENT;
}

void loop() {
  bool wifi_connected = checkWiFi();
  
  switch (state) {
    case WAITING_AGENT:
      if (wifi_connected) {
        set_microros_wifi_transports(wifi_ssid, wifi_password, agent_ip, agent_port);
        state = AGENT_AVAILABLE;
      }
      break;
      
    case AGENT_AVAILABLE:
      if (rmw_uros_ping_agent(1000, 1) == RMW_RET_OK) {
        if (create_entities()) {
          state = AGENT_CONNECTED;
          last_ping = millis();
          last_feedback_time = millis();
        } else {
          delay(2000);
        }
      } else {
        delay(1000);
      }
      break;
      
    case AGENT_CONNECTED:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        last_ping = millis();
        
        // Publish feedback at 20Hz - ONLY HERE
        if (millis() - last_feedback_time >= FEEDBACK_RATE) {
          publishFeedback();
          last_feedback_time = millis();
        }
      } else {
        state = AGENT_DISCONNECTED;
      }
      
      if (millis() - last_ping > 5000) {
        state = AGENT_DISCONNECTED;
      }
      break;
      
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      delay(2000);
      break;
  }
  
  delay(10);
}
