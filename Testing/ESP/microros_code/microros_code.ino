/*
 * NEXVA ESP32 MICRO-ROS FIRMWARE
 * --------------------------------
 * Automatic ESP32 restart on Micro-ROS Agent disconnect.
 *
 * ROS 2:
 *   /cmd_vel
 *   /odom
 *   /enco/left
 *   /enco/right
 *   /tf
 *
 * Frames:
 *   odom -> base_footprint
 */

#include <Arduino.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <rmw_microros/rmw_microros.h>


// ============================================================
// MOTOR PINS
// ============================================================

#define LEFT_IN1   26
#define LEFT_IN2   25
#define LEFT_ENA   27

#define RIGHT_IN1  18
#define RIGHT_IN2  19
#define RIGHT_ENB  23


// ============================================================
// ENCODER PINS
// ============================================================

#define RIGHT_ENCODER_A 32
#define RIGHT_ENCODER_B 33

#define LEFT_ENCODER_A  22
#define LEFT_ENCODER_B  21


// ============================================================
// MOTOR PARAMETERS
// ============================================================

#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

#define MAX_PWM 255

#define MAX_LINEAR_SPEED  0.30
#define MAX_ANGULAR_SPEED 2.0


// ============================================================
// ROBOT PARAMETERS
// ============================================================

#define WHEEL_DIAMETER 0.067
#define WHEEL_RADIUS   (WHEEL_DIAMETER / 2.0)

#define WHEEL_BASE 0.245

#define ENCODER_CPR 662.0


// ============================================================
// TIMING
// ============================================================

#define CMD_TIMEOUT_MS       500
#define ODOM_PERIOD_MS        50
#define AGENT_CHECK_PERIOD_MS 500

#define RESTART_DELAY_MS     1000


// ============================================================
// MICRO-ROS OBJECTS
// ============================================================

rcl_node_t node;
rcl_subscription_t cmd_vel_subscriber;

rcl_publisher_t left_encoder_publisher;
rcl_publisher_t right_encoder_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t tf_publisher;

rclc_executor_t executor;
rclc_support_t support;

rcl_allocator_t allocator;

geometry_msgs__msg__Twist cmd_vel_msg;

std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;

nav_msgs__msg__Odometry odom_msg;

tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped tf_transform;


// ============================================================
// STATE
// ============================================================

enum AgentState
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

AgentState agent_state = WAITING_AGENT;


// ============================================================
// ENCODERS
// ============================================================

volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;


// ============================================================
// ODOMETRY
// ============================================================

double x_position = 0.0;
double y_position = 0.0;
double theta_position = 0.0;

long previous_left_count = 0;
long previous_right_count = 0;

unsigned long last_odom_time = 0;


// ============================================================
// COMMAND
// ============================================================

unsigned long last_cmd_time = 0;

float current_linear = 0.0;
float current_angular = 0.0;


// ============================================================
// FLAGS
// ============================================================

bool odom_message_initialized = false;
bool tf_message_initialized = false;


// ============================================================
// ERROR HANDLING
// ============================================================

void error_loop()
{
  stopMotors();

  while (true)
  {
    delay(100);
  }
}


#define RCCHECK(fn)                                      \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if (temp_rc != RCL_RET_OK)                           \
    {                                                    \
      error_loop();                                      \
    }                                                    \
  }


#define RCSOFTCHECK(fn)                                  \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    (void)temp_rc;                                       \
  }


// ============================================================
// MOTOR CONTROL
// ============================================================

void setLeftMotor(int pwm)
{
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0)
  {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_ENA, pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    analogWrite(LEFT_ENA, -pwm);
  }
  else
  {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_ENA, 0);
  }
}


void setRightMotor(int pwm)
{
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0)
  {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_ENB, pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(RIGHT_ENB, -pwm);
  }
  else
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_ENB, 0);
  }
}


void stopMotors()
{
  setLeftMotor(0);
  setRightMotor(0);

  current_linear = 0.0;
  current_angular = 0.0;
}


// ============================================================
// ENCODER INTERRUPTS
// ============================================================

void IRAM_ATTR leftEncoderISR()
{
  int a = digitalRead(LEFT_ENCODER_A);
  int b = digitalRead(LEFT_ENCODER_B);

  if (a == b)
    left_encoder_count++;
  else
    left_encoder_count--;
}


void IRAM_ATTR rightEncoderISR()
{
  int a = digitalRead(RIGHT_ENCODER_A);
  int b = digitalRead(RIGHT_ENCODER_B);

  if (a == b)
    right_encoder_count++;
  else
    right_encoder_count--;
}


// ============================================================
// CMD_VEL CALLBACK
// ============================================================

void cmdVelCallback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  current_linear = constrain(
      msg->linear.x,
      -MAX_LINEAR_SPEED,
      MAX_LINEAR_SPEED);

  current_angular = constrain(
      msg->angular.z,
      -MAX_ANGULAR_SPEED,
      MAX_ANGULAR_SPEED);


  // Differential drive
  double left_velocity =
      current_linear -
      (current_angular * WHEEL_BASE / 2.0);

  double right_velocity =
      current_linear +
      (current_angular * WHEEL_BASE / 2.0);


  // Convert wheel velocity to PWM
  int left_pwm =
      (int)((left_velocity / MAX_LINEAR_SPEED) * 255.0);

  int right_pwm =
      (int)((right_velocity / MAX_LINEAR_SPEED) * 255.0);


  left_pwm = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);


  setLeftMotor(left_pwm);
  setRightMotor(right_pwm);

  last_cmd_time = millis();
}


// ============================================================
// RESET ODOMETRY
// ============================================================

void resetOdometry()
{
  x_position = 0.0;
  y_position = 0.0;
  theta_position = 0.0;

  noInterrupts();

  previous_left_count = left_encoder_count;
  previous_right_count = right_encoder_count;

  interrupts();

  last_odom_time = millis();
}


// ============================================================
// INITIALIZE ODOM MESSAGE
// ============================================================

bool initializeOdomMessage()
{
  if (!nav_msgs__msg__Odometry__init(&odom_msg))
  {
    return false;
  }

  odom_msg.header.frame_id.data =
      (char *)"odom";

  odom_msg.header.frame_id.size =
      strlen("odom");

  odom_msg.header.frame_id.capacity =
      strlen("odom") + 1;


  odom_msg.child_frame_id.data =
      (char *)"base_footprint";

  odom_msg.child_frame_id.size =
      strlen("base_footprint");

  odom_msg.child_frame_id.capacity =
      strlen("base_footprint") + 1;


  odom_message_initialized = true;

  return true;
}


// ============================================================
// INITIALIZE TF MESSAGE
// ============================================================

bool initializeTFMessage()
{
  if (!tf2_msgs__msg__TFMessage__init(&tf_msg))
  {
    return false;
  }

  if (!geometry_msgs__msg__TransformStamped__init(&tf_transform))
  {
    return false;
  }

  tf_transform.header.frame_id.data =
      (char *)"odom";

  tf_transform.header.frame_id.size =
      strlen("odom");

  tf_transform.header.frame_id.capacity =
      strlen("odom") + 1;


  tf_transform.child_frame_id.data =
      (char *)"base_footprint";

  tf_transform.child_frame_id.size =
      strlen("base_footprint");

  tf_transform.child_frame_id.capacity =
      strlen("base_footprint") + 1;


  tf_msg.transforms.data = &tf_transform;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.capacity = 1;

  tf_message_initialized = true;

  return true;
}


// ============================================================
// CREATE MICRO-ROS ENTITIES
// ============================================================

bool createEntities()
{
  allocator = rcl_get_default_allocator();


  // ----------------------------------------------------------
  // Support
  // ----------------------------------------------------------

  if (rclc_support_init(
          &support,
          0,
          NULL,
          &allocator) != RCL_RET_OK)
  {
    return false;
  }


  // ----------------------------------------------------------
  // Node
  // ----------------------------------------------------------

  if (rclc_node_init_default(
          &node,
          "nexva_esp32",
          "",
          &support) != RCL_RET_OK)
  {
    return false;
  }


  // ----------------------------------------------------------
  // CMD_VEL Subscriber
  // ----------------------------------------------------------

  if (rclc_subscription_init_default(
          &cmd_vel_subscriber,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              geometry_msgs,
              msg,
              Twist),
          "/cmd_vel") != RCL_RET_OK)
  {
    return false;
  }


  // ----------------------------------------------------------
  // LEFT ENCODER
  // ----------------------------------------------------------

  if (rclc_publisher_init_default(
          &left_encoder_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              std_msgs,
              msg,
              Int32),
          "/enco/left") != RCL_RET_OK)
  {
    return false;
  }


  // ----------------------------------------------------------
  // RIGHT ENCODER
  // ----------------------------------------------------------

  if (rclc_publisher_init_default(
          &right_encoder_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              std_msgs,
              msg,
              Int32),
          "/enco/right") != RCL_RET_OK)
  {
    return false;
  }


  // ----------------------------------------------------------
  // ODOM
  // ----------------------------------------------------------

  if (rclc_publisher_init_default(
          &odom_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              nav_msgs,
              msg,
              Odometry),
          "/odom") != RCL_RET_OK)
  {
    return false;
  }


  // ----------------------------------------------------------
  // TF
  // ----------------------------------------------------------

  if (rclc_publisher_init_default(
          &tf_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              tf2_msgs,
              msg,
              TFMessage),
          "/tf") != RCL_RET_OK)
  {
    return false;
  }


  // ----------------------------------------------------------
  // Messages
  // ----------------------------------------------------------

  geometry_msgs__msg__Twist__init(&cmd_vel_msg);

  std_msgs__msg__Int32__init(&left_encoder_msg);
  std_msgs__msg__Int32__init(&right_encoder_msg);


  if (!initializeOdomMessage())
    return false;

  if (!initializeTFMessage())
    return false;


  // ----------------------------------------------------------
  // Executor
  // ----------------------------------------------------------

  if (rclc_executor_init(
          &executor,
          &support.context,
          1,
          &allocator) != RCL_RET_OK)
  {
    return false;
  }


  if (rclc_executor_add_subscription(
          &executor,
          &cmd_vel_subscriber,
          &cmd_vel_msg,
          &cmdVelCallback,
          ON_NEW_DATA) != RCL_RET_OK)
  {
    return false;
  }


  resetOdometry();

  last_cmd_time = millis();

  Serial.println();
  Serial.println("--------------------------------");
  Serial.println("MICRO-ROS ENTITIES CREATED");
  Serial.println("Node: /nexva_esp32");
  Serial.println("Session: NEW");
  Serial.println("--------------------------------");
  Serial.println();

  return true;
}


// ============================================================
// DESTROY MICRO-ROS ENTITIES
// ============================================================

void destroyEntities()
{
  Serial.println("Destroying Micro-ROS entities...");

  stopMotors();


  if (executor.context != NULL)
  {
    rclc_executor_fini(&executor);
  }


  if (rcl_publisher_fini(
          &left_encoder_publisher,
          &node) != RCL_RET_OK)
  {
  }

  if (rcl_publisher_fini(
          &right_encoder_publisher,
          &node) != RCL_RET_OK)
  {
  }

  if (rcl_publisher_fini(
          &odom_publisher,
          &node) != RCL_RET_OK)
  {
  }

  if (rcl_publisher_fini(
          &tf_publisher,
          &node) != RCL_RET_OK)
  {
  }


  if (rcl_subscription_fini(
          &cmd_vel_subscriber,
          &node) != RCL_RET_OK)
  {
  }


  if (rcl_node_fini(&node) != RCL_RET_OK)
  {
  }


  if (rclc_support_fini(&support) != RCL_RET_OK)
  {
  }


  odom_message_initialized = false;
  tf_message_initialized = false;

  Serial.println("Micro-ROS entities destroyed.");
}


// ============================================================
// CHECK AGENT
// ============================================================

bool agentAvailable()
{
  rcl_ret_t rc =
      rmw_uros_ping_agent(100, 1);

  return rc == RCL_RET_OK;
}


// ============================================================
// PUBLISH ENCODERS
// ============================================================

void publishEncoders()
{
  noInterrupts();

  long left_count = left_encoder_count;
  long right_count = right_encoder_count;

  interrupts();


  left_encoder_msg.data = (int32_t)left_count;
  right_encoder_msg.data = (int32_t)right_count;


  RCSOFTCHECK(
      rcl_publish(
          &left_encoder_publisher,
          &left_encoder_msg,
          NULL));


  RCSOFTCHECK(
      rcl_publish(
          &right_encoder_publisher,
          &right_encoder_msg,
          NULL));
}


// ============================================================
// PUBLISH ODOMETRY
// ============================================================

void publishOdometry()
{
  unsigned long now = millis();

  double dt =
      (now - last_odom_time) / 1000.0;

  if (dt <= 0.0)
    return;

  last_odom_time = now;


  noInterrupts();

  long left_count = left_encoder_count;
  long right_count = right_encoder_count;

  interrupts();


  long delta_left =
      left_count - previous_left_count;

  long delta_right =
      right_count - previous_right_count;


  previous_left_count = left_count;
  previous_right_count = right_count;


  double left_distance =
      (delta_left / ENCODER_CPR) *
      (2.0 * PI * WHEEL_RADIUS);

  double right_distance =
      (delta_right / ENCODER_CPR) *
      (2.0 * PI * WHEEL_RADIUS);


  double distance =
      (left_distance + right_distance) / 2.0;

  double delta_theta =
      (right_distance - left_distance) /
      WHEEL_BASE;


  theta_position += delta_theta;

  x_position +=
      distance * cos(theta_position);

  y_position +=
      distance * sin(theta_position);


  double linear_velocity =
      distance / dt;

  double angular_velocity =
      delta_theta / dt;


  int64_t stamp =
      rmw_uros_epoch_nanos();


  odom_msg.header.stamp.sec =
      stamp / 1000000000LL;

  odom_msg.header.stamp.nanosec =
      stamp % 1000000000LL;


  odom_msg.pose.pose.position.x =
      x_position;

  odom_msg.pose.pose.position.y =
      y_position;

  odom_msg.pose.pose.position.z =
      0.0;


  odom_msg.pose.pose.orientation.x =
      0.0;

  odom_msg.pose.pose.orientation.y =
      0.0;

  odom_msg.pose.pose.orientation.z =
      sin(theta_position / 2.0);

  odom_msg.pose.pose.orientation.w =
      cos(theta_position / 2.0);


  odom_msg.twist.twist.linear.x =
      linear_velocity;

  odom_msg.twist.twist.angular.z =
      angular_velocity;


  RCSOFTCHECK(
      rcl_publish(
          &odom_publisher,
          &odom_msg,
          NULL));


  // ----------------------------------------------------------
  // TF
  // ----------------------------------------------------------

  tf_transform.header.stamp.sec =
      odom_msg.header.stamp.sec;

  tf_transform.header.stamp.nanosec =
      odom_msg.header.stamp.nanosec;


  tf_transform.transform.translation.x =
      x_position;

  tf_transform.transform.translation.y =
      y_position;

  tf_transform.transform.translation.z =
      0.0;


  tf_transform.transform.rotation.x =
      0.0;

  tf_transform.transform.rotation.y =
      0.0;

  tf_transform.transform.rotation.z =
      sin(theta_position / 2.0);

  tf_transform.transform.rotation.w =
      cos(theta_position / 2.0);


  RCSOFTCHECK(
      rcl_publish(
          &tf_publisher,
          &tf_msg,
          NULL));
}


// ============================================================
// AUTOMATIC ESP32 RESTART
// ============================================================

void restartESP32()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("MICRO-ROS AGENT DISCONNECTED");
  Serial.println("STOPPING MOTORS");
  Serial.println("DESTROYING SESSION");
  Serial.println("RESTARTING ESP32...");
  Serial.println("================================");
  Serial.flush();


  stopMotors();

  destroyEntities();

  delay(RESTART_DELAY_MS);


  // ----------------------------------------------------------
  // HARD SOFTWARE REBOOT
  // ----------------------------------------------------------

  ESP.restart();


  // Should never reach here
  while (true)
  {
    stopMotors();
    delay(100);
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(2000);


  // ==========================================================
  // MOTOR SETUP
  // ==========================================================

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_ENA, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);


  analogWriteFrequency(LEFT_ENA, PWM_FREQ);
  analogWriteFrequency(RIGHT_ENB, PWM_FREQ);

  analogWriteResolution(LEFT_ENA, PWM_RESOLUTION);
  analogWriteResolution(RIGHT_ENB, PWM_RESOLUTION);


  stopMotors();


  // ==========================================================
  // ENCODER SETUP
  // ==========================================================

  pinMode(
      LEFT_ENCODER_A,
      INPUT_PULLUP);

  pinMode(
      LEFT_ENCODER_B,
      INPUT_PULLUP);

  pinMode(
      RIGHT_ENCODER_A,
      INPUT_PULLUP);

  pinMode(
      RIGHT_ENCODER_B,
      INPUT_PULLUP);


  attachInterrupt(
      digitalPinToInterrupt(LEFT_ENCODER_A),
      leftEncoderISR,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(RIGHT_ENCODER_A),
      rightEncoderISR,
      CHANGE);


  // ==========================================================
  // MICRO-ROS TRANSPORT
  // ==========================================================

  Serial.println();
  Serial.println("================================");
  Serial.println("NEXVA ESP32 STARTING");
  Serial.println("================================");

  Serial.println(
      "Initializing Micro-ROS transport...");


  set_microros_transports();


  delay(2000);


  Serial.println(
      "Micro-ROS transport initialized.");

  Serial.println(
      "Waiting for Micro-ROS Agent...");


  agent_state = WAITING_AGENT;
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  static unsigned long last_agent_check = 0;
  static unsigned long last_odom_publish = 0;


  // ==========================================================
  // WAIT FOR AGENT
  // ==========================================================

  if (agent_state == WAITING_AGENT)
  {
    if (millis() - last_agent_check >=
        AGENT_CHECK_PERIOD_MS)
    {
      last_agent_check = millis();


      if (agentAvailable())
      {
        Serial.println();
        Serial.println(
            "Micro-ROS Agent detected.");

        agent_state = AGENT_AVAILABLE;
      }
    }


    stopMotors();

    delay(10);

    return;
  }


  // ==========================================================
  // CREATE NEW SESSION / ENTITIES
  // ==========================================================

  if (agent_state == AGENT_AVAILABLE)
  {
    Serial.println(
        "Creating NEW Micro-ROS session...");


    if (createEntities())
    {
      agent_state = AGENT_CONNECTED;

      Serial.println(
          "NEW Micro-ROS session CONNECTED.");
    }
    else
    {
      Serial.println(
          "Entity creation failed.");

      delay(500);

      // Reboot to guarantee completely clean state
      ESP.restart();
    }

    return;
  }


  // ==========================================================
  // CONNECTED
  // ==========================================================

  if (agent_state == AGENT_CONNECTED)
  {
    // --------------------------------------------------------
    // Execute ROS callbacks
    // --------------------------------------------------------

    RCSOFTCHECK(
        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(5)));


    // --------------------------------------------------------
    // Command timeout
    // --------------------------------------------------------

    if (millis() - last_cmd_time >
        CMD_TIMEOUT_MS)
    {
      stopMotors();
    }


    // --------------------------------------------------------
    // Publish encoders + odom
    // --------------------------------------------------------

    if (millis() - last_odom_publish >=
        ODOM_PERIOD_MS)
    {
      last_odom_publish = millis();

      publishEncoders();
      publishOdometry();
    }


    // --------------------------------------------------------
    // Check Micro-ROS Agent
    // --------------------------------------------------------

    if (millis() - last_agent_check >=
        AGENT_CHECK_PERIOD_MS)
    {
      last_agent_check = millis();


      if (!agentAvailable())
      {
        agent_state =
            AGENT_DISCONNECTED;
      }
    }


    delay(2);

    return;
  }


  // ==========================================================
  // AGENT DISCONNECTED
  // ==========================================================

  if (agent_state == AGENT_DISCONNECTED)
  {
    restartESP32();

    return;
  }
}