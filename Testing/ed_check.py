#include <Arduino.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32.h>

#include <rosidl_runtime_c/string_functions.h>

// ============================================================
// MOTOR PINS
// ============================================================

// Left motor
#define LEFT_IN1 25
#define LEFT_IN2 26
#define LEFT_ENA 27

// Right motor
#define RIGHT_IN1 18
#define RIGHT_IN2 19
#define RIGHT_ENB 23

// ============================================================
// ENCODER PINS
// ============================================================

#define LEFT_ENCODER_A 32
#define LEFT_ENCODER_B 34

#define RIGHT_ENCODER_A 35
#define RIGHT_ENCODER_B 39

// ============================================================
// ROBOT PARAMETERS
// ============================================================

// Wheel diameter = 67 mm
#define WHEEL_DIAMETER 0.067

// Wheel separation = 245 mm
#define WHEEL_BASE 0.245

// CHANGE THIS TO YOUR ACTUAL ENCODER VALUE
// Example: 600, 1000, 2048, etc.
#define TICKS_PER_REV 600.0

// Maximum PWM
#define PWM_MAX 255

// Minimum PWM used to overcome motor dead zone
#define MIN_PWM 60

// ============================================================
// PWM
// ============================================================

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// ============================================================
// ENCODER VARIABLES
// ============================================================

volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// ============================================================
// CMD_VEL VARIABLES
// ============================================================

float target_linear_x = 0.0;
float target_angular_z = 0.0;

// ============================================================
// ODOMETRY
// ============================================================

float x = 0.0;
float y = 0.0;
float theta = 0.0;

long previous_left_ticks = 0;
long previous_right_ticks = 0;

// ============================================================
// MICRO ROS OBJECTS
// ============================================================

rcl_allocator_t allocator;

rclc_support_t support;

rcl_node_t node;

rclc_executor_t executor;

rcl_subscription_t cmd_vel_subscriber;

rcl_publisher_t odom_publisher;

rcl_publisher_t left_encoder_publisher;

rcl_publisher_t right_encoder_publisher;

// ============================================================
// ROS MESSAGES
// ============================================================

geometry_msgs__msg__Twist cmd_vel_msg;

nav_msgs__msg__Odometry odom_msg;

std_msgs__msg__Int32 left_encoder_msg;

std_msgs__msg__Int32 right_encoder_msg;

// ============================================================
// TIMING
// ============================================================

unsigned long last_odom_time = 0;
unsigned long last_publish_time = 0;

// ============================================================
// ENCODER INTERRUPTS
// ============================================================

void IRAM_ATTR leftEncoderISR()
{
  int b = digitalRead(LEFT_ENCODER_B);

  if (b == HIGH)
  {
    left_encoder_ticks++;
  }
  else
  {
    left_encoder_ticks--;
  }
}


void IRAM_ATTR rightEncoderISR()
{
  int b = digitalRead(RIGHT_ENCODER_B);

  if (b == HIGH)
  {
    right_encoder_ticks++;
  }
  else
  {
    right_encoder_ticks--;
  }
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void setLeftMotor(int pwm)
{
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  if (pwm > 0)
  {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);

    if (pwm < MIN_PWM)
      pwm = MIN_PWM;

    ledcWrite(LEFT_ENA, pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);

    pwm = abs(pwm);

    if (pwm < MIN_PWM)
      pwm = MIN_PWM;

    ledcWrite(LEFT_ENA, pwm);
  }
  else
  {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);

    ledcWrite(LEFT_ENA, 0);
  }
}


void setRightMotor(int pwm)
{
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  if (pwm > 0)
  {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);

    if (pwm < MIN_PWM)
      pwm = MIN_PWM;

    ledcWrite(RIGHT_ENB, pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);

    pwm = abs(pwm);

    if (pwm < MIN_PWM)
      pwm = MIN_PWM;

    ledcWrite(RIGHT_ENB, pwm);
  }
  else
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);

    ledcWrite(RIGHT_ENB, 0);
  }
}


// ============================================================
// CMD_VEL CALLBACK
// ============================================================

void cmdVelCallback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  target_linear_x = msg->linear.x;
  target_angular_z = msg->angular.z;

  // ==========================================================
  // DIFFERENTIAL DRIVE
  //
  // left_velocity  = v - (w * wheel_base / 2)
  // right_velocity = v + (w * wheel_base / 2)
  // ==========================================================

  float left_velocity =
      target_linear_x -
      (target_angular_z * WHEEL_BASE / 2.0);

  float right_velocity =
      target_linear_x +
      (target_angular_z * WHEEL_BASE / 2.0);

  // Convert velocity to PWM
  //
  // This is a SIMPLE open-loop mapping.
  // Later we should replace this with PID control.

  float max_speed = 0.5;  // m/s at PWM 255

  int left_pwm =
      (int)((left_velocity / max_speed) * PWM_MAX);

  int right_pwm =
      (int)((right_velocity / max_speed) * PWM_MAX);

  left_pwm = constrain(left_pwm, -PWM_MAX, PWM_MAX);
  right_pwm = constrain(right_pwm, -PWM_MAX, PWM_MAX);

  setLeftMotor(left_pwm);
  setRightMotor(right_pwm);
}


// ============================================================
// UPDATE ODOMETRY
// ============================================================

void updateOdometry()
{
  unsigned long current_time = millis();

  float dt =
      (current_time - last_odom_time) / 1000.0;

  if (dt <= 0.0)
    return;

  last_odom_time = current_time;

  // Safely copy encoder values
  noInterrupts();

  long left_ticks = left_encoder_ticks;
  long right_ticks = right_encoder_ticks;

  interrupts();

  // ----------------------------------------------------------
  // Tick difference
  // ----------------------------------------------------------

  long delta_left =
      left_ticks - previous_left_ticks;

  long delta_right =
      right_ticks - previous_right_ticks;

  previous_left_ticks = left_ticks;
  previous_right_ticks = right_ticks;

  // ----------------------------------------------------------
  // Convert ticks -> wheel distance
  // ----------------------------------------------------------

  float wheel_circumference =
      PI * WHEEL_DIAMETER;

  float left_distance =
      ((float)delta_left / TICKS_PER_REV) *
      wheel_circumference;

  float right_distance =
      ((float)delta_right / TICKS_PER_REV) *
      wheel_circumference;

  // ----------------------------------------------------------
  // Differential drive odometry
  // ----------------------------------------------------------

  float distance =
      (left_distance + right_distance) / 2.0;

  float delta_theta =
      (right_distance - left_distance) /
      WHEEL_BASE;

  // Midpoint integration
  float theta_mid =
      theta + (delta_theta / 2.0);

  x += distance * cos(theta_mid);
  y += distance * sin(theta_mid);

  theta += delta_theta;

  // Keep theta within -PI to PI
  if (theta > PI)
    theta -= 2.0 * PI;

  if (theta < -PI)
    theta += 2.0 * PI;

  // ----------------------------------------------------------
  // Calculate velocity
  // ----------------------------------------------------------

  float linear_velocity =
      distance / dt;

  float angular_velocity =
      delta_theta / dt;

  // ----------------------------------------------------------
  // Fill odometry message
  // ----------------------------------------------------------

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  // Convert yaw -> quaternion

  float half_theta = theta / 2.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(half_theta);
  odom_msg.pose.pose.orientation.w = cos(half_theta);

  odom_msg.twist.twist.linear.x =
      linear_velocity;

  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;

  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;

  odom_msg.twist.twist.angular.z =
      angular_velocity;
}


// ============================================================
// PUBLISH ENCODERS
// ============================================================

void publishEncoders()
{
  noInterrupts();

  long left_ticks = left_encoder_ticks;
  long right_ticks = right_encoder_ticks;

  interrupts();

  left_encoder_msg.data = left_ticks;
  right_encoder_msg.data = right_ticks;

  rcl_publish(
      &left_encoder_publisher,
      &left_encoder_msg,
      NULL);

  rcl_publish(
      &right_encoder_publisher,
      &right_encoder_msg,
      NULL);
}


// ============================================================
// PUBLISH ODOM
// ============================================================

void publishOdometry()
{
  int64_t time_ns =
      rmw_uros_epoch_nanos();

  odom_msg.header.stamp.sec =
      time_ns / 1000000000LL;

  odom_msg.header.stamp.nanosec =
      time_ns % 1000000000LL;

  rcl_publish(
      &odom_publisher,
      &odom_msg,
      NULL);
}


// ============================================================
// MICRO ROS ERROR HANDLING
// ============================================================

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }

#define RCSOFTCHECK(fn) \
  {                     \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc;          \
  }


// ============================================================
// ERROR LOOP
// ============================================================

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  // ----------------------------------------------------------
  // Motor pins
  // ----------------------------------------------------------

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // ESP32 PWM
  ledcAttach(LEFT_ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_ENB, PWM_FREQ, PWM_RESOLUTION);

  setLeftMotor(0);
  setRightMotor(0);

  // ----------------------------------------------------------
  // Encoder pins
  // ----------------------------------------------------------

  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);

  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  attachInterrupt(
      digitalPinToInterrupt(LEFT_ENCODER_A),
      leftEncoderISR,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(RIGHT_ENCODER_A),
      rightEncoderISR,
      RISING);

  // ----------------------------------------------------------
  // micro-ROS transport
  //
  // USB Serial transport
  // ----------------------------------------------------------

  set_microros_serial_transports(Serial);

  delay(2000);

  // ----------------------------------------------------------
  // ROS initialization
  // ----------------------------------------------------------

  allocator =
      rcl_get_default_allocator();

  RCCHECK(
      rclc_support_init(
          &support,
          0,
          NULL,
          &allocator));

  // ----------------------------------------------------------
  // Create node
  // ----------------------------------------------------------

  RCCHECK(
      rclc_node_init_default(
          &node,
          "nexva_esp32",
          "",
          &support));

  // ----------------------------------------------------------
  // /cmd_vel subscriber
  // ----------------------------------------------------------

  RCCHECK(
      rclc_subscription_init_default(
          &cmd_vel_subscriber,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              geometry_msgs,
              msg,
              Twist),
          "/cmd_vel"));

  // ----------------------------------------------------------
  // /odom publisher
  // ----------------------------------------------------------

  RCCHECK(
      rclc_publisher_init_default(
          &odom_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              nav_msgs,
              msg,
              Odometry),
          "/odom"));

  // ----------------------------------------------------------
  // /left_enco publisher
  // ----------------------------------------------------------

  RCCHECK(
      rclc_publisher_init_default(
          &left_encoder_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              std_msgs,
              msg,
              Int32),
          "/left_enco"));

  // ----------------------------------------------------------
  // /right_enco publisher
  // ----------------------------------------------------------

  RCCHECK(
      rclc_publisher_init_default(
          &right_encoder_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              std_msgs,
              msg,
              Int32),
          "/right_enco"));

  // ----------------------------------------------------------
  // Initialize message frame IDs
  // ----------------------------------------------------------

  rosidl_runtime_c__String__assign(
      &odom_msg.header.frame_id,
      "odom");

  rosidl_runtime_c__String__assign(
      &odom_msg.child_frame_id,
      "base_link");

  // ----------------------------------------------------------
  // Initialize executor
  // ----------------------------------------------------------

  RCCHECK(
      rclc_executor_init(
          &executor,
          &support.context,
          1,
          &allocator));

  RCCHECK(
      rclc_executor_add_subscription(
          &executor,
          &cmd_vel_subscriber,
          &cmd_vel_msg,
          &cmdVelCallback,
          ON_NEW_DATA));

  // ----------------------------------------------------------
  // Start timing
  // ----------------------------------------------------------

  last_odom_time = millis();
  last_publish_time = millis();

  digitalWrite(LED_BUILTIN, HIGH);
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  // Process ROS messages
  RCSOFTCHECK(
      rclc_executor_spin_some(
          &executor,
          RCL_MS_TO_NS(5)));

  // ----------------------------------------------------------
  // Update odometry
  // ----------------------------------------------------------

  updateOdometry();

  // ----------------------------------------------------------
  // Publish every 20 ms = 50 Hz
  // ----------------------------------------------------------

  if (millis() - last_publish_time >= 20)
  {
    last_publish_time = millis();

    publishEncoders();

    publishOdometry();
  }

  delay(2);
}   99
