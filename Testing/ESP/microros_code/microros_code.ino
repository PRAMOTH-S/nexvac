

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/vector3.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

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
// Full authority for the controller. The speed limit is NOT set here any more
// - see MAX_LINEAR_SPEED. Capping PWM to limit speed is what made the robot
// unusable on 28 Sep: PWM is a duty cycle, and how much speed a given duty
// cycle buys depends on the battery.
#define MAX_PWM 255

// The real speed limits, enforced on the velocity TARGET. Because the loop
// below closes on measured wheel speed, these are honoured whatever the pack
// voltage is doing.
#define MAX_LINEAR_SPEED  0.30
#define MAX_ANGULAR_SPEED 2.0

// ---------- closed-loop wheel velocity control ----------
//
// Open loop was the root cause of every speed problem this robot has had. The
// PWM->speed curve moved between 23 Sep (0.00341 * (PWM-147)) and 28 Sep
// (0.0065 * (PWM-108)) with no code change at all - battery state of charge.
// A PI loop on measured wheel speed makes that irrelevant.
//
// Feedforward gets the wheel moving immediately; the integrator absorbs
// whatever the feedforward got wrong, including pack voltage drift.
#define CONTROL_HZ        50.0
#define FF_BREAKAWAY      86.0   // PWM at which the wheels start turning
#define FF_SLOPE         0.0098  // m/s gained per PWM count above breakaway
#define PID_I_MAX        110.0   // anti-windup clamp, in PWM counts
#define VEL_DEADBAND      0.005  // below this target, hold the wheel stopped
#define VEL_FILTER_ALPHA  0.35   // low-pass on measured wheel speed

// Starting gains, in PWM counts per (m/s) and per (m/s x s). Retunable at
// runtime by publishing to /pid_gains (x=Kp, y=Ki, z=feedforward breakaway),
// so tuning does not need a reflash.
double pid_kp = 45.0;
double pid_ki = 500.0;
double ff_breakaway = FF_BREAKAWAY;

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
#define RECONNECT_DELAY_MS   2000

// rmw_uros_ping_agent cannot see a closed serial port - the ESP32's writes
// still drain into the UART, so the ping keeps reporting success long after
// the agent is gone. Failed publishes are the reliable signal instead.
#define PUBLISH_FAILURE_LIMIT 20

// rmw_uros_epoch_nanos() returns time since boot until the session clock is
// synced against the agent. Unsynced stamps put /tf ~1.7e9 s in the past, so
// RViz cannot resolve odom -> laser and the scan vanishes under fixed frame
// odom. Sync once per session, then periodically to cover drift.
#define TIME_SYNC_PERIOD_MS 10000

// ============================================================
// DEBUG OUTPUT
// ============================================================
// Serial (UART0) belongs to the micro-ROS transport - anything printed on it
// is injected into the XRCE-DDS stream and corrupts the link. Debug goes to
// UART2 instead; with DEBUG_ENABLED 0 the prints compile away entirely.

#define DEBUG_ENABLED 0
#define DEBUG_BAUD    115200
#define DEBUG_RX_PIN  16
#define DEBUG_TX_PIN  17

#if DEBUG_ENABLED
  #define DBG_BEGIN() Serial2.begin(DEBUG_BAUD, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN)
  #define DBG(x)      Serial2.print(x)
  #define DBGLN(x)    Serial2.println(x)
  #define DBGNL()     Serial2.println()
#else
  #define DBG_BEGIN() do {} while (0)
  #define DBG(x)      do {} while (0)
  #define DBGLN(x)    do {} while (0)
  #define DBGNL()     do {} while (0)
#endif

// ============================================================
// SERIAL TRANSPORT
// ============================================================
// micro_ros_arduino's default transport hardcodes 115200, which caps the link
// at 11.5 KB/s - below the ~18 KB/s this firmware publishes at 20 Hz. Same
// callbacks as the stock transport, just a faster line. The agent must be
// started with a matching -b MICROROS_BAUD.

#define MICROROS_BAUD 460800

// nav_msgs/Odometry is 724 B against a 512 B serial MTU. Publishing it from
// here fragments every message and the reliable stream then blocks ~1 s per
// cycle waiting for delivery confirmation, which pins odom -> base_footprint
// to 1 Hz at any baud rate. Odometry is integrated on the Pi instead, from the
// encoder counts below - see nexva_frimware/wheel_odometry.py. Set to 1 to put
// it back on the ESP32.
#define PUBLISH_ODOM_FROM_ESP32 0

extern "C" bool nexva_transport_open(struct uxrCustomTransport *t) {
  (void)t;
  Serial.begin(MICROROS_BAUD);
  return true;
}

extern "C" bool nexva_transport_close(struct uxrCustomTransport *t) {
  (void)t;
  Serial.end();
  return true;
}

extern "C" size_t nexva_transport_write(struct uxrCustomTransport *t,
                                        const uint8_t *buf, size_t len,
                                        uint8_t *err) {
  (void)t;
  (void)err;
  return Serial.write(buf, len);
}

extern "C" size_t nexva_transport_read(struct uxrCustomTransport *t,
                                       uint8_t *buf, size_t len, int timeout,
                                       uint8_t *err) {
  (void)t;
  (void)err;
  Serial.setTimeout(timeout);
  return Serial.readBytes((char *)buf, len);
}

// ============================================================
// MICRO-ROS OBJECTS
// ============================================================

rcl_node_t node;
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t gains_subscriber;
rcl_publisher_t encoder_publisher;
rcl_publisher_t pwm_publisher;
rcl_publisher_t wheelvel_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t tf_publisher;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

geometry_msgs__msg__Twist cmd_vel_msg;
geometry_msgs__msg__Vector3 gains_msg;
geometry_msgs__msg__Vector3 encoder_msg;
geometry_msgs__msg__Vector3 pwm_msg;
geometry_msgs__msg__Vector3 wheelvel_msg;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped tf_transform;

// ============================================================
// STATE
// ============================================================

enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  CLEANING_UP
};

AgentState agent_state = WAITING_AGENT;
AgentState previous_state = WAITING_AGENT;

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

// Velocity targets the control loop chases, in m/s at the wheel.
double target_left_velocity = 0.0;
double target_right_velocity = 0.0;

// Measured (filtered) wheel velocities and the loop's integral state.
double measured_left_velocity = 0.0;
double measured_right_velocity = 0.0;
double integral_left = 0.0;
double integral_right = 0.0;
int applied_left_pwm = 0;
int applied_right_pwm = 0;

long prev_control_left = 0;
long prev_control_right = 0;
unsigned long last_control_us = 0;

// ============================================================
// FLAGS
// ============================================================

bool odom_message_initialized = false;
bool tf_message_initialized = false;
bool entities_created = false;

int publish_failures = 0;

void trackPublish(rcl_ret_t rc) {
  if (rc == RCL_RET_OK) {
    publish_failures = 0;
  } else if (publish_failures < PUBLISH_FAILURE_LIMIT) {
    publish_failures++;
  }
}

// ============================================================
// ERROR HANDLING
// ============================================================

void error_loop() {
  stopMotors();
  while (true) {
    delay(100);
  }
}

#define RCCHECK(fn)                                      \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if (temp_rc != RCL_RET_OK) {                         \
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

void setLeftMotor(int pwm) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_ENA, pwm);
  } else if (pwm < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    analogWrite(LEFT_ENA, -pwm);
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_ENA, 0);
  }
}

void setRightMotor(int pwm) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_ENB, pwm);
  } else if (pwm < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(RIGHT_ENB, -pwm);
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_ENB, 0);
  }
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  current_linear = 0.0;
  current_angular = 0.0;
  // Clear the loop too. Leaving the targets or the integral set would have the
  // controller fight the stop and lurch the moment it is allowed to run again.
  target_left_velocity = 0.0;
  target_right_velocity = 0.0;
  integral_left = 0.0;
  integral_right = 0.0;
  applied_left_pwm = 0;
  applied_right_pwm = 0;
  last_control_us = 0;   // re-seed dt rather than carry the idle gap forward
}

// ============================================================
// ENCODER INTERRUPTS
// ============================================================

void IRAM_ATTR leftEncoderISR() {
  int a = digitalRead(LEFT_ENCODER_A);
  int b = digitalRead(LEFT_ENCODER_B);
  if (a == b)
    left_encoder_count++;
  else
    left_encoder_count--;
}

void IRAM_ATTR rightEncoderISR() {
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

// cmd_vel now only sets velocity TARGETS. Nothing here touches PWM - that is
// the control loop's job, once it can compare the target against what the
// wheels are actually doing.
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  current_linear = constrain(msg->linear.x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
  current_angular = constrain(msg->angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

  double left_velocity = current_linear - (current_angular * WHEEL_BASE / 2.0);
  double right_velocity = current_linear + (current_angular * WHEEL_BASE / 2.0);

  // A turn can push one wheel past the limit. Scale both together so the
  // commanded turning ratio survives, instead of clipping one wheel and
  // silently straightening the curve.
  double peak = max(fabs(left_velocity), fabs(right_velocity));
  if (peak > MAX_LINEAR_SPEED) {
    double scale = MAX_LINEAR_SPEED / peak;
    left_velocity *= scale;
    right_velocity *= scale;
  }

  target_left_velocity = left_velocity;
  target_right_velocity = right_velocity;

  last_cmd_time = millis();
}

// Live gain tuning: publish geometry_msgs/Vector3 to /pid_gains with
// x = Kp, y = Ki, z = feedforward breakaway PWM. Zero or negative leaves that
// term alone, so any one of the three can be adjusted on its own.
void gainsCallback(const void *msgin) {
  const geometry_msgs__msg__Vector3 *msg =
      (const geometry_msgs__msg__Vector3 *)msgin;
  if (msg->x > 0.0) pid_kp = msg->x;
  if (msg->y >= 0.0) pid_ki = msg->y;
  if (msg->z > 0.0) ff_breakaway = msg->z;
  integral_left = 0.0;
  integral_right = 0.0;
}

// ============================================================
// WHEEL VELOCITY CONTROL LOOP
// ============================================================

static inline double countsToMetres(long counts) {
  return ((double)counts / ENCODER_CPR) * (2.0 * PI * WHEEL_RADIUS);
}

// One wheel's PI + feedforward step. Returns the PWM to apply.
static int wheelControl(double target, double measured, double *integral,
                        double dt) {
  if (fabs(target) < VEL_DEADBAND) {
    *integral = 0.0;
    return 0;
  }

  double error = target - measured;
  double sign = (target > 0.0) ? 1.0 : -1.0;

  // Feedforward: the duty cycle this speed needed last time we measured the
  // motors. Only ever an estimate - the integrator carries the rest.
  double ff = sign * (ff_breakaway + fabs(target) / FF_SLOPE);

  double candidate = ff + pid_kp * error + pid_ki * (*integral);

  // Integrate only while we have authority left, so a saturated output does
  // not keep winding the integral up and overshoot on the way back down.
  if (candidate > -MAX_PWM && candidate < MAX_PWM) {
    *integral += error * dt;
    *integral = constrain(*integral, -PID_I_MAX / max(pid_ki, 1.0),
                                      PID_I_MAX / max(pid_ki, 1.0));
    candidate = ff + pid_kp * error + pid_ki * (*integral);
  }

  return (int)constrain(candidate, -(double)MAX_PWM, (double)MAX_PWM);
}

void controlLoop() {
  unsigned long now = micros();
  if (last_control_us == 0) {
    last_control_us = now;
    return;
  }
  double dt = (now - last_control_us) / 1000000.0;
  if (dt < 1.0 / CONTROL_HZ) {
    return;
  }
  last_control_us = now;

  // The loop does not run while the base is stopped, so the first tick after
  // driving resumes sees dt equal to the whole idle period. Integrating that
  // in one step drove the integral straight to its clamp and produced a
  // full-power kick - measured at PWM 246 for a 0.15 m/s request. Cap dt so a
  // gap can only ever contribute one normal step.
  if (dt > 2.0 / CONTROL_HZ) {
    dt = 2.0 / CONTROL_HZ;
  }

  noInterrupts();
  long l = left_encoder_count;
  long r = right_encoder_count;
  interrupts();

  double raw_left = countsToMetres(l - prev_control_left) / dt;
  double raw_right = countsToMetres(r - prev_control_right) / dt;
  prev_control_left = l;
  prev_control_right = r;

  measured_left_velocity += VEL_FILTER_ALPHA * (raw_left - measured_left_velocity);
  measured_right_velocity += VEL_FILTER_ALPHA * (raw_right - measured_right_velocity);

  applied_left_pwm = wheelControl(target_left_velocity, measured_left_velocity,
                                  &integral_left, dt);
  applied_right_pwm = wheelControl(target_right_velocity, measured_right_velocity,
                                   &integral_right, dt);

  setLeftMotor(applied_left_pwm);
  setRightMotor(applied_right_pwm);
}

// ============================================================
// RESET ODOMETRY
// ============================================================

void resetOdometry() {
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

bool initializeOdomMessage() {
  if (!nav_msgs__msg__Odometry__init(&odom_msg)) {
    return false;
  }

  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.header.frame_id.capacity = strlen("odom") + 1;

  odom_msg.child_frame_id.data = (char *)"base_footprint";
  odom_msg.child_frame_id.size = strlen("base_footprint");
  odom_msg.child_frame_id.capacity = strlen("base_footprint") + 1;

  odom_message_initialized = true;
  return true;
}

// ============================================================
// INITIALIZE TF MESSAGE
// ============================================================

bool initializeTFMessage() {
  if (!tf2_msgs__msg__TFMessage__init(&tf_msg)) {
    return false;
  }

  if (!geometry_msgs__msg__TransformStamped__init(&tf_transform)) {
    return false;
  }

  tf_transform.header.frame_id.data = (char *)"odom";
  tf_transform.header.frame_id.size = strlen("odom");
  tf_transform.header.frame_id.capacity = strlen("odom") + 1;

  tf_transform.child_frame_id.data = (char *)"base_footprint";
  tf_transform.child_frame_id.size = strlen("base_footprint");
  tf_transform.child_frame_id.capacity = strlen("base_footprint") + 1;

  tf_msg.transforms.data = &tf_transform;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.capacity = 1;

  tf_message_initialized = true;
  return true;
}

// ============================================================
// CREATE MICRO-ROS ENTITIES
// ============================================================
// Publishers stay RELIABLE on purpose. tf2_ros::TransformListener subscribes
// to /tf as RELIABLE and offers no way to change that, and RViz/ros2 topic
// default to RELIABLE too - a BEST_EFFORT publisher simply never matches them.
// The 1 Hz stall this used to cause was a bandwidth problem, not a QoS one,
// and is fixed by MICROROS_BAUD above.

bool createEntities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    DBGLN("Failed to init support");
    return false;
  }

  // A destroy timeout of 0 keeps destroyEntities() from blocking on delete
  // acknowledgements the agent can no longer send after it is killed. Without
  // this, teardown stalls for minutes before the ESP32 will reconnect.
  rmw_uros_set_context_entity_destroy_session_timeout(
      rcl_context_get_rmw_context(&support.context), 0);

  if (rclc_node_init_default(&node, "nexva_esp32", "", &support) != RCL_RET_OK) {
    DBGLN("Failed to init node");
    return false;
  }

  if (rclc_subscription_init_default(&cmd_vel_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/cmd_vel") != RCL_RET_OK) {
    DBGLN("Failed to create cmd_vel subscriber");
    return false;
  }

  // Both counts travel in one message. Split across two topics they could
  // arrive out of step, and the Pi would pair a fresh left against a stale
  // right - which integrates as a rotation that never happened.
  //
  // BEST_EFFORT is essential here, not an optimisation. A reliable publish
  // calls uxr_run_session_until_confirm_delivery() and blocks until the agent
  // acknowledges, which times out at RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT
  // (1000 ms) and pins the whole control loop to 1 Hz - at any baud rate and
  // any message size. Nothing but nexva_frimware's own nodes read this topic,
  // and they subscribe BEST_EFFORT to match; /odom and /tf are re-published
  // from the Pi as RELIABLE for tf2 and RViz.
  if (rclc_publisher_init_best_effort(&encoder_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
          "/enco/counts") != RCL_RET_OK) {
    DBGLN("Failed to create encoder publisher");
    return false;
  }

  // Observability for the control loop. BEST_EFFORT: these are debug streams,
  // a dropped sample is fine and must never block the reliable odom stream.
  // /motor_pwm  x = left PWM,            y = right PWM
  // /wheel_vel  x = left measured m/s,   y = right measured m/s
  if (rclc_publisher_init_best_effort(&pwm_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
          "/motor_pwm") != RCL_RET_OK) {
    DBGLN("Failed to create PWM publisher");
    return false;
  }

  if (rclc_publisher_init_best_effort(&wheelvel_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
          "/wheel_vel") != RCL_RET_OK) {
    DBGLN("Failed to create wheel velocity publisher");
    return false;
  }

  if (rclc_subscription_init_default(&gains_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
          "/pid_gains") != RCL_RET_OK) {
    DBGLN("Failed to create gains subscriber");
    return false;
  }

#if PUBLISH_ODOM_FROM_ESP32
  if (rclc_publisher_init_default(&odom_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
          "/odom") != RCL_RET_OK) {
    DBGLN("Failed to create odom publisher");
    return false;
  }

  if (rclc_publisher_init_default(&tf_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
          "/tf") != RCL_RET_OK) {
    DBGLN("Failed to create tf publisher");
    return false;
  }
#endif

  geometry_msgs__msg__Twist__init(&cmd_vel_msg);
  geometry_msgs__msg__Vector3__init(&encoder_msg);
  geometry_msgs__msg__Vector3__init(&pwm_msg);
  geometry_msgs__msg__Vector3__init(&wheelvel_msg);
  geometry_msgs__msg__Vector3__init(&gains_msg);

  if (!initializeOdomMessage() || !initializeTFMessage()) {
    DBGLN("Failed to initialize messages");
    return false;
  }

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
    DBGLN("Failed to init executor");
    return false;
  }

  if (rclc_executor_add_subscription(&executor, &cmd_vel_subscriber,
          &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) {
    DBGLN("Failed to add subscription to executor");
    return false;
  }

  if (rclc_executor_add_subscription(&executor, &gains_subscriber,
          &gains_msg, &gainsCallback, ON_NEW_DATA) != RCL_RET_OK) {
    DBGLN("Failed to add gains subscription to executor");
    return false;
  }

  // Must happen before the first publishOdometry(), or /tf and /odom go out
  // stamped with ESP32 uptime instead of ROS time.
  rmw_uros_sync_session(1000);

  resetOdometry();
  last_cmd_time = millis();
  publish_failures = 0;

  DBGNL();
  DBGLN("--------------------------------");
  DBGLN("MICRO-ROS ENTITIES CREATED");
  DBGLN("Node: /nexva_esp32");
  DBGLN("Session: NEW");
  DBGLN("--------------------------------");
  DBGNL();

  entities_created = true;
  return true;
}

// ============================================================
// DESTROY MICRO-ROS ENTITIES
// ============================================================

void destroyEntities() {
  if (!entities_created) {
    return;
  }

  DBGLN("Destroying Micro-ROS entities...");

  stopMotors();

  if (executor.context != NULL) {
    rclc_executor_fini(&executor);
  }

  rcl_publisher_fini(&encoder_publisher, &node);
  rcl_publisher_fini(&pwm_publisher, &node);
  rcl_publisher_fini(&wheelvel_publisher, &node);
#if PUBLISH_ODOM_FROM_ESP32
  rcl_publisher_fini(&odom_publisher, &node);
  rcl_publisher_fini(&tf_publisher, &node);
#endif

  rcl_subscription_fini(&cmd_vel_subscriber, &node);
  rcl_subscription_fini(&gains_subscriber, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  odom_message_initialized = false;
  tf_message_initialized = false;
  entities_created = false;

  DBGLN("Micro-ROS entities destroyed.");
  DBGLN("Waiting for agent reconnection...");
}

// ============================================================
// CHECK AGENT
// ============================================================

bool agentAvailable() {
  rcl_ret_t rc = rmw_uros_ping_agent(100, 1);
  return rc == RCL_RET_OK;
}

// ============================================================
// PUBLISH ENCODERS
// ============================================================

void publishEncoders() {
  noInterrupts();
  long left_count = left_encoder_count;
  long right_count = right_encoder_count;
  interrupts();

  encoder_msg.x = (double)left_count;
  encoder_msg.y = (double)right_count;
  encoder_msg.z = 0.0;

  trackPublish(rcl_publish(&encoder_publisher, &encoder_msg, NULL));

  pwm_msg.x = (double)applied_left_pwm;
  pwm_msg.y = (double)applied_right_pwm;
  pwm_msg.z = 0.0;
  rcl_publish(&pwm_publisher, &pwm_msg, NULL);

  wheelvel_msg.x = measured_left_velocity;
  wheelvel_msg.y = measured_right_velocity;
  wheelvel_msg.z = 0.0;
  rcl_publish(&wheelvel_publisher, &wheelvel_msg, NULL);
}

// ============================================================
// PUBLISH ODOMETRY
// ============================================================

void publishOdometry() {
  unsigned long now = millis();
  double dt = (now - last_odom_time) / 1000.0;

  if (dt <= 0.0)
    return;

  last_odom_time = now;

  noInterrupts();
  long left_count = left_encoder_count;
  long right_count = right_encoder_count;
  interrupts();

  long delta_left = left_count - previous_left_count;
  long delta_right = right_count - previous_right_count;

  previous_left_count = left_count;
  previous_right_count = right_count;

  double left_distance = (delta_left / ENCODER_CPR) * (2.0 * PI * WHEEL_RADIUS);
  double right_distance = (delta_right / ENCODER_CPR) * (2.0 * PI * WHEEL_RADIUS);

  double distance = (left_distance + right_distance) / 2.0;
  double delta_theta = (right_distance - left_distance) / WHEEL_BASE;

  theta_position += delta_theta;
  x_position += distance * cos(theta_position);
  y_position += distance * sin(theta_position);

  double linear_velocity = distance / dt;
  double angular_velocity = delta_theta / dt;

  int64_t stamp = rmw_uros_epoch_nanos();

  odom_msg.header.stamp.sec = stamp / 1000000000LL;
  odom_msg.header.stamp.nanosec = stamp % 1000000000LL;

  odom_msg.pose.pose.position.x = x_position;
  odom_msg.pose.pose.position.y = y_position;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(theta_position / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta_position / 2.0);

  odom_msg.twist.twist.linear.x = linear_velocity;
  odom_msg.twist.twist.angular.z = angular_velocity;

#if PUBLISH_ODOM_FROM_ESP32
  trackPublish(rcl_publish(&odom_publisher, &odom_msg, NULL));
#endif

  // TF
  tf_transform.header.stamp.sec = odom_msg.header.stamp.sec;
  tf_transform.header.stamp.nanosec = odom_msg.header.stamp.nanosec;

  tf_transform.transform.translation.x = x_position;
  tf_transform.transform.translation.y = y_position;
  tf_transform.transform.translation.z = 0.0;

  tf_transform.transform.rotation.x = 0.0;
  tf_transform.transform.rotation.y = 0.0;
  tf_transform.transform.rotation.z = sin(theta_position / 2.0);
  tf_transform.transform.rotation.w = cos(theta_position / 2.0);

#if PUBLISH_ODOM_FROM_ESP32
  trackPublish(rcl_publish(&tf_publisher, &tf_msg, NULL));
#endif
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  // Serial is opened by set_microros_transports() below - do not touch it here.
  DBG_BEGIN();
  delay(1000);

  // Motor setup
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

  // Encoder setup
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);

  DBGNL();
  DBGLN("================================");
  DBGLN("NEXVA ESP32 STARTING");
  DBGLN("================================");
  DBGLN("Initializing Micro-ROS transport...");

  rmw_uros_set_custom_transport(true, NULL,
                                nexva_transport_open, nexva_transport_close,
                                nexva_transport_write, nexva_transport_read);

  delay(500);

  DBGLN("Micro-ROS transport initialized.");
  DBGLN("Waiting for Micro-ROS Agent...");

  agent_state = WAITING_AGENT;
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  static unsigned long last_agent_check = 0;
  static unsigned long last_odom_publish = 0;
  static unsigned long disconnection_time = 0;
  static unsigned long last_time_sync = 0;

  // Track state changes
  if (agent_state != previous_state) {
    previous_state = agent_state;
  }

  // WAIT FOR AGENT
  if (agent_state == WAITING_AGENT) {
    if (millis() - last_agent_check >= AGENT_CHECK_PERIOD_MS) {
      last_agent_check = millis();

      if (agentAvailable()) {
        DBGNL();
        DBGLN("Micro-ROS Agent detected.");
        agent_state = AGENT_AVAILABLE;
      }
    }

    stopMotors();
    delay(10);
    return;
  }

  // CREATE NEW SESSION
  if (agent_state == AGENT_AVAILABLE) {
    DBGLN("Creating NEW Micro-ROS session...");

    if (createEntities()) {
      agent_state = AGENT_CONNECTED;
      DBGLN("NEW Micro-ROS session CONNECTED.");
    } else {
      DBGLN("Entity creation failed. Retrying...");
      delay(1000);
    }

    return;
  }

  // CONNECTED
  if (agent_state == AGENT_CONNECTED) {
    // Timeout MUST be 0. rcl_wait() documents 0 as a non-blocking poll, but
    // any small non-zero value is not honoured through this rmw layer and
    // falls through to a ~1 s block - measured at 995 ms for RCL_MS_TO_NS(5).
    // That alone pinned the whole loop, and therefore /enco/counts, to 1 Hz.
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));

    if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
      stopMotors();
    } else {
      // Self-gated to CONTROL_HZ; the surrounding loop runs far faster.
      controlLoop();
    }

    if (millis() - last_odom_publish >= ODOM_PERIOD_MS) {
      last_odom_publish = millis();
      publishEncoders();
      publishOdometry();
    }

    if (publish_failures >= PUBLISH_FAILURE_LIMIT) {
      agent_state = AGENT_DISCONNECTED;
      disconnection_time = millis();
      return;
    }

    if (millis() - last_time_sync >= TIME_SYNC_PERIOD_MS) {
      last_time_sync = millis();
      rmw_uros_sync_session(100);
    }

    if (millis() - last_agent_check >= AGENT_CHECK_PERIOD_MS) {
      last_agent_check = millis();

      if (!agentAvailable()) {
        agent_state = AGENT_DISCONNECTED;
        disconnection_time = millis();
      }
    }

    delay(2);
    return;
  }

  // AGENT DISCONNECTED - REBOOT FOR A CLEAN SESSION
  if (agent_state == AGENT_DISCONNECTED) {
    DBGNL();
    DBGLN("================================");
    DBGLN("MICRO-ROS AGENT DISCONNECTED");
    DBGLN("Rebooting...");
    DBGLN("================================");

    // Tearing the session down in place does not work: the client keeps
    // pinging with the old session id (0x81) and a fresh agent only answers
    // session-create requests (0x80), so it never reconnects. A reboot is the
    // only way to guarantee a clean session, transport and UART.
    stopMotors();
    delay(50);
    ESP.restart();
  }

  // CLEANING UP - WAIT BEFORE RECONNECT
  if (agent_state == CLEANING_UP) {
    if (millis() - disconnection_time >= RECONNECT_DELAY_MS) {
      agent_state = WAITING_AGENT;
      last_agent_check = millis();
    }

    stopMotors();
    delay(100);
    return;
  }
}