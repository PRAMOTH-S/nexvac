// ============================================================
// NEXVA ESP32 - micro-ROS differential drive base
// ============================================================
// Arduino sketch (.ino) for ESP32. Put this file in a folder with the same
// name, i.e.  nexva_esp32/nexva_esp32.ino , or the Arduino IDE will not open
// it.
//
// Task layout (FreeRTOS, both cores used on purpose):
//
//   core 0, prio 3 : rosTask      - micro-ROS session, executor, publishing
//   core 1, prio 5 : controlTask  - wheel velocity PI loop, hard 50 Hz
//   core 1, prio 1 : loop()       - Arduino's own task, parked and idle
//
// micro-ROS (rcl / rclc / the XRCE session) is NOT thread safe. Every rcl_*,
// rclc_* and rmw_* call in this file happens on rosTask and nowhere else.
// controlTask never touches ROS, and rosTask never touches a motor pin - it
// only asks for a stop through requestStop(). That split is what keeps a
// blocking session call from stalling the control loop, which is the whole
// reason for using tasks here.

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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

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

// Usable PWM band. Any non-zero command is pushed into [min_pwm, max_pwm] in
// the direction the target asks for, so the motor is never handed a duty cycle
// too small to actually turn the wheel - which is what made it sit and whine.
// Zero target still means a true 0 (both IN pins LOW, wheel coasts).
//
// These are the power-on DEFAULTS. The live values are the variables below and
// are retunable from /pid_limits without a reflash - see limitsCallback().
#define DEFAULT_MAX_PWM 255
#define DEFAULT_MIN_PWM 150

// The real speed limits, enforced on the velocity TARGET. Because the loop
// below closes on measured wheel speed, these are honoured whatever the pack
// voltage is doing.
#define DEFAULT_MAX_LINEAR_SPEED  0.30
#define MAX_ANGULAR_SPEED 2.0

// Hard bounds on what /pid_limits is allowed to set. A UI slider, a typo or a
// stale message must not be able to command a duty cycle the driver cannot do
// or a speed this chassis cannot stop from. The floor of 40 on max_pwm is
// simply "low enough to be useless, high enough not to be a divide-by-nothing".
#define PWM_ABS_MAX            255
#define PWM_ABS_MIN              0
#define MAX_PWM_LOWER_BOUND     40
#define SPEED_ABS_MAX          0.50

// Live limits. int (not double) on purpose: a 32-bit aligned load is atomic on
// the ESP32, so setLeftMotor/setRightMotor can read these straight from
// controlTask without taking state_mux in the motor path.
int motor_min_pwm = DEFAULT_MIN_PWM;
int motor_max_pwm = DEFAULT_MAX_PWM;

// Only ever touched on rosTask (cmdVelCallback reads it, limitsCallback writes
// it), so unlike the PWM band it needs no cross-task guard.
double max_linear_speed = DEFAULT_MAX_LINEAR_SPEED;

// ---------- closed-loop wheel velocity control ----------
//
// Open loop was the root cause of every speed problem this robot has had. The
// PWM->speed curve moved between 23 Sep (0.00341 * (PWM-147)) and 28 Sep
// (0.0065 * (PWM-108)) with no code change at all - battery state of charge.
// A PI loop on measured wheel speed makes that irrelevant.
//
// Feedforward gets the wheel moving immediately; the integrator absorbs
// whatever the feedforward got wrong, including pack voltage drift.
#define CONTROL_PERIOD_MS 20                          // 50 Hz
#define CONTROL_HZ        (1000.0 / CONTROL_PERIOD_MS)

// Matches the default MIN_PWM: the floor is the breakaway, so the feedforward
// starts exactly where the wheel starts turning instead of below it. Raising
// min_pwm at runtime does NOT move this by itself - set z on /pid_gains too,
// or the feedforward starts below the new floor and the clamp does the work.
#define FF_BREAKAWAY      ((double)DEFAULT_MIN_PWM)
#define FF_SLOPE         0.0098  // m/s gained per PWM count above breakaway
#define PID_I_MAX        110.0   // anti-windup clamp, in PWM counts
#define VEL_DEADBAND      0.005  // below this target, hold the wheel stopped
#define VEL_FILTER_ALPHA  0.35   // low-pass on measured wheel speed

// Backstop for the case where MIN_PWM still is not enough to break static
// friction (thick carpet, a low pack, a stiff gearbox). While a target is set
// and the wheel is not turning, the output climbs at a fixed rate instead of
// waiting on (small error x Ki), which took over a second on its own.
#define STALL_SPEED       0.01   // m/s: measured speed below this = stalled
#define STALL_RAMP_PWM    150.0  // PWM counts per second added while stalled

// Starting gains, in PWM counts per (m/s) and per (m/s x s). Retunable at
// runtime by publishing to /pid_gains (x=Kp, y=Ki, z=feedforward breakaway),
// so tuning does not need a reflash. Guarded by state_mux - rosTask writes
// them, controlTask reads them.
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
// TASK CONFIGURATION
// ============================================================
// micro-ROS needs a large stack - rcl plus the XRCE session plus the message
// (de)serialisers all live on it. 16 KB is the smallest size that has proved
// reliable; below about 12 KB it stack-overflows during session creation.
#define ROS_TASK_STACK        16384
#define ROS_TASK_PRIORITY     3
#define ROS_TASK_CORE         0

// The control loop is small (floats and two analogWrite calls) but must never
// be late, so it gets the higher priority and its own core.
#define CONTROL_TASK_STACK    4096
#define CONTROL_TASK_PRIORITY 5
#define CONTROL_TASK_CORE     1

TaskHandle_t ros_task_handle = NULL;
TaskHandle_t control_task_handle = NULL;

// Two spinlocks rather than one mutex. Both critical sections are a handful of
// assignments, so they finish in well under a microsecond and cannot cause the
// priority inversion a mutex could between a prio-5 and a prio-3 task.
//
//   enc_mux   - encoder counts. Shared with the ISRs, so ISR context must use
//               the portENTER_CRITICAL_ISR variant.
//   state_mux - commands, gains and telemetry shared by the two tasks.
portMUX_TYPE enc_mux   = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

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
// Touched only by rosTask.

rcl_node_t node;
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t gains_subscriber;
rcl_subscription_t limits_subscriber;
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
geometry_msgs__msg__Vector3 limits_msg;
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
// ENCODERS  (guarded by enc_mux)
// ============================================================

volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// ============================================================
// ODOMETRY  (rosTask only)
// ============================================================

double x_position = 0.0;
double y_position = 0.0;
double theta_position = 0.0;

long previous_left_count = 0;
long previous_right_count = 0;

unsigned long last_odom_time = 0;

// ============================================================
// SHARED COMMAND / TELEMETRY  (guarded by state_mux)
// ============================================================

unsigned long last_cmd_time = 0;
float current_linear = 0.0;
float current_angular = 0.0;

// Velocity targets the control loop chases, in m/s at the wheel.
double target_left_velocity = 0.0;
double target_right_velocity = 0.0;

// Set false whenever the base must not drive: no agent, command timeout,
// shutting down. controlTask is the only thing that writes the motor pins, so
// this flag is how every other part of the firmware stops the robot.
bool drive_enabled = false;

// Filled in by controlTask, published by rosTask.
double measured_left_velocity = 0.0;
double measured_right_velocity = 0.0;
int applied_left_pwm = 0;
int applied_right_pwm = 0;

// controlTask private state - no lock needed.
double integral_left = 0.0;
double integral_right = 0.0;
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

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
// The .ino preprocessor usually generates these, but it gives up on files
// that mix classes and macros. Declaring them by hand keeps the build honest.

void setLeftMotor(int pwm);
void setRightMotor(int pwm);
void requestStop();
void forceMotorsOff();
void error_loop();
void resetOdometry();
bool createEntities();
void destroyEntities();
bool agentAvailable();
void publishEncoders();
void publishOdometry();
void controlStep();
void controlTask(void *arg);
void rosTask(void *arg);

// ============================================================
// PUBLISH TRACKING
// ============================================================

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
  // Suspend the control task first so nothing fights us for the motor pins,
  // then kill the outputs directly and stay dead.
  if (control_task_handle != NULL) {
    vTaskSuspend(control_task_handle);
  }
  forceMotorsOff();

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));
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
// MOTOR OUTPUT
// ============================================================
// Called from controlTask only, except forceMotorsOff() on the shutdown and
// error paths - and those suspend controlTask first.

void setLeftMotor(int pwm) {
  pwm = constrain(pwm, -motor_max_pwm, motor_max_pwm);

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
  pwm = constrain(pwm, -motor_max_pwm, motor_max_pwm);

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

void forceMotorsOff() {
  setLeftMotor(0);
  setRightMotor(0);
}

// Ask for a stop from any task. This does NOT write the motor pins: it clears
// the targets and drops drive_enabled, and controlTask zeroes the outputs on
// its next tick (within CONTROL_PERIOD_MS). Keeping all pin writes on one task
// is what makes the two-task design safe.
void requestStop() {
  portENTER_CRITICAL(&state_mux);
  drive_enabled = false;
  target_left_velocity = 0.0;
  target_right_velocity = 0.0;
  current_linear = 0.0;
  current_angular = 0.0;
  portEXIT_CRITICAL(&state_mux);
}

// ============================================================
// ENCODER INTERRUPTS
// ============================================================
// Single-edge, single-channel: A rising or falling, direction from B. Uses the
// _ISR spinlock variant, which is mandatory in interrupt context.

void IRAM_ATTR leftEncoderISR() {
  int a = digitalRead(LEFT_ENCODER_A);
  int b = digitalRead(LEFT_ENCODER_B);

  portENTER_CRITICAL_ISR(&enc_mux);
  if (a == b)
    left_encoder_count++;
  else
    left_encoder_count--;
  portEXIT_CRITICAL_ISR(&enc_mux);
}

void IRAM_ATTR rightEncoderISR() {
  int a = digitalRead(RIGHT_ENCODER_A);
  int b = digitalRead(RIGHT_ENCODER_B);

  portENTER_CRITICAL_ISR(&enc_mux);
  if (a == b)
    right_encoder_count++;
  else
    right_encoder_count--;
  portEXIT_CRITICAL_ISR(&enc_mux);
}

// Read both counts as one consistent pair. Taken separately, a fresh left
// against a stale right integrates as a rotation that never happened.
static inline void readEncoders(long *l, long *r) {
  portENTER_CRITICAL(&enc_mux);
  *l = left_encoder_count;
  *r = right_encoder_count;
  portEXIT_CRITICAL(&enc_mux);
}

// ============================================================
// CMD_VEL CALLBACK   (rosTask)
// ============================================================

// cmd_vel only sets velocity TARGETS. Nothing here touches PWM - that is
// controlTask's job, once it can compare the target against what the wheels
// are actually doing.
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  double linear  = constrain(msg->linear.x, -max_linear_speed, max_linear_speed);
  double angular = constrain(msg->angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

  double left_velocity  = linear - (angular * WHEEL_BASE / 2.0);
  double right_velocity = linear + (angular * WHEEL_BASE / 2.0);

  // A turn can push one wheel past the limit. Scale both together so the
  // commanded turning ratio survives, instead of clipping one wheel and
  // silently straightening the curve.
  double peak = fmax(fabs(left_velocity), fabs(right_velocity));
  if (peak > max_linear_speed) {
    double scale = max_linear_speed / peak;
    left_velocity  *= scale;
    right_velocity *= scale;
  }

  portENTER_CRITICAL(&state_mux);
  current_linear  = (float)linear;
  current_angular = (float)angular;
  target_left_velocity  = left_velocity;
  target_right_velocity = right_velocity;
  drive_enabled = true;
  last_cmd_time = millis();
  portEXIT_CRITICAL(&state_mux);
}

// Live gain tuning: publish geometry_msgs/Vector3 to /pid_gains with
// x = Kp, y = Ki, z = feedforward breakaway PWM. Zero or negative leaves that
// term alone (except Ki, where 0 is accepted and only negative is ignored),
// so any one of the three can be adjusted on its own:
//
//   ros2 topic pub --once /pid_gains geometry_msgs/msg/Vector3 \
//        "{x: -1.0, y: -1.0, z: 160.0}"
void gainsCallback(const void *msgin) {
  const geometry_msgs__msg__Vector3 *msg =
      (const geometry_msgs__msg__Vector3 *)msgin;

  portENTER_CRITICAL(&state_mux);
  if (msg->x > 0.0)  pid_kp = msg->x;
  if (msg->y >= 0.0) pid_ki = msg->y;
  if (msg->z > 0.0)  ff_breakaway = msg->z;
  portEXIT_CRITICAL(&state_mux);

  // Private to controlTask, and a stale integral against new gains is worse
  // than a zero one.
  integral_left = 0.0;
  integral_right = 0.0;
}

// Live speed/PWM limits: publish geometry_msgs/Vector3 to /pid_limits with
// x = min PWM, y = max PWM, z = max linear speed (m/s). A value <= 0 leaves
// that field alone, so any one of the three can be changed on its own:
//
//   ros2 topic pub --once /pid_limits geometry_msgs/msg/Vector3 \
//        "{x: 120.0, y: 200.0, z: 0.22}"      all three
//   ros2 topic pub --once /pid_limits geometry_msgs/msg/Vector3 \
//        "{x: -1.0, y: -1.0, z: 0.15}"        just the speed cap
//
// min is the duty cycle at which the wheels actually break away; below it the
// motors buzz and do not turn. max caps the top duty cycle. The PI loop is
// then only ever allowed to ask for something inside that band.
//
// Everything is bounded and the pair is kept ordered, because these numbers
// arrive from a web page. An inverted band (min > max) would make wheelControl
// clamp to the floor and then to the ceiling, pinning both wheels at max - a
// robot that goes to full speed the moment a slider is dragged the wrong way.
void limitsCallback(const void *msgin) {
  const geometry_msgs__msg__Vector3 *msg =
      (const geometry_msgs__msg__Vector3 *)msgin;

  int    new_min   = motor_min_pwm;
  int    new_max   = motor_max_pwm;
  double new_speed = max_linear_speed;

  if (msg->x > 0.0) new_min = (int)constrain(msg->x, PWM_ABS_MIN, PWM_ABS_MAX);
  if (msg->y > 0.0) new_max = (int)constrain(msg->y, MAX_PWM_LOWER_BOUND,
                                             PWM_ABS_MAX);
  if (msg->z > 0.0) new_speed = constrain(msg->z, 0.01, SPEED_ABS_MAX);

  // Ordering is enforced after clamping, not before: clamping can itself
  // invert the pair (min 250 against a max clamped to 200).
  if (new_min >= new_max) new_min = new_max - 1;
  if (new_min < PWM_ABS_MIN) new_min = PWM_ABS_MIN;

  portENTER_CRITICAL(&state_mux);
  motor_min_pwm = new_min;
  motor_max_pwm = new_max;
  portEXIT_CRITICAL(&state_mux);

  // rosTask-only, so it sits outside the critical section.
  max_linear_speed = new_speed;

  // The band moved under the integrator, so whatever it had wound up to was
  // accumulated against a different clamp. Same reasoning as gainsCallback.
  integral_left = 0.0;
  integral_right = 0.0;

  DBG("limits: pwm ");
  DBG(new_min);
  DBG("..");
  DBG(new_max);
  DBG("  speed ");
  DBG(new_speed);
  DBGLN(" m/s");
}

// ============================================================
// WHEEL VELOCITY CONTROL   (controlTask)
// ============================================================

static inline double countsToMetres(long counts) {
  return ((double)counts / ENCODER_CPR) * (2.0 * PI * WHEEL_RADIUS);
}

// One wheel's feedforward + PI step. Returns the PWM to apply, already forced
// into the [min_pwm, max_pwm] band in the direction the target asks for. The
// band is passed in rather than read from the globals: controlStep snapshots
// it under state_mux once, so both wheels are controlled against the same
// limits even if /pid_limits lands mid-step.
static int wheelControl(double target, double measured, double *integral,
                        double dt, double kp, double ki, double ff_break,
                        int min_pwm, int max_pwm) {
  if (fabs(target) < VEL_DEADBAND) {
    *integral = 0.0;
    return 0;
  }

  double error   = target - measured;
  double sign    = (target > 0.0) ? 1.0 : -1.0;
  double ki_safe = (ki > 1.0) ? ki : 1.0;

  // Feedforward: the duty cycle this speed needed last time we measured the
  // motors. Only ever an estimate - the integrator carries the rest.
  double ff = sign * (ff_break + fabs(target) / FF_SLOPE);

  double candidate = ff + kp * error + ki * (*integral);

  // Everything below reasons in the driving direction, so positive always
  // means "more output", whichever way the wheel is turning.
  double out = candidate * sign;
  double signed_error = error * sign;

  bool at_ceiling = (out >= (double)max_pwm);
  bool at_floor   = (out <= (double)min_pwm);

  // Conditional integration. The band is hard at both ends now, so winding
  // past either only builds a term that has to unwind again later. The floor
  // matters as much as the ceiling: at a low target the wheel overruns what
  // was asked, the error goes negative, and without this guard the integral
  // would drive itself to its clamp for nothing.
  bool block_up   = at_ceiling && signed_error > 0.0;
  bool block_down = at_floor   && signed_error < 0.0;

  if (!block_up && !block_down) {
    // Wheel not turning although a target is set: static friction MIN_PWM did
    // not break. A small error x Ki integrates far too slowly to help, so ramp
    // at a fixed rate instead. Once the wheel moves, normal PI takes over.
    if (fabs(measured) < STALL_SPEED) {
      *integral += sign * (STALL_RAMP_PWM / ki_safe) * dt;
    } else {
      *integral += error * dt;
    }

    *integral = constrain(*integral, -PID_I_MAX / ki_safe, PID_I_MAX / ki_safe);

    candidate = ff + kp * error + ki * (*integral);
    out = candidate * sign;
  }

  // Force into the usable band. Clamping `out` rather than |candidate| keeps
  // the direction tied to the target: if the PI term ever asks for reverse to
  // shed an overshoot, the wheel coasts down at MIN_PWM instead of being
  // actively driven backwards, which would be a hard plug-brake.
  if (out < (double)min_pwm) out = (double)min_pwm;
  if (out > (double)max_pwm) out = (double)max_pwm;

  return (int)(sign * out);
}

void controlStep() {
  // ---- snapshot the shared command state ----
  bool   enabled;
  double target_l, target_r;
  double kp, ki, ff_break;
  int    min_pwm, max_pwm;
  unsigned long cmd_time;

  portENTER_CRITICAL(&state_mux);
  enabled  = drive_enabled;
  target_l = target_left_velocity;
  target_r = target_right_velocity;
  kp       = pid_kp;
  ki       = pid_ki;
  ff_break = ff_breakaway;
  min_pwm  = motor_min_pwm;
  max_pwm  = motor_max_pwm;
  cmd_time = last_cmd_time;
  portEXIT_CRITICAL(&state_mux);

  // Command watchdog lives here, not on rosTask. If the ROS side is blocked in
  // a session call the base must still notice that commands stopped arriving.
  if (enabled && (millis() - cmd_time > CMD_TIMEOUT_MS)) {
    enabled = false;
    portENTER_CRITICAL(&state_mux);
    drive_enabled = false;
    target_left_velocity = 0.0;
    target_right_velocity = 0.0;
    portEXIT_CRITICAL(&state_mux);
    target_l = 0.0;
    target_r = 0.0;
  }

  unsigned long now = micros();
  double dt;

  if (last_control_us == 0) {
    dt = 1.0 / CONTROL_HZ;
  } else {
    dt = (now - last_control_us) / 1000000.0;
  }
  last_control_us = now;

  // vTaskDelayUntil makes dt very close to CONTROL_PERIOD_MS, but a missed
  // tick or a resume after a stop must not integrate a whole gap in one step.
  // That is what produced the old full-power kick (PWM 246 for a 0.15 m/s
  // request), so cap it at two nominal periods.
  if (dt > 2.0 / CONTROL_HZ) dt = 2.0 / CONTROL_HZ;
  if (dt <= 0.0)             dt = 1.0 / CONTROL_HZ;

  // ---- measure ----
  long l, r;
  readEncoders(&l, &r);

  double raw_left  = countsToMetres(l - prev_control_left) / dt;
  double raw_right = countsToMetres(r - prev_control_right) / dt;
  prev_control_left  = l;
  prev_control_right = r;

  double meas_l = measured_left_velocity
                + VEL_FILTER_ALPHA * (raw_left - measured_left_velocity);
  double meas_r = measured_right_velocity
                + VEL_FILTER_ALPHA * (raw_right - measured_right_velocity);

  // ---- act ----
  int pwm_l = 0;
  int pwm_r = 0;

  if (enabled) {
    pwm_l = wheelControl(target_l, meas_l, &integral_left,  dt, kp, ki, ff_break,
                         min_pwm, max_pwm);
    pwm_r = wheelControl(target_r, meas_r, &integral_right, dt, kp, ki, ff_break,
                         min_pwm, max_pwm);
  } else {
    // Clear the loop as well. A live integral would have the controller fight
    // the stop and lurch the moment it is allowed to run again.
    integral_left = 0.0;
    integral_right = 0.0;
  }

  setLeftMotor(pwm_l);
  setRightMotor(pwm_r);

  // ---- publish state for rosTask ----
  portENTER_CRITICAL(&state_mux);
  measured_left_velocity  = meas_l;
  measured_right_velocity = meas_r;
  applied_left_pwm  = pwm_l;
  applied_right_pwm = pwm_r;
  portEXIT_CRITICAL(&state_mux);
}

// Hard 50 Hz. vTaskDelayUntil paces against an absolute wake time, so the
// period does not drift with however long controlStep() took - unlike a
// delay() at the bottom of a loop, which adds its own runtime every cycle.
void controlTask(void *arg) {
  (void)arg;

  const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t last_wake = xTaskGetTickCount();

  // Seed the encoder deltas so the first tick does not see a huge jump.
  readEncoders(&prev_control_left, &prev_control_right);
  last_control_us = micros();

  for (;;) {
    vTaskDelayUntil(&last_wake, period);
    controlStep();
  }
}

// ============================================================
// RESET ODOMETRY
// ============================================================

void resetOdometry() {
  x_position = 0.0;
  y_position = 0.0;
  theta_position = 0.0;

  readEncoders(&previous_left_count, &previous_right_count);

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
// CREATE MICRO-ROS ENTITIES   (rosTask)
// ============================================================
// /odom and /tf stay RELIABLE on purpose. tf2_ros::TransformListener
// subscribes to /tf as RELIABLE and offers no way to change that, and
// RViz/ros2 topic default to RELIABLE too - a BEST_EFFORT publisher simply
// never matches them.

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
  // (1000 ms) and pins rosTask to 1 Hz - at any baud rate and any message
  // size. Nothing but nexva_frimware's own nodes read this topic, and they
  // subscribe BEST_EFFORT to match; /odom and /tf are re-published from the Pi
  // as RELIABLE for tf2 and RViz.
  if (rclc_publisher_init_best_effort(&encoder_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
          "/enco/counts") != RCL_RET_OK) {
    DBGLN("Failed to create encoder publisher");
    return false;
  }

  // Observability for the control loop. BEST_EFFORT: these are debug streams,
  // a dropped sample is fine and must never block anything else.
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

  // x = min PWM, y = max PWM, z = max linear speed. See limitsCallback.
  if (rclc_subscription_init_default(&limits_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
          "/pid_limits") != RCL_RET_OK) {
    DBGLN("Failed to create limits subscriber");
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
  geometry_msgs__msg__Vector3__init(&limits_msg);

  if (!initializeOdomMessage() || !initializeTFMessage()) {
    DBGLN("Failed to initialize messages");
    return false;
  }

  // Handle count must equal the number of subscriptions added below:
  // cmd_vel, pid_gains, pid_limits. Adding a subscription without raising this
  // makes rclc_executor_add_subscription fail and entity creation abort.
  if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) {
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

  if (rclc_executor_add_subscription(&executor, &limits_subscriber,
          &limits_msg, &limitsCallback, ON_NEW_DATA) != RCL_RET_OK) {
    DBGLN("Failed to add limits subscription to executor");
    return false;
  }

  // Must happen before the first publishOdometry(), or /tf and /odom go out
  // stamped with ESP32 uptime instead of ROS time.
  rmw_uros_sync_session(1000);

  resetOdometry();

  portENTER_CRITICAL(&state_mux);
  last_cmd_time = millis();
  portEXIT_CRITICAL(&state_mux);

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
// DESTROY MICRO-ROS ENTITIES   (rosTask)
// ============================================================

void destroyEntities() {
  if (!entities_created) {
    return;
  }

  DBGLN("Destroying Micro-ROS entities...");

  requestStop();
  vTaskDelay(pdMS_TO_TICKS(2 * CONTROL_PERIOD_MS));  // let controlTask act

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
  rcl_subscription_fini(&limits_subscriber, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  odom_message_initialized = false;
  tf_message_initialized = false;
  entities_created = false;

  DBGLN("Micro-ROS entities destroyed.");
  DBGLN("Waiting for agent reconnection...");
}

// ============================================================
// CHECK AGENT   (rosTask)
// ============================================================

bool agentAvailable() {
  rcl_ret_t rc = rmw_uros_ping_agent(100, 1);
  return rc == RCL_RET_OK;
}

// ============================================================
// PUBLISH ENCODERS   (rosTask)
// ============================================================

void publishEncoders() {
  long left_count, right_count;
  readEncoders(&left_count, &right_count);

  encoder_msg.x = (double)left_count;
  encoder_msg.y = (double)right_count;
  encoder_msg.z = 0.0;

  trackPublish(rcl_publish(&encoder_publisher, &encoder_msg, NULL));

  // Snapshot the telemetry controlTask produced.
  int    pwm_l, pwm_r;
  double vel_l, vel_r;

  portENTER_CRITICAL(&state_mux);
  pwm_l = applied_left_pwm;
  pwm_r = applied_right_pwm;
  vel_l = measured_left_velocity;
  vel_r = measured_right_velocity;
  portEXIT_CRITICAL(&state_mux);

  pwm_msg.x = (double)pwm_l;
  pwm_msg.y = (double)pwm_r;
  pwm_msg.z = 0.0;
  rcl_publish(&pwm_publisher, &pwm_msg, NULL);

  wheelvel_msg.x = vel_l;
  wheelvel_msg.y = vel_r;
  wheelvel_msg.z = 0.0;
  rcl_publish(&wheelvel_publisher, &wheelvel_msg, NULL);
}

// ============================================================
// PUBLISH ODOMETRY   (rosTask)
// ============================================================

void publishOdometry() {
  unsigned long now = millis();
  double dt = (now - last_odom_time) / 1000.0;

  if (dt <= 0.0)
    return;

  last_odom_time = now;

  long left_count, right_count;
  readEncoders(&left_count, &right_count);

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
// ROS TASK
// ============================================================
// The old loop() body, now a task of its own on core 0. Every vTaskDelay in
// here is load bearing: without one the task WDT trips on core 0's idle task,
// and nothing else pinned to core 0 (the WiFi/BT stack, if you ever add it)
// would get a look in.

void rosTask(void *arg) {
  (void)arg;

  unsigned long last_agent_check = 0;
  unsigned long last_odom_publish = 0;
  unsigned long disconnection_time = 0;
  unsigned long last_time_sync = 0;

  for (;;) {
    if (agent_state != previous_state) {
      previous_state = agent_state;
    }

    // ---------- WAIT FOR AGENT ----------
    if (agent_state == WAITING_AGENT) {
      requestStop();

      if (millis() - last_agent_check >= AGENT_CHECK_PERIOD_MS) {
        last_agent_check = millis();

        if (agentAvailable()) {
          DBGNL();
          DBGLN("Micro-ROS Agent detected.");
          agent_state = AGENT_AVAILABLE;
        }
      }

      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // ---------- CREATE NEW SESSION ----------
    if (agent_state == AGENT_AVAILABLE) {
      DBGLN("Creating NEW Micro-ROS session...");

      if (createEntities()) {
        agent_state = AGENT_CONNECTED;
        DBGLN("NEW Micro-ROS session CONNECTED.");
      } else {
        DBGLN("Entity creation failed. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
      }

      continue;
    }

    // ---------- CONNECTED ----------
    if (agent_state == AGENT_CONNECTED) {
      // Timeout MUST be 0. rcl_wait() documents 0 as a non-blocking poll, but
      // any small non-zero value is not honoured through this rmw layer and
      // falls through to a ~1 s block - measured at 995 ms for
      // RCL_MS_TO_NS(5). On the old single-loop firmware that alone pinned the
      // control loop to 1 Hz. It cannot do that any more now that the loop is
      // its own task, but it would still throttle /enco/counts.
      RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));

      // The command timeout is enforced inside controlStep() now, so the base
      // keeps failing safe even while this task sits in a session call.

      if (millis() - last_odom_publish >= ODOM_PERIOD_MS) {
        last_odom_publish = millis();
        publishEncoders();
        publishOdometry();
      }

      if (publish_failures >= PUBLISH_FAILURE_LIMIT) {
        agent_state = AGENT_DISCONNECTED;
        disconnection_time = millis();
        continue;
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

      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    // ---------- AGENT DISCONNECTED - REBOOT FOR A CLEAN SESSION ----------
    if (agent_state == AGENT_DISCONNECTED) {
      DBGNL();
      DBGLN("================================");
      DBGLN("MICRO-ROS AGENT DISCONNECTED");
      DBGLN("Rebooting...");
      DBGLN("================================");

      // Tearing the session down in place does not work: the client keeps
      // pinging with the old session id (0x81) and a fresh agent only answers
      // session-create requests (0x80), so it never reconnects. A reboot is
      // the only way to guarantee a clean session, transport and UART.
      //
      // Suspend controlTask before killing the outputs, so the two tasks
      // cannot both be writing motor pins across the restart.
      requestStop();
      vTaskDelay(pdMS_TO_TICKS(2 * CONTROL_PERIOD_MS));

      if (control_task_handle != NULL) {
        vTaskSuspend(control_task_handle);
      }
      forceMotorsOff();

      vTaskDelay(pdMS_TO_TICKS(50));
      ESP.restart();
    }

    // ---------- CLEANING UP - WAIT BEFORE RECONNECT ----------
    if (agent_state == CLEANING_UP) {
      requestStop();

      if (millis() - disconnection_time >= RECONNECT_DELAY_MS) {
        agent_state = WAITING_AGENT;
        last_agent_check = millis();
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Unknown state - should not happen, but never spin hot.
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  // Serial belongs to the micro-ROS transport, which opens it from
  // nexva_transport_open() - do not touch it here.
  DBG_BEGIN();
  delay(1000);

  // ---- Motor pins ----
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

  forceMotorsOff();
  requestStop();

  // ---- Encoder pins ----
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

  agent_state = WAITING_AGENT;

  // ---- Start the tasks ----
  // Control task first, so the motors are being actively held at 0 before the
  // ROS side can ever set a target.
  BaseType_t ok;

  ok = xTaskCreatePinnedToCore(controlTask, "control", CONTROL_TASK_STACK, NULL,
                               CONTROL_TASK_PRIORITY, &control_task_handle,
                               CONTROL_TASK_CORE);
  if (ok != pdPASS) {
    DBGLN("Failed to create control task");
    error_loop();
  }

  ok = xTaskCreatePinnedToCore(rosTask, "microros", ROS_TASK_STACK, NULL,
                               ROS_TASK_PRIORITY, &ros_task_handle,
                               ROS_TASK_CORE);
  if (ok != pdPASS) {
    DBGLN("Failed to create ROS task");
    error_loop();
  }

  DBGLN("Tasks started. Waiting for Micro-ROS Agent...");
}

// ============================================================
// LOOP
// ============================================================
// Arduino's own loopTask still exists (core 1, priority 1) and cannot be
// removed from a sketch, so park it. Leaving it empty would have it spin at
// full speed and starve anything else at priority 1 on core 1; a delay yields
// the core to controlTask and the idle task instead. Do not put work here -
// it has no timing guarantee and shares a core with the control loop.

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}