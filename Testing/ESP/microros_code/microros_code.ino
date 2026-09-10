#include <Arduino.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <rmw_microros/rmw_microros.h>


// ============================================================
//                     MOTOR PINS
// ============================================================

#define LEFT_IN1   26
#define LEFT_IN2   25
#define LEFT_ENA   27

#define RIGHT_IN1  18
#define RIGHT_IN2  19
#define RIGHT_ENB  23


// ============================================================
//                    ENCODER PINS
// ============================================================

#define RIGHT_ENCODER_A  32
#define RIGHT_ENCODER_B  33

#define LEFT_ENCODER_A   22
#define LEFT_ENCODER_B  21


// ============================================================
//                     MOTOR SETTINGS
// ============================================================

#define PWM_FREQ  1000
#define PWM_BITS  8
#define MAX_PWM   255

#define MAX_LINEAR_SPEED  0.30f
#define MAX_ANGULAR_SPEED 2.0f


// ============================================================
//                  ROBOT DIMENSIONS
// ============================================================

// Wheel diameter = 67 mm
#define WHEEL_DIAMETER  0.067f

// Wheel separation = 245 mm
#define WHEEL_BASE      0.245f

// Encoder counts per wheel revolution
#define ENCODER_CPR     662.0f


// ============================================================
//                  COMMUNICATION SETTINGS
// ============================================================

#define CMD_VEL_TIMEOUT_MS      500
#define ODOM_PUBLISH_PERIOD_MS  50
#define AGENT_CHECK_PERIOD_MS   500

// Time synchronization timeout
#define TIME_SYNC_TIMEOUT_MS    1000


// ============================================================
//                  ENCODER VARIABLES
// ============================================================

volatile long leftTicks = 0;
volatile long rightTicks = 0;


// ============================================================
//                  MICRO-ROS OBJECTS
// ============================================================

rcl_allocator_t allocator;

rclc_support_t support;

rcl_node_t node;

rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;

rcl_publisher_t left_encoder_pub;
rcl_publisher_t right_encoder_pub;
rcl_publisher_t odom_pub;
rcl_publisher_t tf_pub;


// ============================================================
//                  ROS MESSAGES
// ============================================================

geometry_msgs__msg__Twist cmd_vel_msg;

std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;

nav_msgs__msg__Odometry odom_msg;

tf2_msgs__msg__TFMessage tf_msg;


// ============================================================
//                  ROBOT ODOMETRY
// ============================================================

float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;

long previousLeftTicks = 0;
long previousRightTicks = 0;


// ============================================================
//                  TIMING VARIABLES
// ============================================================

unsigned long lastCmdVelTime = 0;

unsigned long lastOdomPublishTime = 0;

unsigned long lastAgentCheckTime = 0;


// ============================================================
//                  TIME SYNCHRONIZATION
// ============================================================

bool time_synchronized = false;


// ============================================================
//                  MICRO-ROS STATE
// ============================================================

enum AgentState
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

AgentState agentState = WAITING_AGENT;


// ============================================================
//              MICRO-ROS INITIALIZATION FLAGS
// ============================================================

bool support_initialized = false;

bool node_initialized = false;

bool left_encoder_pub_initialized = false;

bool right_encoder_pub_initialized = false;

bool odom_pub_initialized = false;

bool tf_pub_initialized = false;

bool cmd_vel_sub_initialized = false;

bool executor_initialized = false;


// ============================================================
//                  ENCODER INTERRUPTS
// ============================================================

void IRAM_ATTR leftEncoderISR()
{
  int A = digitalRead(LEFT_ENCODER_A);
  int B = digitalRead(LEFT_ENCODER_B);

  if (A == B)
  {
    leftTicks++;
  }
  else
  {
    leftTicks--;
  }
}


void IRAM_ATTR rightEncoderISR()
{
  int A = digitalRead(RIGHT_ENCODER_A);
  int B = digitalRead(RIGHT_ENCODER_B);

  if (A == B)
  {
    rightTicks++;
  }
  else
  {
    rightTicks--;
  }
}


// ============================================================
//                       LEFT MOTOR
// ============================================================

void leftMotor(int pwm)
{
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0)
  {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);

    ledcWrite(LEFT_ENA, pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);

    ledcWrite(LEFT_ENA, -pwm);
  }
  else
  {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);

    ledcWrite(LEFT_ENA, 0);
  }
}


// ============================================================
//                       RIGHT MOTOR
// ============================================================

void rightMotor(int pwm)
{
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0)
  {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);

    ledcWrite(RIGHT_ENB, pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);

    ledcWrite(RIGHT_ENB, -pwm);
  }
  else
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);

    ledcWrite(RIGHT_ENB, 0);
  }
}


// ============================================================
//                     STOP MOTORS
// ============================================================

void stopMotors()
{
  leftMotor(0);
  rightMotor(0);
}


// ============================================================
//                   RESET ENCODERS
// ============================================================

void resetEncoders()
{
  noInterrupts();

  leftTicks = 0;
  rightTicks = 0;

  interrupts();

  previousLeftTicks = 0;
  previousRightTicks = 0;
}


// ============================================================
//                    CMD_VEL CALLBACK
// ============================================================

void cmdVelCallback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  float linear_x = msg->linear.x;

  float angular_z = msg->angular.z;


  // ----------------------------------------------------------
  // Safety timer
  // ----------------------------------------------------------

  lastCmdVelTime = millis();


  // ----------------------------------------------------------
  // Limit commands
  // ----------------------------------------------------------

  linear_x = constrain(
      linear_x,
      -MAX_LINEAR_SPEED,
      MAX_LINEAR_SPEED
  );

  angular_z = constrain(
      angular_z,
      -MAX_ANGULAR_SPEED,
      MAX_ANGULAR_SPEED
  );


  // ----------------------------------------------------------
  // Differential drive
  //
  // left  = v - omega * wheel_base / 2
  // right = v + omega * wheel_base / 2
  // ----------------------------------------------------------

  float leftVelocity =
      linear_x -
      (angular_z * WHEEL_BASE / 2.0f);

  float rightVelocity =
      linear_x +
      (angular_z * WHEEL_BASE / 2.0f);


  // ----------------------------------------------------------
  // Convert velocity to PWM
  // ----------------------------------------------------------

  int leftPWM =
      (int)(
          (leftVelocity / MAX_LINEAR_SPEED)
          * MAX_PWM
      );

  int rightPWM =
      (int)(
          (rightVelocity / MAX_LINEAR_SPEED)
          * MAX_PWM
      );


  leftPWM =
      constrain(leftPWM, -MAX_PWM, MAX_PWM);

  rightPWM =
      constrain(rightPWM, -MAX_PWM, MAX_PWM);


  // ----------------------------------------------------------
  // Drive motors
  // ----------------------------------------------------------

  leftMotor(leftPWM);

  rightMotor(rightPWM);
}


// ============================================================
//              INITIALIZE ODOM MESSAGE
// ============================================================

bool initializeOdomMessage()
{
  if (!nav_msgs__msg__Odometry__init(&odom_msg))
  {
    return false;
  }


  // ----------------------------------------------------------
  // Header frame
  // ----------------------------------------------------------

  odom_msg.header.frame_id.data =
      (char *)"odom";

  odom_msg.header.frame_id.size = 4;

  odom_msg.header.frame_id.capacity = 5;


  // ----------------------------------------------------------
  // Child frame
  // ----------------------------------------------------------

  odom_msg.child_frame_id.data =
      (char *)"base_footprint";

  odom_msg.child_frame_id.size = 14;

  odom_msg.child_frame_id.capacity = 15;


  return true;
}


// ============================================================
//                INITIALIZE TF MESSAGE
// ============================================================

bool initializeTFMessage()
{
  if (!tf2_msgs__msg__TFMessage__init(&tf_msg))
  {
    return false;
  }


  if (!geometry_msgs__msg__TransformStamped__Sequence__init(
        &tf_msg.transforms,
        1))
  {
    return false;
  }


  // ----------------------------------------------------------
  // Parent frame
  // ----------------------------------------------------------

  tf_msg.transforms.data[0].header.frame_id.data =
      (char *)"odom";

  tf_msg.transforms.data[0].header.frame_id.size = 4;

  tf_msg.transforms.data[0].header.frame_id.capacity = 5;


  // ----------------------------------------------------------
  // Child frame
  // ----------------------------------------------------------

  tf_msg.transforms.data[0].child_frame_id.data =
      (char *)"base_footprint";

  tf_msg.transforms.data[0].child_frame_id.size = 14;

  tf_msg.transforms.data[0].child_frame_id.capacity = 15;


  return true;
}


// ============================================================
//              SYNCHRONIZE ESP32 CLOCK
// ============================================================

bool synchronizeTime()
{
  Serial.println();
  Serial.println(
      "Synchronizing ESP32 time with micro-ROS Agent..."
  );


  if (rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS)
      != RMW_RET_OK)
  {
    Serial.println(
        "ERROR: Time synchronization failed"
    );

    time_synchronized = false;

    return false;
  }


  if (!rmw_uros_epoch_synchronized())
  {
    Serial.println(
        "ERROR: Epoch is NOT synchronized"
    );

    time_synchronized = false;

    return false;
  }


  time_synchronized = true;


  Serial.println(
      "Time synchronization successful."
  );

  return true;
}


// ============================================================
//                CREATE MICRO-ROS ENTITIES
// ============================================================

bool createEntities()
{
  Serial.println();
  Serial.println(
      "Creating micro-ROS entities..."
  );


  // ----------------------------------------------------------
  // Allocator
  // ----------------------------------------------------------

  allocator =
      rcl_get_default_allocator();


  // ----------------------------------------------------------
  // Support
  // ----------------------------------------------------------

  if (rclc_support_init(
        &support,
        0,
        NULL,
        &allocator) != RCL_RET_OK)
  {
    Serial.println(
        "ERROR: rclc_support_init failed"
    );

    return false;
  }

  support_initialized = true;


  // ----------------------------------------------------------
  // Node
  // ----------------------------------------------------------

  if (rclc_node_init_default(
        &node,
        "nexva_esp32",
        "",
        &support) != RCL_RET_OK)
  {
    Serial.println(
        "ERROR: node initialization failed"
    );

    return false;
  }

  node_initialized = true;


  // ----------------------------------------------------------
  // /cmd_vel subscriber
  // ----------------------------------------------------------

  if (rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            geometry_msgs,
            msg,
            Twist),
        "/cmd_vel") != RCL_RET_OK)
  {
    Serial.println(
        "ERROR: /cmd_vel subscriber failed"
    );

    return false;
  }

  cmd_vel_sub_initialized = true;


  // ----------------------------------------------------------
  // /enco/left publisher
  // ----------------------------------------------------------

  if (rclc_publisher_init_default(
        &left_encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Int32),
        "/enco/left") != RCL_RET_OK)
  {
    Serial.println(
        "ERROR: /enco/left publisher failed"
    );

    return false;
  }

  left_encoder_pub_initialized = true;


  // ----------------------------------------------------------
  // /enco/right publisher
  // ----------------------------------------------------------

  if (rclc_publisher_init_default(
        &right_encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Int32),
        "/enco/right") != RCL_RET_OK)
  {
    Serial.println(
        "ERROR: /enco/right publisher failed"
    );

    return false;
  }

  right_encoder_pub_initialized = true;


  // ----------------------------------------------------------
  // /odom publisher
  // ----------------------------------------------------------

  if (rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            nav_msgs,
            msg,
            Odometry),
        "/odom") != RCL_RET_OK)
  {
    Serial.println(
        "ERROR: /odom publisher failed"
    );

    return false;
  }

  odom_pub_initialized = true;


  // ----------------------------------------------------------
  // /tf publisher
  // ----------------------------------------------------------

  if (rclc_publisher_init_default(
        &tf_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            tf2_msgs,
            msg,
            TFMessage),
        "/tf") != RCL_RET_OK)
  {
    Serial.println(
        "ERROR: /tf publisher failed"
    );

    return false;
  }

  tf_pub_initialized = true;


  // ----------------------------------------------------------
  // Initialize messages
  // ----------------------------------------------------------

  if (!initializeOdomMessage())
  {
    Serial.println(
        "ERROR: Odometry message initialization failed"
    );

    return false;
  }


  if (!initializeTFMessage())
  {
    Serial.println(
        "ERROR: TF message initialization failed"
    );

    return false;
  }


  // ----------------------------------------------------------
  // TIME SYNCHRONIZATION
  // ----------------------------------------------------------

  if (!synchronizeTime())
  {
    Serial.println(
        "ERROR: Cannot synchronize time"
    );

    return false;
  }


  // ----------------------------------------------------------
  // Executor
  // ----------------------------------------------------------

  if (rclc_executor_init(
        &executor,
        &support.context,
        1,
        &allocator) != RCL_RET_OK)
  {
    Serial.println(
        "ERROR: executor initialization failed"
    );

    return false;
  }

  executor_initialized = true;


  if (rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        &cmdVelCallback,
        ON_NEW_DATA) != RCL_RET_OK)
  {
    Serial.println(
        "ERROR: executor subscription failed"
    );

    return false;
  }


  // ----------------------------------------------------------
  // Reset odometry starting point
  // ----------------------------------------------------------

  noInterrupts();

  previousLeftTicks = leftTicks;

  previousRightTicks = rightTicks;

  interrupts();


  odom_x = 0.0f;

  odom_y = 0.0f;

  odom_theta = 0.0f;


  lastCmdVelTime = millis();

  lastOdomPublishTime = millis();


  Serial.println();
  Serial.println(
      "micro-ROS entities created successfully."
  );

  Serial.println(
      "ESP32 time synchronized."
  );

  return true;
}


// ============================================================
//                DESTROY MICRO-ROS ENTITIES
// ============================================================

void destroyEntities()
{
  Serial.println(
      "Destroying micro-ROS entities..."
  );


  // ----------------------------------------------------------
  // Executor
  // ----------------------------------------------------------

  if (executor_initialized)
  {
    rclc_executor_fini(&executor);

    executor_initialized = false;
  }


  // ----------------------------------------------------------
  // TF publisher
  // ----------------------------------------------------------

  if (tf_pub_initialized)
  {
    rcl_publisher_fini(
        &tf_pub,
        &node
    );

    tf_pub_initialized = false;
  }


  // ----------------------------------------------------------
  // Odom publisher
  // ----------------------------------------------------------

  if (odom_pub_initialized)
  {
    rcl_publisher_fini(
        &odom_pub,
        &node
    );

    odom_pub_initialized = false;
  }


  // ----------------------------------------------------------
  // Right encoder publisher
  // ----------------------------------------------------------

  if (right_encoder_pub_initialized)
  {
    rcl_publisher_fini(
        &right_encoder_pub,
        &node
    );

    right_encoder_pub_initialized = false;
  }


  // ----------------------------------------------------------
  // Left encoder publisher
  // ----------------------------------------------------------

  if (left_encoder_pub_initialized)
  {
    rcl_publisher_fini(
        &left_encoder_pub,
        &node
    );

    left_encoder_pub_initialized = false;
  }


  // ----------------------------------------------------------
  // cmd_vel subscriber
  // ----------------------------------------------------------

  if (cmd_vel_sub_initialized)
  {
    rcl_subscription_fini(
        &cmd_vel_sub,
        &node
    );

    cmd_vel_sub_initialized = false;
  }


  // ----------------------------------------------------------
  // Node
  // ----------------------------------------------------------

  if (node_initialized)
  {
    rcl_node_fini(&node);

    node_initialized = false;
  }


  // ----------------------------------------------------------
  // Support
  // ----------------------------------------------------------

  if (support_initialized)
  {
    rclc_support_fini(&support);

    support_initialized = false;
  }


  // ----------------------------------------------------------
  // Free TF sequence
  // ----------------------------------------------------------

  if (tf_msg.transforms.data != NULL)
  {
    geometry_msgs__msg__TransformStamped__Sequence__fini(
        &tf_msg.transforms
    );
  }


  tf_msg.transforms.data = NULL;

  tf_msg.transforms.size = 0;

  tf_msg.transforms.capacity = 0;


  time_synchronized = false;


  Serial.println(
      "micro-ROS entities destroyed."
  );
}


// ============================================================
//              UPDATE ODOMETRY FROM ENCODERS
// ============================================================

void updateOdometry()
{
  long currentLeftTicks;

  long currentRightTicks;


  // ----------------------------------------------------------
  // Read encoder values safely
  // ----------------------------------------------------------

  noInterrupts();

  currentLeftTicks = leftTicks;

  currentRightTicks = rightTicks;

  interrupts();


  // ----------------------------------------------------------
  // Tick difference
  // ----------------------------------------------------------

  long deltaLeftTicks =
      currentLeftTicks -
      previousLeftTicks;

  long deltaRightTicks =
      currentRightTicks -
      previousRightTicks;


  previousLeftTicks =
      currentLeftTicks;

  previousRightTicks =
      currentRightTicks;


  // ----------------------------------------------------------
  // Wheel circumference
  // ----------------------------------------------------------

  const float wheelCircumference =
      PI * WHEEL_DIAMETER;


  // ----------------------------------------------------------
  // Distance per encoder tick
  // ----------------------------------------------------------

  const float distancePerTick =
      wheelCircumference /
      ENCODER_CPR;


  // ----------------------------------------------------------
  // Individual wheel distance
  // ----------------------------------------------------------

  float leftDistance =
      deltaLeftTicks *
      distancePerTick;

  float rightDistance =
      deltaRightTicks *
      distancePerTick;


  // ----------------------------------------------------------
  // Robot center displacement
  // ----------------------------------------------------------

  float centerDistance =
      (leftDistance + rightDistance)
      / 2.0f;


  // ----------------------------------------------------------
  // Robot angular displacement
  // ----------------------------------------------------------

  float deltaTheta =
      (rightDistance - leftDistance)
      / WHEEL_BASE;


  // ----------------------------------------------------------
  // Midpoint integration
  // ----------------------------------------------------------

  float midpointTheta =
      odom_theta +
      (deltaTheta / 2.0f);


  odom_x +=
      centerDistance *
      cos(midpointTheta);

  odom_y +=
      centerDistance *
      sin(midpointTheta);

  odom_theta += deltaTheta;


  // ----------------------------------------------------------
  // Normalize angle
  // ----------------------------------------------------------

  while (odom_theta > PI)
  {
    odom_theta -= 2.0f * PI;
  }

  while (odom_theta < -PI)
  {
    odom_theta += 2.0f * PI;
  }
}


// ============================================================
//                  PUBLISH ENCODERS
// ============================================================

void publishEncoders()
{
  long left;

  long right;


  noInterrupts();

  left = leftTicks;

  right = rightTicks;

  interrupts();


  left_encoder_msg.data =
      (int32_t)left;

  right_encoder_msg.data =
      (int32_t)right;


  rcl_publish(
      &left_encoder_pub,
      &left_encoder_msg,
      NULL
  );


  rcl_publish(
      &right_encoder_pub,
      &right_encoder_msg,
      NULL
  );
}


// ============================================================
//                 PUBLISH ODOMETRY + TF
// ============================================================

void publishOdometry()
{
  // ----------------------------------------------------------
  // Do not publish timestamped odom/tf until clock sync
  // ----------------------------------------------------------

  if (!time_synchronized)
  {
    return;
  }


  // ----------------------------------------------------------
  // Update encoder odometry
  // ----------------------------------------------------------

  updateOdometry();


  // ----------------------------------------------------------
  // Get synchronized ROS epoch time
  // ----------------------------------------------------------

  int64_t time_ns =
      rmw_uros_epoch_nanos();


  if (time_ns <= 0)
  {
    return;
  }


  int32_t sec =
      (int32_t)(
          time_ns /
          1000000000LL
      );

  uint32_t nanosec =
      (uint32_t)(
          time_ns %
          1000000000LL
      );


  // ----------------------------------------------------------
  // ODOM timestamp
  // ----------------------------------------------------------

  odom_msg.header.stamp.sec =
      sec;

  odom_msg.header.stamp.nanosec =
      nanosec;


  // ----------------------------------------------------------
  // TF timestamp
  // ----------------------------------------------------------

  tf_msg.transforms.data[0]
      .header.stamp.sec =
      sec;

  tf_msg.transforms.data[0]
      .header.stamp.nanosec =
      nanosec;


  // ----------------------------------------------------------
  // Position
  // ----------------------------------------------------------

  odom_msg.pose.pose.position.x =
      odom_x;

  odom_msg.pose.pose.position.y =
      odom_y;

  odom_msg.pose.pose.position.z =
      0.0;


  // ----------------------------------------------------------
  // Orientation quaternion
  // yaw → quaternion
  // ----------------------------------------------------------

  float halfYaw =
      odom_theta / 2.0f;


  float sinHalfYaw =
      sin(halfYaw);

  float cosHalfYaw =
      cos(halfYaw);


  odom_msg.pose.pose.orientation.x =
      0.0;

  odom_msg.pose.pose.orientation.y =
      0.0;

  odom_msg.pose.pose.orientation.z =
      sinHalfYaw;

  odom_msg.pose.pose.orientation.w =
      cosHalfYaw;


  // ----------------------------------------------------------
  // TF translation
  // ----------------------------------------------------------

  tf_msg.transforms.data[0]
      .transform.translation.x =
      odom_x;

  tf_msg.transforms.data[0]
      .transform.translation.y =
      odom_y;

  tf_msg.transforms.data[0]
      .transform.translation.z =
      0.0;


  // ----------------------------------------------------------
  // TF rotation
  // ----------------------------------------------------------

  tf_msg.transforms.data[0]
      .transform.rotation.x =
      0.0;

  tf_msg.transforms.data[0]
      .transform.rotation.y =
      0.0;

  tf_msg.transforms.data[0]
      .transform.rotation.z =
      sinHalfYaw;

  tf_msg.transforms.data[0]
      .transform.rotation.w =
      cosHalfYaw;


  // ----------------------------------------------------------
  // Publish odometry
  // ----------------------------------------------------------

  rcl_publish(
      &odom_pub,
      &odom_msg,
      NULL
  );


  // ----------------------------------------------------------
  // Publish TF
  // ----------------------------------------------------------

  rcl_publish(
      &tf_pub,
      &tf_msg,
      NULL
  );
}


// ============================================================
//                  SAFETY TIMEOUT
// ============================================================

void checkCmdVelTimeout()
{
  if (
      millis() -
      lastCmdVelTime >
      CMD_VEL_TIMEOUT_MS
     )
  {
    stopMotors();
  }
}


// ============================================================
//                 WAIT FOR MICRO-ROS AGENT
// ============================================================

void waitForAgent()
{
  if (
      millis() -
      lastAgentCheckTime <
      AGENT_CHECK_PERIOD_MS
     )
  {
    return;
  }


  lastAgentCheckTime =
      millis();


  Serial.println(
      "Checking micro-ROS Agent..."
  );


  if (
      rmw_uros_ping_agent(100, 1)
      == RMW_RET_OK
     )
  {
    Serial.println(
        "micro-ROS Agent detected!"
    );

    agentState =
        AGENT_AVAILABLE;
  }
  else
  {
    Serial.println(
        "Agent not available."
    );
  }
}


// ============================================================
//                         SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(500);


  // ========================================================
  // MOTOR SETUP
  // ========================================================

  pinMode(
      LEFT_IN1,
      OUTPUT
  );

  pinMode(
      LEFT_IN2,
      OUTPUT
  );

  pinMode(
      RIGHT_IN1,
      OUTPUT
  );

  pinMode(
      RIGHT_IN2,
      OUTPUT
  );


  // --------------------------------------------------------
  // ESP32 Arduino Core 3.x LEDC
  // --------------------------------------------------------

  ledcAttach(
      LEFT_ENA,
      PWM_FREQ,
      PWM_BITS
  );

  ledcAttach(
      RIGHT_ENB,
      PWM_FREQ,
      PWM_BITS
  );


  stopMotors();


  // ========================================================
  // ENCODER SETUP
  // ========================================================

  pinMode(
      LEFT_ENCODER_A,
      INPUT_PULLUP
  );

  pinMode(
      LEFT_ENCODER_B,
      INPUT_PULLUP
  );

  pinMode(
      RIGHT_ENCODER_A,
      INPUT_PULLUP
  );

  pinMode(
      RIGHT_ENCODER_B,
      INPUT_PULLUP
  );


  // --------------------------------------------------------
  // Encoder interrupts
  // --------------------------------------------------------

  attachInterrupt(
      digitalPinToInterrupt(
          LEFT_ENCODER_A
      ),
      leftEncoderISR,
      CHANGE
  );


  attachInterrupt(
      digitalPinToInterrupt(
          RIGHT_ENCODER_A
      ),
      rightEncoderISR,
      CHANGE
  );


  resetEncoders();


  // ========================================================
  // MICRO-ROS SERIAL TRANSPORT
  // ========================================================

  set_microros_transports();


  // ========================================================
  // INITIAL STATE
  // ========================================================

  lastCmdVelTime =
      millis();

  lastOdomPublishTime =
      millis();

  lastAgentCheckTime =
      0;


  time_synchronized =
      false;


  agentState =
      WAITING_AGENT;


  // ========================================================
  // STARTUP INFORMATION
  // ========================================================

  Serial.println();

  Serial.println(
      "========================================"
  );

  Serial.println(
      "       NEXVA ESP32 MICRO-ROS"
  );

  Serial.println(
      "========================================"
  );


  Serial.println();

  Serial.println("Motor:");

  Serial.println(
      "  Left  IN1=26 IN2=25 ENA=27"
  );

  Serial.println(
      "  Right IN1=18 IN2=19 ENB=23"
  );


  Serial.println();

  Serial.println("Encoder:");

  Serial.println(
      "  Left  A=22 B=21"
  );

  Serial.println(
      "  Right A=32 B=33"
  );


  Serial.println();

  Serial.println("Robot:");

  Serial.println(
      "  Wheel diameter = 0.067 m"
  );

  Serial.println(
      "  Wheel base     = 0.245 m"
  );

  Serial.println(
      "  Encoder CPR    = 662"
  );


  Serial.println();

  Serial.println("Frames:");

  Serial.println(
      "  odom -> base_footprint"
  );


  Serial.println();

  Serial.println("Topics:");

  Serial.println(
      "  SUB  /cmd_vel"
  );

  Serial.println(
      "  PUB  /enco/left"
  );

  Serial.println(
      "  PUB  /enco/right"
  );

  Serial.println(
      "  PUB  /odom"
  );

  Serial.println(
      "  PUB  /tf"
  );


  Serial.println();

  Serial.println(
      "Waiting for micro-ROS Agent..."
  );

  Serial.println(
      "========================================"
  );
}


// ============================================================
//                          LOOP
// ============================================================

void loop()
{
  switch (agentState)
  {

    // ======================================================
    // WAITING FOR AGENT
    // ======================================================

    case WAITING_AGENT:

      stopMotors();

      waitForAgent();

      break;


    // ======================================================
    // AGENT FOUND
    // ======================================================

    case AGENT_AVAILABLE:

      if (createEntities())
      {
        Serial.println();

        Serial.println(
            "========================================"
        );

        Serial.println(
            "       MICRO-ROS CONNECTED"
        );

        Serial.println(
            "       TIME SYNCHRONIZED"
        );

        Serial.println(
            "========================================"
        );


        agentState =
            AGENT_CONNECTED;
      }
      else
      {
        Serial.println(
            "Failed to create micro-ROS entities."
        );


        destroyEntities();


        agentState =
            WAITING_AGENT;
      }

      break;


    // ======================================================
    // NORMAL OPERATION
    // ======================================================

    case AGENT_CONNECTED:
    {
      // ----------------------------------------------------
      // Process /cmd_vel
      // ----------------------------------------------------

      rclc_executor_spin_some(
          &executor,
          RCL_MS_TO_NS(5)
      );


      // ----------------------------------------------------
      // Motor safety
      // ----------------------------------------------------

      checkCmdVelTimeout();


      // ----------------------------------------------------
      // Encoder + odometry
      // ----------------------------------------------------

      if (
          millis() -
          lastOdomPublishTime >=
          ODOM_PUBLISH_PERIOD_MS
         )
      {
        lastOdomPublishTime =
            millis();


        publishEncoders();

        publishOdometry();
      }


      // ----------------------------------------------------
      // Check Agent connection
      // ----------------------------------------------------

      if (
          millis() -
          lastAgentCheckTime >=
          AGENT_CHECK_PERIOD_MS
         )
      {
        lastAgentCheckTime =
            millis();


        if (
            rmw_uros_ping_agent(100, 1)
            != RMW_RET_OK
           )
        {
          Serial.println();

          Serial.println(
              "micro-ROS Agent disconnected!"
          );


          stopMotors();


          agentState =
              AGENT_DISCONNECTED;
        }
      }


      break;
    }


    // ======================================================
    // AGENT DISCONNECTED
    // ======================================================

    case AGENT_DISCONNECTED:

      stopMotors();


      destroyEntities();


      Serial.println();

      Serial.println(
          "Waiting for micro-ROS Agent to reconnect..."
      );


      agentState =
          WAITING_AGENT;

      break;
  }


  delay(1);
}