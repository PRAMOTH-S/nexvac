#include <Arduino.h>

// ============================================================
// MOTOR PINS
// ============================================================

// LEFT MOTOR
#define LEFT_IN1  26
#define LEFT_IN2  25
#define LEFT_ENA  27

// RIGHT MOTOR
#define RIGHT_IN1 18
#define RIGHT_IN2 19
#define RIGHT_ENB 23


// ============================================================
// ENCODER PINS
// ============================================================

// RIGHT ENCODER
#define RIGHT_ENCODER_A 32
#define RIGHT_ENCODER_B 33

// LEFT ENCODER
#define LEFT_ENCODER_A 22
#define LEFT_ENCODER_B 21


// ============================================================
// PWM
// ============================================================

#define PWM_FREQ 1000
#define PWM_BITS 8

int SPEED = 130;


// ============================================================
// ENCODER COUNTERS
// ============================================================

volatile long leftTicks = 0;
volatile long rightTicks = 0;


// ============================================================
// LEFT ENCODER ISR
// ============================================================

void IRAM_ATTR leftEncoderISR()
{
  int A = digitalRead(LEFT_ENCODER_A);
  int B = digitalRead(LEFT_ENCODER_B);

  if (A == B)
    leftTicks++;
  else
    leftTicks--;
}


// ============================================================
// RIGHT ENCODER ISR
// ============================================================

void IRAM_ATTR rightEncoderISR()
{
  int A = digitalRead(RIGHT_ENCODER_A);
  int B = digitalRead(RIGHT_ENCODER_B);

  if (A == B)
    rightTicks++;
  else
    rightTicks--;
}


// ============================================================
// LEFT MOTOR
// ============================================================

void leftMotor(int pwm)
{
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
// RIGHT MOTOR
// ============================================================

void rightMotor(int pwm)
{
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
// STOP MOTORS
// ============================================================

void stopMotors()
{
  leftMotor(0);
  rightMotor(0);

  Serial.println("STOP");
}


// ============================================================
// RESET ENCODERS
// ============================================================

void resetEncoders()
{
  noInterrupts();

  leftTicks = 0;
  rightTicks = 0;

  interrupts();

  Serial.println("ENCODERS RESET");
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);


  // ==========================================================
  // MOTOR PINS
  // ==========================================================

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);


  // ==========================================================
  // PWM
  // ==========================================================

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


  // ==========================================================
  // ENCODER PINS
  // ==========================================================

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


  // ==========================================================
  // ENCODER INTERRUPTS
  // ==========================================================

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER_A),
    leftEncoderISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER_A),
    rightEncoderISR,
    CHANGE
  );


  // ==========================================================
  // RESET ENCODERS
  // ==========================================================

  resetEncoders();


  // ==========================================================
  // STOP MOTORS ON STARTUP
  // ==========================================================

  stopMotors();


  // ==========================================================
  // START MESSAGE
  // ==========================================================

  Serial.println();
  Serial.println("========================================");
  Serial.println("       NEXVA MOTOR + ENCODER TEST");
  Serial.println("========================================");

  Serial.println();

  Serial.println("Motor:");
  Serial.println("  Left  IN1=26 IN2=25 ENA=27");
  Serial.println("  Right IN1=18 IN2=19 ENB=23");

  Serial.println();

  Serial.println("Encoder:");
  Serial.println("  Left  A=22 B=21");
  Serial.println("  Right A=32 B=33");

  Serial.println();

  Serial.println("PWM Speed:");
  Serial.print("  SPEED = ");
  Serial.println(SPEED);

  Serial.println();

  Serial.println("Commands:");
  Serial.println("  f = Forward");
  Serial.println("  b = Backward");
  Serial.println("  l = Left");
  Serial.println("  r = Right");
  Serial.println("  c = Reset Encoders");
  Serial.println("  s = Stop");

  Serial.println("========================================");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // MOTOR / ENCODER COMMAND
  // ==========================================================

  if (Serial.available())
  {
    char command = Serial.read();


    // --------------------------------------------------------
    // Ignore newline
    // --------------------------------------------------------

    if (command == '\n' || command == '\r')
      return;


    // ========================================================
    // COMMAND SWITCH
    // ========================================================

    switch (command)
    {

      // ------------------------------------------------------
      // FORWARD
      // ------------------------------------------------------

      case 'f':
      case 'F':

        leftMotor(SPEED);
        rightMotor(SPEED);

        Serial.println("FORWARD");

        break;


      // ------------------------------------------------------
      // BACKWARD
      // ------------------------------------------------------

      case 'b':
      case 'B':

        leftMotor(-SPEED);
        rightMotor(-SPEED);

        Serial.println("BACKWARD");

        break;


      // ------------------------------------------------------
      // LEFT
      // ------------------------------------------------------

      case 'l':
      case 'L':

        leftMotor(-SPEED);
        rightMotor(SPEED);

        Serial.println("LEFT");

        break;


      // ------------------------------------------------------
      // RIGHT
      // ------------------------------------------------------

      case 'r':
      case 'R':

        leftMotor(SPEED);
        rightMotor(-SPEED);

        Serial.println("RIGHT");

        break;


      // ------------------------------------------------------
      // RESET ENCODERS
      // ------------------------------------------------------

      case 'c':
      case 'C':

        resetEncoders();

        break;


      // ------------------------------------------------------
      // STOP
      // ------------------------------------------------------

      case 's':
      case 'S':

        stopMotors();

        break;


      // ------------------------------------------------------
      // INVALID COMMAND
      // ------------------------------------------------------

      default:

        Serial.println(
          "Invalid command! Use f/b/l/r/c/s"
        );

        break;
    }
  }


  // ==========================================================
  // READ ENCODERS SAFELY
  // ==========================================================

  long left;
  long right;

  noInterrupts();

  left = leftTicks;
  right = rightTicks;

  interrupts();


  // ==========================================================
  // PRINT ENCODER VALUES
  // ==========================================================

  Serial.print("Left Encoder: ");
  Serial.print(left);

  Serial.print("    |    Right Encoder: ");
  Serial.println(right);


  // ==========================================================
  // LOOP DELAY
  // ==========================================================

  delay(100);
}