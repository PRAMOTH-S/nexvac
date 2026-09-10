#include <Arduino.h>

// ===============================
// MOTOR PINS
// ===============================

#define LEFT_IN1  26
#define LEFT_IN2  25
#define LEFT_ENA  27

#define RIGHT_IN1 18
#define RIGHT_IN2 19
#define RIGHT_ENB 23

// ===============================
// PWM
// ===============================

#define PWM_FREQ 1000
#define PWM_BITS 8

int SPEED = 150;


// ===============================
// LEFT MOTOR
// ===============================

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


// ===============================
// RIGHT MOTOR
// ===============================

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


// ===============================
// STOP
// ===============================

void stopMotors()
{
  leftMotor(0);
  rightMotor(0);

  Serial.println("STOP");
}


// ===============================
// SETUP
// ===============================

void setup()
{
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  ledcAttach(LEFT_ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(RIGHT_ENB, PWM_FREQ, PWM_BITS);

  stopMotors();

  Serial.println();
  Serial.println("================================");
  Serial.println("     NEXVA MOTOR TEST");
  Serial.println("================================");
  Serial.println("f = Forward");
  Serial.println("b = Backward");
  Serial.println("l = Left");
  Serial.println("r = Right");
  Serial.println("s = Stop");
  Serial.println("================================");
}


// ===============================
// LOOP
// ===============================

void loop()
{
  if (Serial.available())
  {
    char command = Serial.read();

    // Ignore newline
    if (command == '\n' || command == '\r')
      return;

    switch (command)
    {
      // ---------------------------
      // FORWARD
      // ---------------------------
      case 'f':
      case 'F':

        leftMotor(SPEED);
        rightMotor(SPEED);

        Serial.println("FORWARD");
        break;


      // ---------------------------
      // BACKWARD
      // ---------------------------
      case 'b':
      case 'B':

        leftMotor(-SPEED);
        rightMotor(-SPEED);

        Serial.println("BACKWARD");
        break;


      // ---------------------------
      // LEFT
      // ---------------------------
      case 'l':
      case 'L':

        leftMotor(-SPEED);
        rightMotor(SPEED);

        Serial.println("LEFT");
        break;


      // ---------------------------
      // RIGHT
      // ---------------------------
      case 'r':
      case 'R':

        leftMotor(SPEED);
        rightMotor(-SPEED);

        Serial.println("RIGHT");
        break;


      // ---------------------------
      // STOP
      // ---------------------------
      case 's':
      case 'S':

        stopMotors();
        break;


      default:

        Serial.println("Invalid command! Use f/b/l/r/s");
        break;
    }
  }
}