#include <Arduino.h>

// ===============================
// ENCODER PINS
// ===============================

#define RIGHT_ENCODER_A  32
#define RIGHT_ENCODER_B  33

#define LEFT_ENCODER_A 22
#define LEFT_ENCODER_B 21

// ===============================
// ENCODER COUNTERS
// ===============================

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ===============================
// LEFT ENCODER ISR
// ===============================

void IRAM_ATTR leftEncoderISR()
{
  int A = digitalRead(LEFT_ENCODER_A);
  int B = digitalRead(LEFT_ENCODER_B);

  if (A == B)
    leftTicks++;
  else
    leftTicks--;
}

// ===============================
// RIGHT ENCODER ISR
// ===============================

void IRAM_ATTR rightEncoderISR()
{
  int A = digitalRead(RIGHT_ENCODER_A);
  int B = digitalRead(RIGHT_ENCODER_B);

  if (A == B)
    rightTicks++;
  else
    rightTicks--;
}

// ===============================
// SETUP
// ===============================

void setup()
{
  Serial.begin(115200);

  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);

  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

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

  Serial.println();
  Serial.println("================================");
  Serial.println("     NEXVA ENCODER TEST");
  Serial.println("================================");
  Serial.println("Left  Encoder: A=32 B=33");
  Serial.println("Right Encoder: A=21 B=22");
  Serial.println();
}

// ===============================
// LOOP
// ===============================

void loop()
{
  long left;
  long right;

  noInterrupts();

  left = leftTicks;
  right = rightTicks;

  interrupts();

  Serial.print("Left Encoder  : ");
  Serial.print(left);

  Serial.print("    |    Right Encoder : ");
  Serial.println(right);

  delay(100);
}