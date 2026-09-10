#include <Arduino.h>

// ================================
// ENCODER PINS
// ================================

#define LEFT_ENCODER_A 32
#define LEFT_ENCODER_B 34

#define RIGHT_ENCODER_A 35
#define RIGHT_ENCODER_B 39

// ================================
// ENCODER COUNTERS
// ================================

volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;


// ================================
// LEFT ENCODER INTERRUPT
// ================================

void IRAM_ATTR leftEncoderISR()
{
  int b = digitalRead(LEFT_ENCODER_B);

  if (b == HIGH)
    left_encoder_ticks++;
  else
    left_encoder_ticks--;
}


// ================================
// RIGHT ENCODER INTERRUPT
// ================================

void IRAM_ATTR rightEncoderISR()
{
  int b = digitalRead(RIGHT_ENCODER_B);

  if (b == HIGH)
    right_encoder_ticks++;
  else
    right_encoder_ticks--;
}


// ================================
// SETUP
// ================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  // Encoder pins
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);

  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  // Interrupts
  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER_A),
    leftEncoderISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER_A),
    rightEncoderISR,
    RISING
  );

  Serial.println("=================================");
  Serial.println(" Nexva Encoder Test");
  Serial.println("=================================");
  Serial.println("Left Encoder  : GPIO 32 / 34");
  Serial.println("Right Encoder : GPIO 35 / 39");
  Serial.println();
}


// ================================
// LOOP
// ================================

void loop()
{
  long left_ticks;
  long right_ticks;

  // Safely read encoder counters
  noInterrupts();

  left_ticks = left_encoder_ticks;
  right_ticks = right_encoder_ticks;

  interrupts();

  // Print values
  Serial.print("Left Encoder: ");
  Serial.print(left_ticks);

  Serial.print("    Right Encoder: ");
  Serial.println(right_ticks);

  delay(100);
}