#include <Arduino.h>

constexpr uint8_t MOTOR_PWM_PIN = 5;

// Controls Direction, Stop & Coast, or Stop & Brake (depending on which one is HIGH and LOW at once)
constexpr uint8_t MOTOR_IN1_PIN = 7;
constexpr uint8_t MOTOR_IN2_PIN = 8;

// Must be in HIGH for motor driver to work
constexpr uint8_t MOTOR_STANDBY_PIN = 6;

constexpr uint8_t ENCODER_A_PIN = 2;
constexpr uint8_t ENCODER_B_PIN = 3;

constexpr uint8_t POTENTIOMETER_PIN = A0;

// 7 pole pairs * 2 edges * 50:1 gearbox
constexpr float ENCODER_COUNTS_PER_REV = 700.0;
constexpr float DEGREES_PER_ENCODER_COUNT = 360 / ENCODER_COUNTS_PER_REV;

constexpr unsigned long CONTROL_INTERVAL_MS = 10;
constexpr unsigned long DISPLAY_INTERVAL_MS = 50;

constexpr float DEG_TOLERANCE = 1.0;

volatile long EncoderCount = 0;
unsigned long EncoderCountSnapshot = 0;

float CurrentAngle = 0.0;
float TargetAngle = 0.0;

unsigned long PreviousControlTimeMs = 0;
unsigned long PreviousDisplayTimeMs = 0;

constexpr uint8_t Kp = 2;

void updateEncoder();

void setup() {

  Serial.begin(115200);

  pinMode(MOTOR_STANDBY_PIN, OUTPUT);
  // Turn off motor driver initially
  digitalWrite(MOTOR_STANDBY_PIN, LOW);

  pinMode(MOTOR_PWM_PIN, OUTPUT);

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);

  // Default HIGH
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A_PIN),
    updateEncoder,
    CHANGE
  );

}

void loop() {}

void updateEncoder() {
  bool encoderA = digitalRead(ENCODER_A_PIN);
  bool encoderB = digitalRead(ENCODER_B_PIN);

  if (encoderA == encoderB) {
    EncoderCount++;
  } else {
    EncoderCount--;
  }
}

