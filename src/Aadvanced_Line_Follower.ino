/*
 * Advanced PID Line Follower Robot
 * Optimized for speed, reliability, and smooth navigation.
 * 
 * Features:
 * - Proportional-Integral-Derivative (PID) line tracking
 * - Unified motor control system with active braking/reversing
 * - Dynamic line-loss recovery
 * - Sensor-based (delay-free) sharp 90-degree turn execution
 */

// ================= MOTOR PINS =================
const uint8_t ENA = 9;
const uint8_t IN1 = 4;
const uint8_t IN2 = 5;
const uint8_t IN3 = 2;
const uint8_t IN4 = 3;
const uint8_t ENB = 10;

// ================= SENSOR PINS =================
const uint8_t S1 = A0;  // Far left
const uint8_t S2 = A1;  // Left
const uint8_t S3 = A2;  // Center
const uint8_t S4 = A3;  // Right
const uint8_t S5 = A4;  // Far right

// ================= TUNING PARAMETERS =================
// Scaled up for higher precision math
const float Kp = 4.2;    
const float Ki = 0.002;  
const float Kd = 2.8;    

const int BASE_SPEED = 90;   
const int MAX_SPEED  = 130;  
const int TURN_SPEED = 110;  // Speed used during sharp pivots
const int LOST_SPEED = 60;   // Speed for searching when line is lost

// ================= PID & STATE VARIABLES =================
float error = 0;
float lastError = 0;
float integral = 0;

int lastDirection = 0;   // -1 = left, +1 = right, 0 = straight

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  Serial.begin(115200);
  
  setMotors(0, 0); // Ensure motors are off at startup
  delay(1000);     // Wait before starting
}

void loop() {
  // Read sensors (Assuming 1 = Black Line, 0 = White Background)
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  // Calculate weighted sum for error (multiplied by 10 for better precision)
  int sum = (-40 * s1) + (-20 * s2) + (0 * s3) + (20 * s4) + (40 * s5);
  int count = s1 + s2 + s3 + s4 + s5;

  // ---------------------------------------------------------
  // 1. Sharp 90° Turn Detection (Intersections / Hard Corners)
  // ---------------------------------------------------------
  if (count >= 3) {
    if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0) { 
      executeSharpTurn(-1); // Left turn
      return;
    }
    if (s2 == 0 && s3 == 1 && s4 == 1 && s5 == 1) { 
      executeSharpTurn(1);  // Right turn
      return;
    }
  }

  // ---------------------------------------------------------
  // 2. Line Lost Recovery (Memory-based search)
  // ---------------------------------------------------------
  if (count == 0) {
    if (lastDirection < 0) {
      setMotors(-LOST_SPEED, LOST_SPEED + 15); // Sweep left
    } else if (lastDirection > 0) {
      setMotors(LOST_SPEED + 15, -LOST_SPEED); // Sweep right
    } else {
      setMotors(LOST_SPEED, LOST_SPEED);       // Creep forward
    }
    return;
  }

  // ---------------------------------------------------------
  // 3. Standard PID Line Following
  // ---------------------------------------------------------
  error = (float)sum / count;

  integral += error;
  integral = constrain(integral, -1000, 1000); // Anti-windup safeguard

  float derivative = error - lastError;
  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  // Constrain speeds to allow braking/reversing on tight curves
  leftSpeed  = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);

  // Update memory states
  lastError = error;
  if (error > 5) lastDirection = 1;
  else if (error < -5) lastDirection = -1;
  else lastDirection = 0;
}

// ==========================================================
// CORE MOTOR DRIVER
// Handles forward, reverse, and braking automatically.
// Uses your original directional logic (IN3=LOW/IN4=HIGH is Forward)
// ==========================================================
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor Logic
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed; // Convert to positive for PWM
  }

  // Right Motor Logic
  if (rightSpeed >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    rightSpeed = -rightSpeed; // Convert to positive for PWM
  }

  analogWrite(ENA, constrain(leftSpeed, 0, 255));
  analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

// ==========================================================
// SENSOR-BASED SHARP TURN EXECUTION
// Replaces blind delays with active sensor polling
// direction: -1 for Left, 1 for Right
// ==========================================================
void executeSharpTurn(int direction) {
  // 1. Push past the intersection slightly to align pivot point
  setMotors(BASE_SPEED, BASE_SPEED);
  delay(40); 

  // 2. Initiate the pivot
  if (direction == -1) {
    setMotors(-TURN_SPEED, TURN_SPEED); // Spin Left
  } else {
    setMotors(TURN_SPEED, -TURN_SPEED); // Spin Right
  }
  
  // 3. Blind spot delay to clear the line we were just on
  delay(150); 

  // 4. Actively poll until the center sensor finds the new line
  while (digitalRead(S3) == 0) {
    // Keep spinning until the center sensor hits black
  }

  // 5. Hard brake to prevent overshooting
  setMotors(-direction * 50, direction * 50); 
  delay(30);
  
  // 6. Reset PID history to prevent erratic jerks post-turn
  integral = 0;
  lastError = 0;
}
