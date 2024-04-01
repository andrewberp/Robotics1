#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Motors motors;
Buzzer buzzer;
Servo headServo;

// Ultrasonic sensor pins
const int ECHO_PIN = 2;
const int TRIG_PIN = 3;

// Ultrasonic max distance
const float MAX_DISTANCE = 100;

// Ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 100;

// Current ultrasonic distance reading
float distance = 0;

// Debugging switch for the head movement
const boolean HEAD_DEBUG = true;

// Timing for head movement
unsigned long headCM;
unsigned long headPM;
const unsigned long HEAD_MOVEMENT_PERIOD = 500;

// Head servo constants
const int HEAD_SERVO_PIN = 12;
const int NUM_HEAD_POSITIONS = 2;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {120, 40}; 

// Head servo data
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

// Motor constants
const float MOTOR_BASE_SPEED = 100.0;

// Desired positions for PID control
const double desiredStateR = 30;
const double desiredStateS = 50;

// Motor timing
unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 150;

// PID constants for Right and Straight directions
const double kpR = 2, kdR = 2.5;
const double kpS = 2, kdS = 0;

double priorErrorR = 0.0, priorErrorS = 0.0;

// Head direction indicator (1 for Right, 0 for Straight)
int headDirection = 0;

void setup() {
  Serial.begin(57600);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(40);
  delay(3000);
  buzzer.play("c32");
}

void loop() {
  moveHead();
  usReadCm();
  setMotors();
}

void moveHead() {
  headCM = millis();
  if (headCM > headPM + HEAD_MOVEMENT_PERIOD) {
    if (HEAD_DEBUG) {
      Serial.print("Position: "); Serial.print(currentHeadPosition);
      Serial.print(" Direction: "); Serial.print(headDirection);
      Serial.print(" Angle: "); Serial.println(HEAD_POSITIONS[currentHeadPosition]);
    }
    headServo.write(HEAD_POSITIONS[currentHeadPosition]);
    updateHeadDirection();
    headPM = headCM;
  }
}

void usReadCm() {
  usCm = millis();
  if (usCm > usPm + US_PERIOD) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    distance = (duration * 0.034 / 2 > MAX_DISTANCE) ? MAX_DISTANCE : duration * 0.034 / 2;
    
    Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
    usPm = usCm;
  }
}

void setMotors() {
  motorCm = millis();
  if (motorCm > motorPm + MOTOR_PERIOD) {
    double error, proportional, derivative, pidOutput;
    if (headDirection == 1) { // Right direction PID control
      error = desiredStateR - distance;
      proportional = kpR * error;
      derivative = kdR * (error - priorErrorR);
      priorErrorR = error;
      pidOutput = proportional + derivative;
    } else { // Straight direction PID control
      error = desiredStateS - distance;
      proportional = kpS * error;
      derivative = kdS * (error - priorErrorS);
      priorErrorS = error;
      pidOutput = proportional + derivative;
    }

    // Adjust motor speeds based on PID output
    float leftSpeed = -(MOTOR_BASE_SPEED + pidOutput); // Negative for forward movement
    float rightSpeed = -(MOTOR_BASE_SPEED - pid Output); // Negative for forward movement

    Serial.print("PID Output: "); Serial.println(pidOutput);
    Serial.print("Left Speed: "); Serial.print(leftSpeed);
    Serial.print(" Right Speed: "); Serial.println(rightSpeed);
  
    // Apply the speed adjustments to the motors
    motors.setSpeeds(leftSpeed, rightSpeed);
  
    motorPm = motorCm;

  }
}

void updateHeadDirection() {
  // Toggle the head direction between 1 and 0
  if (headDirectionClockwise) {
    if (currentHeadPosition >= (NUM_HEAD_POSITIONS - 1)) {
      headDirectionClockwise = !headDirectionClockwise;
      currentHeadPosition--;
      headDirection = 0; // Assume 0 is for straight ahead
  } else {
    currentHeadPosition++;
    headDirection = 1; // Assume 1 is for the right side
  }
    } else {
      if (currentHeadPosition <= 0) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
        headDirection = 1; // Back to checking the right side
    } else {
      currentHeadPosition--;
        headDirection = 0; // Straight ahead
    }
  }
}


