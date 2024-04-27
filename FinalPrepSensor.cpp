#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;

unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2F;
float DIST_PER_TICK; // This will be calculated in setup

float Sl = 0.0F; // Distance traveled by left wheel
float Sr = 0.0F; // Distance traveled by right wheel

float x = 0.0F; // current x position
float y = 0.0F; // current y position
float theta = 0.0F; // current orientation in radians
const float WHEEL_BASE = 8.5F; // Distance between the wheels, adjust as needed

const int ECHO_PIN = 21;
const int TRIG_PIN = 22;
const int HEAD_SERVO_PIN = 11;

unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 150;

const int NUM_HEAD_POSITIONS= 1;
const int head_positions[num_head_positions] = {150}; // assuming wall is to the left

// Coordinates
const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30.0F, 15.0F, 0.0F};
float yGoals[NUMBER_OF_GOALS] = {30.0F, 60.0F, 0.0F};
int goalCounter = 0;
bool isAtTarget = false;

const float MOTOR_BASE_SPEED = 75.0;

void setup() {
    Serial.begin(57600);
    delay(1000);
    buzzer.play("c32");

    // Calculate DIST_PER_TICK based on the provided formula
    float wheelCircumference = PI * WHEEL_DIAMETER;
    DIST_PER_TICK = wheelCircumference / (CLICKS_PER_ROTATION * GEAR_RATIO);
}

void loop() {
    checkEncoders();
    
    if (!isAtTarget) {
        navigateToTarget();
    }
}

void checkEncoders() {

    currentMillis = millis();
    if (currentMillis - prevMillis >= PERIOD) {
        long newCountsLeft = encoders.getCountsLeft();
        long newCountsRight = encoders.getCountsRight();

        Serial.print("Encoder Counts - Left: ");
        Serial.print(newCountsLeft);
        Serial.print(", Right: ");
        Serial.println(newCountsRight);

        float dSl = (newCountsLeft - prevLeft) * DIST_PER_TICK;
        float dSr = (newCountsRight - prevRight) * DIST_PER_TICK;

        Serial.print("Distance Change - dSl: ");
        Serial.print(dSl);
        Serial.print(" cm, dSr: ");
        Serial.print(dSr);
        Serial.println(" cm");

        Sl += dSl;
        Sr += dSr;
        prevLeft = newCountsLeft;
        prevRight = newCountsRight;

        float dS = (dSr + dSl) / 2.0;
        float dTheta = (dSr - dSl) / WHEEL_BASE;

        theta += dTheta;

        float dx = dS * cos(theta + dTheta / 2.0);
        float dy = dS * sin(theta + dTheta / 2.0);

        x += dx;
        y += dy;

        Serial.print("Updated Position - X: ");
        Serial.print(x);
        Serial.print(" cm, Y: ");
        Serial.print(y);
        Serial.print(" cm, Theta: ");
        Serial.println(theta);

        prevMillis = currentMillis;
    }
}

void navigateToTarget() {
    if (goalCounter < NUMBER_OF_GOALS) {
        float distanceToTarget = sqrt(pow(yGoals[goalCounter] - x, 2) + pow(xGoals[goalCounter] - y, 2));
        Serial.print("Distance to Target: ");
        Serial.print(distanceToTarget);
        Serial.println(" cm");

        // Define the Proportional Gain for angular control
        float Kp_angle = 30.0; 

        if (distanceToTarget > 1.0) {
            float angleToTarget = atan2(xGoals[goalCounter] - y, yGoals[goalCounter] - x);
            float angleDifference = fmod(angleToTarget - theta + PI, 2 * PI) - PI;

            Serial.print("Angle to Target: ");
            Serial.print(angleToTarget);
            Serial.print(" rad, Current Theta: ");
            Serial.print(theta);
            Serial.print(" rad, Angle Difference: ");
            Serial.println(angleDifference);
            Serial.println(" radians");

            // Apply proportional control based on the angle difference
            float turnSpeed = (float)(Kp_angle * angleDifference);
            //turnSpeed = constrain(turnSpeed, -200, 200); // Constrain to max/min motor speed

            // Calculate motor speeds
            float leftSpeed = MOTOR_BASE_SPEED - turnSpeed;
            float rightSpeed = MOTOR_BASE_SPEED + turnSpeed;
            motors.setSpeeds(leftSpeed, rightSpeed);
            Serial.print("Turning with speed: ");
            Serial.println(turnSpeed);
                
            }
         else {
            motors.setSpeeds(0, 0);
            Serial.print("Reached Target: ");
            Serial.println(goalCounter);
            buzzer.play("c32");
            goalCounter++;  // Move to the next goal
            if (goalCounter >= NUMBER_OF_GOALS) {
                isAtTarget = true;
                buzzer.play("b32");
            }
            
        }
    }
}

void attractToGoal() {


}

void repulseFromObstacle() {

  if (currentObstacle > 10.0) {
    navigateToTarget();
  }


}
