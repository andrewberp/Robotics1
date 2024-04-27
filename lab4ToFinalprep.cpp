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

// Coordinates
const int NUMBER_OF_GOALS = 4;
float xGoals[NUMBER_OF_GOALS] = {-30.0F, -80.0F, 60.0F, 0.0F};
float yGoals[NUMBER_OF_GOALS] = {0.0F, 50.0F, -30.0F, 0.0F};
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
        float distanceToTarget = sqrt(pow(yGoals[goalCounter] - y, 2) + pow(xGoals[goalCounter] - x, 2));
        Serial.print("Distance to Target: ");
        Serial.print(distanceToTarget);
        Serial.println(" cm");

        // Define the Proportional Gain for angular control
        float Kp_angle = 30.0; 

        if (distanceToTarget > 4.0) {
            // Calculate angle to target, as usual
            float angleToTarget = atan2(yGoals[goalCounter] - y, xGoals[goalCounter] - x);
            float angleDifference = fmod(angleToTarget - theta + PI, 2 * PI) - PI;

            Serial.print("Angle to Target: ");
            Serial.print(angleToTarget);
            Serial.print(" rad, Current Theta: ");
            Serial.print(theta);
            Serial.print(" rad, Angle Difference: ");
            Serial.println(angleDifference);
            Serial.println(" radians");

            // Apply proportional control based on the angle difference
            float turnSpeed = Kp_angle * angleDifference;

            // Reverse motor speeds so positive values move the robot in the sensor's direction
            // If the robot's wheels are currently configured such that a positive value makes it move "forward" (away from the sensor),
            // we invert the values to make a positive value make it move "backward" (toward the sensor)
            float leftSpeed = -MOTOR_BASE_SPEED + turnSpeed;
            float rightSpeed = -MOTOR_BASE_SPEED - turnSpeed;

            motors.setSpeeds(leftSpeed, rightSpeed);
            Serial.print("Motor Speed - Left: ");
            Serial.print(leftSpeed);
            Serial.print(", Right: ");
            Serial.println(rightSpeed);
        } else {
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
