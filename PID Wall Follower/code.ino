#include <Pololu3piPlus32U4.h>

#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;

Servo headServo;

Motors motors;

const double desiredState = (double) 30;

const double kp = 1.6;

const double ki = 0;

const double kd = 0.01;

double previousError = 0.0;

const float MOTOR_BASE_SPEED = 200.0;

const int MOTOR_MIN_SPEED = 90;

const int ECHO_PIN = 12;

const int TRIG_PIN = 4;

const float MAX_DISTANCE = 200.0;

unsigned long usCm;

unsigned long usPm;

const unsigned long US_PERIOD = 80;

bool isLookingForward = false;

int numOfUsReadings = 0;

float distance = 0;

const int HEAD_SERVO_PIN = 5;

float pidResult = 0;

float leftSpeed = MOTOR_BASE_SPEED;

float rightSpeed = MOTOR_BASE_SPEED;

const int READINGS = 3;

void setup() {

 Serial.begin(56700);

 pinMode(ECHO_PIN, INPUT);

 pinMode(TRIG_PIN, OUTPUT);

 headServo.attach(HEAD_SERVO_PIN);

 headServo.write(0);

 delay(500);

 buzzer.play("c32");

}

void loop() {

 usReadCm();

 if (!isLookingForward) { // use regular pid when looking to the side

   if (numOfUsReadings >= 3) {

     double error = distance - desiredState ;

     double proportional = kp * error;

     float derivative = kd * (error - previousError);

     previousError = error;

     pidResult = proportional + derivative;

     setMotors();

     lookAhead();

     numOfUsReadings = 0;

   }

 } else { // use diff pid when looking forward s

   if (numOfUsReadings >= 3) {

     if (distance < 30) {

       double error = distance * -1;

       double proportional = 50 * error;

       pidResult = proportional;

       setMotors();

     }

     lookAhead();

     numOfUsReadings = 0;

   }

 }

}

void lookAhead() {

 if (isLookingForward) {

   headServo.write(0);

 } else {

   headServo.write(90);

 }

 isLookingForward = !isLookingForward;

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

   distance = duration * 0.034 / 2;

   numOfUsReadings++;

   if (distance > MAX_DISTANCE) {

     if (!isLookingForward) {

       buzzer.play("c32");

     }

     distance = MAX_DISTANCE;

   }

   if (distance == 0) distance = 25;

   if (!isLookingForward) {

     Serial.print("[SIDE] ");

   } else {

     Serial.print("[FORWARD] ");

   }

   Serial.print("Distance: ");

   Serial.print(distance);

   Serial.println(" cm");

   usPm = usCm;

 }

}

void setMotors() {

 leftSpeed = MOTOR_BASE_SPEED;

 rightSpeed = MOTOR_BASE_SPEED;

 leftSpeed -= pidResult;

 rightSpeed += pidResult;

 if (leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;

 if (rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;

 if (leftSpeed > 400) leftSpeed = 400;

 if (rightSpeed > 400) rightSpeed = 400;

 motors.setSpeeds(leftSpeed * -1, rightSpeed * -1);

}
