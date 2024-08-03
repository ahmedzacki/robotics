#include <Pololu3piPlus32U4.h>


using namespace Pololu3piPlus32U4;


Encoders encoders;
Buzzer buzzer;
Motors motors;

// period vals
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 100;

// Motor constants
const float MOTOR_BASE_SPEED = -100;
const float MOTOR_MIN_SPEED = -90;

// motor vals
long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;


// Wheel constants
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;


// holding distance traveled per "reading"
float leftDistanceTraveled = 0.0F;
float rightDistanceTraveled = 0.0F;


float b = 8.40; // distance between both wheels


// PID vals
float kp = 1;

float ki = 0.0;
float errorSum = 0.0;

float prevError = 0.0;
float kd = 0.0;


// pose
float x = 0;
float y = 0;
float theta = 0;


// index of current goal
int goalIndex = 0;


// goals
const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};

float distanceFromGoal = 0.0; // how close we are to the goal

void setup() {
 // put your setup code here, to run once:
 Serial.begin(57600);
 delay(1000);
 buzzer.play("c32");
}

void loop() {
 if (goalIndex == NUMBER_OF_GOALS) return; // stopping everything if we did all the goals

 currentMillis = millis();

 if (currentMillis > prevMillis + PERIOD) {
   // getting encoder counts to calc distance traveled
   countsLeft += encoders.getCountsAndResetLeft(); 
   countsRight += encoders.getCountsAndResetRight();


   leftDistanceTraveled = (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO)) * WHEEL_CIRCUMFERENCE) * -1; // cm
   rightDistanceTraveled = (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO)) * WHEEL_CIRCUMFERENCE) * -1; // cm


   Serial.print("Left: ");
   Serial.println(leftDistanceTraveled);
   Serial.print("Right: ");
   Serial.println(rightDistanceTraveled);

   getVals(); // calculating changes in robot movement (x, y, theta)

   Serial.print("X: ");
   Serial.println(x);
   Serial.print("Y: ");
   Serial.println(y);
   Serial.print("Theta: ");
   Serial.println(theta);

   float error = getError(); // getting desired angle and calc error based on theta
   Serial.print("Error: ");
   Serial.println(error);

   float pid = calcPid(error);
   Serial.print("PID: ");
   Serial.println(pid);

   setMotors(pid);

   // updating prev vals
   prevLeft = countsLeft;
   prevRight = countsRight;
   prevMillis = currentMillis;
 }
}

/*
  Calculating:
  - changeInS
  - changeInTheta
  - changeInX
  - changeInY
*/
void getVals() {
 float changeInS = (rightDistanceTraveled + leftDistanceTraveled) / 2; // "cm"
 float changeInTheta = (rightDistanceTraveled - leftDistanceTraveled) / b; // "radians ?"
 float thetaInRadians = theta * DEG_TO_RAD;

 float changeInX = changeInS * cos(thetaInRadians + (changeInTheta / 2)); // cm ?
 float changeInY = changeInS * sin(thetaInRadians + (changeInTheta / 2));

 x += changeInX;
 y += changeInY;
 theta += (changeInTheta * RAD_TO_DEG);
}

// calculating error based on current theta and desired angle
float getError() {
 // getting the current goal coordinates
 float xGoal = xGoals[goalIndex];
 float yGoal = yGoals[goalIndex];

 // differences for atan2
 float xDifference = xGoal - x;
 float yDifference = yGoal - y;

 // calc and convert desired angle
 float goalOrientationRadians = atan2(yDifference, xDifference);
 float goalOrientationDegrees = goalOrientationRadians * RAD_TO_DEG;

 distanceFromGoal = sqrt(pow(xDifference, 2) + pow((y - yGoal), 2));

 // error = desiredState (atan result) - currentState (theta)
 float errorDegrees = goalOrientationDegrees - theta;


 return errorDegrees;
}


// regular pid for dealing with angle error
float calcPid(float error) {
 float proportional = kp * error;


 errorSum += error;
 float integral = ki * errorSum;


 float difference = error - prevError;
 float derivative = kd * difference;
 prevError = error;

 return proportional + integral + derivative;
}

// setting the motors using the pid value
void setMotors(float pidValue) {
 float leftSpeed = MOTOR_BASE_SPEED;
 float rightSpeed = MOTOR_BASE_SPEED;


 if (distanceFromGoal > 20) { // keep going regular pid
   leftSpeed += pidValue;
   rightSpeed -= pidValue;


   motors.setSpeeds(leftSpeed, rightSpeed);
 } else { // start slowing down
   if (distanceFromGoal > 5) {
     leftSpeed = -100 * (distanceFromGoal / 20);
     rightSpeed = -100 * (distanceFromGoal / 20);


     Serial.print("Left Speed: ");
     Serial.println(leftSpeed);
     Serial.print("Right Speeed: ");
     Serial.println(rightSpeed);


     if (leftSpeed > -25) leftSpeed = -25;
     if (rightSpeed > -25) rightSpeed = -25;


     motors.setSpeeds(leftSpeed, rightSpeed);
   } else {
     motors.setSpeeds(0, 0);
     delay(1000);
    
     for (int i = 0; i < goalIndex + 1; i++) {
       buzzer.play("c45");
       delay(250);
     }


     goalIndex++; // on to the NEXT GOAL!!!
   }

 }
}




