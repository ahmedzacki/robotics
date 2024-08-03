#include <Pololu3piPlus32U4.h>
#include <Servo.h>


using namespace Pololu3piPlus32U4;


Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;


// ultrasonic values
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 250;
const int ECHO_PIN = 12;
const int TRIG_PIN = 4;
const float MAX_DISTANCE = 200.0;


// servo head vals
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 500;
const int HEAD_SERVO_PIN = 5;


const int NUM_HEAD_POSITIONS = 5;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = { 135, 112, 90, 67, 45 };
int currentHeadPosition = 0;
boolean headDirectionClockwise = true;


int readings[NUM_HEAD_POSITIONS] = { 100, 100, 100, 100, 100 };


float leftError = 0;
float rightError = 0;


// period vals
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;


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


float b = 8.40;  // distance between both wheels


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
const int NUMBER_OF_GOALS = 1;
float xGoals[NUMBER_OF_GOALS] = { 400 };
float yGoals[NUMBER_OF_GOALS] = { 100};


float distanceFromGoal = 0.0;  // how close we are to the goal


void setup() {
 // put your setup code here, to run once:


 pinMode(ECHO_PIN, INPUT);
 pinMode(TRIG_PIN, OUTPUT);
 Serial.begin(57600);
 delay(1000);
 buzzer.play("c32");


 headServo.attach(HEAD_SERVO_PIN);
}

void loop() {
 moveHead();


 obstaclePid();


 if (goalIndex == NUMBER_OF_GOALS)
   return;  // stopping everything if we did all the goals


 currentMillis = millis();


 if (currentMillis > prevMillis + PERIOD) {
   // getting encoder counts to calc distance traveled
   countsLeft += encoders.getCountsAndResetLeft();
   countsRight += encoders.getCountsAndResetRight();


   leftDistanceTraveled = (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO)) * WHEEL_CIRCUMFERENCE) * -1;     // cm
   rightDistanceTraveled = (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO)) * WHEEL_CIRCUMFERENCE) * -1;  // cm

   getVals();  // calculating changes in robot movement (x, y, theta)

   float error = getError();  // getting desired angle and calc error based on theta
                              //  Serial.print("Error: ");
                              //  Serial.println(error);

   float pid = calcPid(error);

   setMotors(pid);

   // updating prev vals
   prevLeft = countsLeft;
   prevRight = countsRight;
   prevMillis = currentMillis;
 }
}

// Moves the head of the servo between various angles that are stored in a global array.
void moveHead() {
 headCm = millis();


 if (headCm > headPm + HEAD_MOVEMENT_PERIOD) {
   headServo.write(HEAD_POSITIONS[currentHeadPosition]);
   delay(US_PERIOD);
   usReadCm();


   if (headDirectionClockwise) {
     if (currentHeadPosition >= (NUM_HEAD_POSITIONS - 1)) {
       headDirectionClockwise = !headDirectionClockwise;
       currentHeadPosition--;
     } else {
       currentHeadPosition++;
     }
   } else {
     if (currentHeadPosition <= 0) {
       headDirectionClockwise = !headDirectionClockwise;
       currentHeadPosition++;
     } else {
       currentHeadPosition--;
     }
   }


   headPm = headCm;
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
 float changeInS = (rightDistanceTraveled + leftDistanceTraveled) / 2;      // "cm"
 float changeInTheta = (rightDistanceTraveled - leftDistanceTraveled) / b;  // "radians ?"
 float thetaInRadians = theta * DEG_TO_RAD;


 float changeInX = changeInS * cos(thetaInRadians + (changeInTheta / 2));  // cm ?
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


 if (distanceFromGoal > 20) {  // keep going regular pid
   leftSpeed += pidValue;
   rightSpeed -= pidValue;


   leftSpeed += leftError;
   rightSpeed += rightError;

   if (leftSpeed > -50)
     leftSpeed = -50;
   if (rightSpeed > -50)
     rightSpeed = -50;


   motors.setSpeeds(leftSpeed, rightSpeed);
 } else {  // start slowing down
   if (distanceFromGoal > 20) {
     leftSpeed = -100 * (distanceFromGoal / 20);
     rightSpeed = -100 * (distanceFromGoal / 20);

     if (leftSpeed > -25)
       leftSpeed = -25;
     if (rightSpeed > -25)
       rightSpeed = -25;


     motors.setSpeeds(leftSpeed, rightSpeed);
   } else {
     motors.setSpeeds(0, 0);
     delay(1000);


     for (int i = 0; i < goalIndex + 1; i++) {
       buzzer.play("c45");
       delay(250);
     }


     goalIndex++;  // on to the NEXT GOAL!!!
   }
 }
 leftError = 0;
 rightError = 0;
}

// Gets distance reading from the ultrasonic sensor.
void usReadCm() {
 usCm = millis();

 digitalWrite(TRIG_PIN, LOW);
 delayMicroseconds(2);


 digitalWrite(TRIG_PIN, HIGH);
 delayMicroseconds(10);
 digitalWrite(TRIG_PIN, LOW);


 long duration = pulseIn(ECHO_PIN, HIGH, 38000);


 long distance = duration * 0.034 / 2;


 if (distance > MAX_DISTANCE)
   distance = MAX_DISTANCE;
 if (distance == 0)
   distance = MAX_DISTANCE;


 Serial.print("Distance: ");
 Serial.print(distance);
 Serial.println(" cm");


 readings[currentHeadPosition] = distance;
 usPm = usCm;
 //}
}

// Returns obstacle repulsion force.
float obstaclePid() {
 float outerProportion = 0.002;
 float innerProportion = 0.003;
 float middleProportion = 0.0035;


 for (int i = 0; i < NUM_HEAD_POSITIONS; i++) {
   int result = max(0, 125 - readings[i]);
   // Desides which proportion to apply depending on the position the servo head was in when it obtained the reading.
   if (i == 0 || i == 4) {
     if (i == 0) {
       leftError += (result * outerProportion);
       rightError -= (result * outerProportion);
     } else {
       leftError -= (result * outerProportion);
       rightError += (result * outerProportion);
     }
   } else if (i == 1 || i == 3) {
     if (i == 1) {
       leftError += (result * innerProportion);
       rightError -= (result * innerProportion);
     } else {
       leftError -= (result * innerProportion);
       rightError += (result * innerProportion);
     }
   } else {
     leftError -= (result * middleProportion);
     rightError += (result * middleProportion);
   }
 }
}
