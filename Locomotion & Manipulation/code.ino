#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;

// initialize Ultrasonic
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;

// Ultrasonic Max
const int MAX_DISTANCE = 200; // (200 cm/ 2 meters)

// Ultrasonic timing
unsigned long currentMillis;
unsigned long previousMillis;
const unsigned long US_PERIOD = 100;  // time to wait for the first US to activate

// current US distance reading
int distance = 0;

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // put your setup code here, to run once:
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT); // Corrected to OUTPUT

  delay(1000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:
  usReadCm();
}

void usReadCm() {
  currentMillis = millis(); // Corrected to millis()
  if (currentMillis - previousMillis >= US_PERIOD) {
    // Clears the TRIG_PIN (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Sets the TRIG_PIN HIGH (Active) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
    // note the duration (38000 microseconds) that will allow for reading up max distance supported by the sensor
    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2

    // Apply limits
    // if (distance > MAX_DISTANCE) Serial.print(" max distance");
    // if (distance == 0) Serial.print(" min distance");
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) distance = MAX_DISTANCE;

    Displays the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Update the previousMillis
    previousMillis = currentMillis;
  }
}
