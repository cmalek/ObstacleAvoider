#include <NewPing.h>
#include <Servo.h>
#include <Bounce2.h>
#include <Rolley.h>
#include <LSM303.h>
#include <Wire.h>
#include <ObstacleAvoider.h>

Servo servo;
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE);
rolley::ObstacleAvoider robot = rolley::ObstacleAvoider();

void setup() {
  robot.setup(&servo, &sonar);
  Serial.begin(9600);
}

void loop() {
  robot.run();
}
