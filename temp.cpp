#include "MeMCore.h"
MeBuzzer buzzer;
#define TURNING_TIME_MS 330 // The time duration (ms) for turning

#define TIMEOUT 2000       // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 12

#define input2A A0
#define input2B A1
#define irDetectorInterface A2
MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
MeDCMotor leftMotor(M1);           // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);          // assigning RightMotor to port M2
float readVal;
int delayTime = 200;
int sensorPin = 10;
float lineSensorOutput;

uint8_t motorSpeed = 255;
// Setting motor speed to an integer between 1 and 255
// The larger the number, the faster the speed

void loop()
{
}