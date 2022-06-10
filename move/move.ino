#include "MeMCore.h"

#define TURNING_TIME_MS 330 // The time duration (ms) for turning

#define TIMEOUT 2000       // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 12

MeDCMotor leftMotor(M1);  // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2

uint8_t motorSpeed = 255;
// Setting motor speed to an integer between 1 and 255
// The larger the number, the faster the speed

void setup()
{
    // Any setup code here runs only once:
    delay(2000);        // Do nothing for 10000 ms = 10 seconds
    Serial.begin(9600); // to initialize the serial monitor
}

float ultrasonic()
{
    pinMode(ULTRASONIC, OUTPUT);
    digitalWrite(ULTRASONIC, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC, LOW);

    pinMode(ULTRASONIC, INPUT);
    long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
    if (duration > 0)
    {
        return (duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100);
    }
    else
    {
        return -1;
    }
    delay(500);
}

float irDistance(){
    // read the voltage of the IR sensor and return the distance in cm
    float voltage = analogRead(A2);
//    float distance = (voltage * 5.0) / 1024.0;
    return voltage;
}

void goForward(int leftSpeed, int rightSpeed)
{
    leftMotor.run(leftSpeed);
    rightMotor.run(rightSpeed);
    delay(150);
    leftMotor.stop();  // Stop left motor
    rightMotor.stop(); // Stop right motor
    delay(1000);
}

void loop()
{
    // // The main code here will run repeatedly (i.e., looping):
    // // Going forward:
    // leftMotor.run(-motorSpeed); // Negative: wheel turns anti-clockwise
    // rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    // delay(1000);                // Keep going straight for 1000 ms
    // leftMotor.stop();           // Stop left motor
    // rightMotor.stop();          // Stop right motor
    // delay(1000);                // Stop for 1000 ms
    // float distanceToRight = ultrasonic();
    // if (distanceToRight != -1)
    // {
    //     if (distanceToRight <= 4)
    //     {
    //         goForward(-230, 255);
    //     }
    //     else if (distanceToRight >= 12)
    //     {
    //         goForward(-255, 230);
    //     }
    //     else
    //     {
    //         goForward(-255, 255);
    //     }
    // }
    Serial.println(irDistance());
    delay(500);
}
