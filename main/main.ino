#include "MeMCore.h"

MeBuzzer buzzer;

MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
float readVal;
int delayTime = 30;
float lineSensorOutput;

#define input2A A0
#define input2B A1
#define irDetectorInterface A2
#define ldrInterface A3
#define lineSensorInterface 10

#define TURNING_TIME_MS 368 // The time duration (ms) for turning
MeDCMotor leftMotor(M1);    // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);   // assigning RightMotor to port M2
int goStraightTime = 900;
// from ultrasound code
#define TIMEOUT 2000       // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 12

const uint8_t leftMotorSpeed = 225;
const uint8_t rightMotorSpeed = 255;
const uint8_t motorSpeed = 255;

#define LDRWait 10  // in milliseconds
#define RGBWait 200 // in milliseconds

// Define colour sensor LED pins
int ledArray[] = {1, 2, 3}; // red is 2y3, blue is 2y2, green is 2y1

// placeholders for colour detected
int red = 0;
int green = 0;
int blue = 0;

// floats to hold colour arrays
float colourArray[] = {0, 0, 0};
float whiteArray[] = {0, 0, 0};
float blackArray[] = {0, 0, 0};
float greyDiff[] = {0, 0, 0};

char colourStr[3][5] = {"R = ", "G = ", "B = "};

// red: 3
void redTurn()
{
    // Turning left (on the spot):
    leftMotor.run(motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);     // Keep turning left for this time duration
    leftMotor.stop();           // Stop left motor
    rightMotor.stop();          // Stop right motor
    delay(1000);                // Stop for 1000 ms
}

// green: 4
void greenTurn()
{
    // turning right on the spot
    leftMotor.run(-motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);      // Keep turning left for this time duration
    leftMotor.stop();            // Stop left motor
    rightMotor.stop();           // Stop right motor
    delay(1000);                 // Stop for 1000 ms
}

// orange: 2
void orangeTurn()
{
    // 180 degree turn within the same grid
    leftMotor.run(-motorSpeed + 15); // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed);     // Positive: wheel turns clockwise
    delay(2 * TURNING_TIME_MS);      // Keep turning left for this time duration
    leftMotor.stop();                // Stop left motor
    rightMotor.stop();               // Stop right motor
    delay(1000);                     // Stop for 1000 ms
}

// purple: 5
void purpleTurn()
{
    // two successive left turns in two grids
    leftMotor.run(motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);     // Keep turning left for this time duration
    leftMotor.stop();           // Stop left motor
    rightMotor.stop();          // Stop right motor
    delay(100);                 // Stop for 1000 ms
    // go forward
    leftMotor.run(-motorSpeed); // Negative: wheel turns anti-clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(goStraightTime);      // Keep going straight for 1000 ms
    leftMotor.stop();           // Stop left motor
    rightMotor.stop();          // Stop right motor
    delay(100);                 // Stop for 1000 ms
    // turn left again
    leftMotor.run(motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);     // Keep turning left for this time duration
    leftMotor.stop();           // Stop left motor
    rightMotor.stop();          // Stop right motor
    delay(1000);                // Stop for 1000 ms
}

// blue: 1
void blueTurn()
{
    // two successive right turns in two grids
    leftMotor.run(-motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);      // Keep turning left for this time duration
    leftMotor.stop();            // Stop left motor
    rightMotor.stop();           // Stop right motor
    delay(100);                  // Stop for 1000 ms
    // go forward
    leftMotor.run(-motorSpeed); // Negative: wheel turns anti-clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(goStraightTime);      // Keep going straight for 1000 ms
    leftMotor.stop();           // Stop left motor
    rightMotor.stop();          // Stop right motor
    delay(100);                 // Stop for 1000 ms
    // turn left again
    leftMotor.run(-motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);      // Keep turning left for this time duration
    leftMotor.stop();            // Stop left motor
    rightMotor.stop();           // Stop right motor
    delay(1000);                 // Stop for 1000 ms
}

float ultrasonicDistance()
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
    delay(10);
}

float irDistance()
{
    digitalWrite(input2A, HIGH);
    digitalWrite(input2B, HIGH);
    delay(5);

    float initial_reading = analogRead(irDetectorInterface) * 5.0 / 1024.0;
    // Serial.print("off:");
    // Serial.println(initial_reading);
    delay(5);

    digitalWrite(input2A, LOW);
    digitalWrite(input2B, LOW);
    delay(5);

    float final_reading = analogRead(irDetectorInterface) * 5.0 / 1024.0;
    // Serial.print("on:");
    // Serial.println(final_reading);
    delay(5);
    return initial_reading - final_reading;
}

void goForward(int leftSpeed, int rightSpeed)
{
    leftMotor.run(leftSpeed);
    rightMotor.run(rightSpeed);
    delay(delayTime);
}

void setup()
{
    // // setup the outputs for the colour sensor
    // for (int c = 0; c <= 2; c++)
    // {
    //     pinMode(ledArray[c], OUTPUT);
    // }
    // pinMode(LED, OUTPUT); // Check Indicator -- OFF during Calibration

    // // begin serial communication
    Serial.begin(9600);
    pinMode(input2A, OUTPUT);
    pinMode(input2B, OUTPUT);

    pinMode(irDetectorInterface, INPUT);
    pinMode(lineSensorInterface, INPUT);

    // setBalance();            // calibration
    // digitalWrite(LED, HIGH); // Check Indicator -- ON after Calibration
}

void loop()
{

    // line sensor
    readVal = digitalRead(lineSensorInterface);

    if (readVal == HIGH)
    {

        int alignment = 0; // -1: left, 0: center, 1: right, 2: extreme right

        float distanceToLeft = irDistance();
        float distanceToRight = ultrasonicDistance();

        // Serial.println("distanceToLeft: " + String(distanceToLeft));
        // Serial.println("distanceToRight: " + String(distanceToRight));

        if (distanceToRight <= 10 && distanceToRight != -1)
        {
            alignment = 1;
            if (distanceToRight <= 8)
            {
                alignment = 2;
            }
        }
        if (distanceToLeft >= 2.77)
        {
            alignment = -1;
        }

        Serial.println("alignment: " + String(alignment));

        switch (alignment)
        {
        case 0:
            goForward(-leftMotorSpeed, rightMotorSpeed);
            break;
        case 1:
            goForward(-leftMotorSpeed + 20, rightMotorSpeed);
            break;
        case 2:
            goForward(-leftMotorSpeed + 50, rightMotorSpeed);
            break;
        case -1:
            goForward(-leftMotorSpeed, rightMotorSpeed - 30);
            break;
        default:
            goForward(-leftMotorSpeed, rightMotorSpeed);
            break;
        }
    }
    else if (readVal == LOW)
    {                      // If push button is pushed, the value will be very low
        leftMotor.stop();  // Stop left motor
        rightMotor.stop(); // Stop right motor
        delay(500);        // Delay 500ms so that a button push won't be counted multiple times.
        int currentColour = getColour();
        Serial.println(currentColour);
        executeTurning(getColour()); // get the colour and execute the turning function
    }
}

void executeTurning(int detectedColour)
{
    switch (detectedColour)
    {
    case 1:
        blueTurn();
        break;
    case 2:
        orangeTurn();
        break;
    case 3:
        redTurn();
        break;
    case 4:
        greenTurn();
        break;
    case 5:
        purpleTurn();
        break;
    default:
        Serial.println("No colour detected");
        break;
    }
}

/* return:
 * 1: blue
 * 2: orange
 * 3: red
 * 4: green
 * 5: purple
 * 6: white
 */

int getColour()
{
    Serial.println("Put the colour down ...");

    delay(200);

    int redValue = turnOnRed();
    delay(200);

    int blueValue = turnOnBlue();
    delay(200);

    int greenValue = turnOnGreen();
    delay(200);

    int readings[3] = {redValue, blueValue, greenValue};

    // blue, orange, red, green, purple
    int calibratedReadings[5][3] = {{869, 709, 918},
                                    {923, 755, 798},
                                    {920, 696, 784},
                                    {850, 673, 819},
                                    {878, 686, 878}};
    float results[5];
    for (int i = 0; i < 5; i++)
    {
        results[i] = calculateDistance(readings, calibratedReadings[i]);
    }

    int minIndex = 0;
    for (int i = 0; i < 5; i++)
    {
        if (results[i] < results[minIndex])
        {
            minIndex = i;
        }
    }
    return minIndex + 1;
}

float calculateDistance(int *readings, int *calibratedReadings)
{
    float distance = 0;
    for (int i = 0; i < 3; i++)
    {
        distance += pow(readings[i] - calibratedReadings[i], 2);
    }
    return sqrt(distance);
}

int getAvgReading(int times)
{
    // find the average reading for the requested number of times of scanning LDR
    int reading;
    int total = 0;
    // take the reading as many times as requested and add them up
    for (int i = 0; i < times; i++)
    {
        reading = analogRead(ldrInterface);
        total = reading + total;
        delay(LDRWait);
    }
    // calculate the average and return it
    return total / times;
}

int turnOnRed()
{
    digitalWrite(input2A, HIGH);
    digitalWrite(input2B, HIGH);

    delay(RGBWait);
    int red = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Red = ");
    Serial.println(red);
    return red;
}

int turnOnBlue()
{
    digitalWrite(input2A, HIGH);
    digitalWrite(input2B, LOW);

    delay(RGBWait);
    int blue = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Blue = ");
    Serial.println(blue);
    return blue;
}

int turnOnGreen()
{
    digitalWrite(input2A, LOW);
    digitalWrite(input2B, HIGH);

    delay(RGBWait);
    int green = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Green = ");
    Serial.println(green);
    return green;
}
