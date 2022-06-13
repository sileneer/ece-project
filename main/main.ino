#include "MeMCore.h"

#define irDetectorInterface A2
#define ldrInterface A3
#define input2A A0
#define input2B A1

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

    // setBalance();            // calibration
    // digitalWrite(LED, HIGH); // Check Indicator -- ON after Calibration
}

void loop()
{

    // executeTurning(getColour()); // get the colour and execute the turning function

    int reading;

    Serial.println("Put the colour down ...");

    delay(2000);

    turnOnRed();
    delay(RGBWait);
    reading = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Red = ");
    Serial.println(reading);
    delay(2000);

    turnOnBlue();
    delay(RGBWait);
    reading = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Blue = ");
    Serial.println(reading);
    delay(2000);

    turnOnGreen();
    delay(RGBWait);
    reading = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Green = ");
    Serial.println(reading);
    delay(2000);

    // turnOnRed();
    // delay(2000);
    // turnOnBlue();
    // delay(2000);
    // turnOnGreen();
    // delay(2000);
}

void executeTurning(int detectedColour){
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

int getColour()
{

    return -1;
}

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

    void turnOnRed()
    {
        digitalWrite(input2A, HIGH);
        digitalWrite(input2B, HIGH);
    }

    void turnOnGreen()
    {
        digitalWrite(input2A, HIGH);
        digitalWrite(input2B, LOW);
    }

    void turnOnBlue()
    {
        digitalWrite(input2A, LOW);
        digitalWrite(input2B, HIGH);
    }
