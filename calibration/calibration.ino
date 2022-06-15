#include "MeMCore.h"

#define INPUT_2A A0
#define INPUT_2B A1
#define IR_DETECTOR_INTERFACE A2
#define LDR_INTERFACE A3

#define LINE_SENSOR_INTERFACE 10
#define ULTRASONIC_INTERFACE 12

#define TURNING_TIME_MS 360 // The time duration (ms) for turning

#define TIMEOUT 2000       // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment

#define LDRWait 10  // in milliseconds
#define RGBWait 200 // in milliseconds

void setup()
{
    // begin serial communication
    Serial.begin(9600);
    pinMode(INPUT_2A, OUTPUT);
    pinMode(INPUT_2B, OUTPUT);

    pinMode(IR_DETECTOR_INTERFACE, INPUT);
    pinMode(LDR_INTERFACE, INPUT);
    pinMode(LINE_SENSOR_INTERFACE, INPUT);
}

void loop()
{
    getColour();
}

/* return:
 * 1: blue
 * 2: orange
 * 3: red
 * 4: green
 * 5: purple
 * 6: white
 */

void getColour()
{
    Serial.println("Put the colour down ...");

    int redValue = turnOnRed();
    delay(5);

    int blueValue = turnOnBlue();
    delay(5);

    int greenValue = turnOnGreen();
    delay(5);
}

int getAvgReading(int times)
{
    // find the average reading for the requested number of times of scanning LDR
    int reading;
    int total = 0;
    // take the reading as many times as requested and add them up
    for (int i = 0; i < times; i++)
    {
        reading = analogRead(LDR_INTERFACE);
        total = reading + total;
        delay(LDRWait);
    }
    // calculate the average and return it
    return total / times;
}

int turnOnRed()
{
    digitalWrite(INPUT_2A, HIGH);
    digitalWrite(INPUT_2B, HIGH);

    delay(RGBWait);
    int red = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Red = ");
    Serial.println(red);
    return red;
}

int turnOnBlue()
{
    digitalWrite(INPUT_2A, HIGH);
    digitalWrite(INPUT_2B, LOW);

    delay(RGBWait);
    int blue = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Blue = ");
    Serial.println(blue);
    return blue;
}

int turnOnGreen()
{
    digitalWrite(INPUT_2A, LOW);
    digitalWrite(INPUT_2B, HIGH);

    delay(RGBWait);
    int green = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Green = ");
    Serial.println(green);
    return green;
}