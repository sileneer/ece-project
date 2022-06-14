#include "MeMCore.h"

MeBuzzer buzzer;

MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
float readVal;
int delayTime = 50;
float lineSensorOutput;

#define input2A A0
#define input2B A1
#define irDetectorInterface A2
#define ldrInterface A3
#define lineSensorInterface 10

#define TURNING_TIME_MS 330 // The time duration (ms) for turning
MeDCMotor leftMotor(M1);    // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);   // assigning RightMotor to port M2
int goStraightTime = 1000;
// from ultrasound code
#define TIMEOUT 2000       // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 12

uint8_t motorSpeed = 255;

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

    pinMode(irDetectorInterface, INPUT);
    pinMode(lineSensorInterface, INPUT);

    // setBalance();            // calibration
    // digitalWrite(LED, HIGH); // Check Indicator -- ON after Calibration
}

void loop()
{
    int currentColour = getColour();
    Serial.println(currentColour);
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
    int calibratedReadings[5][3] = {{868, 923, 680},
                                    {925, 818, 720},
                                    {918, 812, 640},
                                    {850, 833, 629},
                                    {879, 885, 640}};
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
    digitalWrite(input2A, LOW);
    digitalWrite(input2B, HIGH);

    delay(RGBWait);
    int blue = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Blue = ");
    Serial.println(blue);
    return blue;
}

int turnOnGreen()
{
    digitalWrite(input2A, HIGH);
    digitalWrite(input2B, LOW);

    delay(RGBWait);
    int green = getAvgReading(5); // scan 5 times and return the average,
    Serial.print("Green = ");
    Serial.println(green);
    return green;
}
