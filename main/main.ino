#include "MeMCore.h"

#define INPUT_2A A0
#define INPUT_2B A1
#define IR_DETECTOR_INTERFACE A2
#define LDR_INTERFACE A3

#define LINE_SENSOR_INTERFACE 10
#define ULTRASONIC_INTERFACE 12

#define TURNING_TIME_MS 355 // The time duration (ms) for turning

#define TIMEOUT 2000       // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment

#define LDRWait 10  // in milliseconds
#define RGBWait 200 // in milliseconds

MeBuzzer buzzer;
MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2

float lineSensorOutput;

MeDCMotor leftMotor(M1);  // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
int goStraightTime = 850;

const uint8_t leftMotorSpeed = 225;
const uint8_t rightMotorSpeed = 255;
const uint8_t motorSpeed = 255;

// red: 3
void redTurn()
{
    // Turning left (on the spot):
    leftMotor.run(motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);     // Keep turning left for this time duration
    leftMotor.stop();           // Stop left motor
    rightMotor.stop();          // Stop right motor
    delay(10);                  // Stop for 1000 ms
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
    delay(10);                   // Stop for 1000 ms
}

// orange: 2
void orangeTurn()
{
    if (ultrasonicDistance() >= 10)
    {
        // 180 degree turn within the same grid
        leftMotor.run(-motorSpeed + 15); // Positive: wheel turns clockwise
        rightMotor.run(-motorSpeed);     // Positive: wheel turns clockwise
        delay(2 * TURNING_TIME_MS);      // Keep turning left for this time duration
        leftMotor.stop();                // Stop left motor
        rightMotor.stop();               // Stop right motor
        delay(10);                       // Stop for 1000 ms
    }
    else
    {
        // 180 degree turn within the same grid
        leftMotor.run(motorSpeed - 15); // Positive: wheel turns clockwise
        rightMotor.run(motorSpeed);     // Positive: wheel turns clockwise
        delay(2 * TURNING_TIME_MS);     // Keep turning left for this time duration
        leftMotor.stop();               // Stop left motor
        rightMotor.stop();              // Stop right motor
        delay(10);                      // Stop for 1000 ms
    }
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
    delay(5);                   // Stop for 1000 ms
    // go forward
    leftMotor.run(-leftMotorSpeed);  // Negative: wheel turns anti-clockwise
    rightMotor.run(rightMotorSpeed); // Positive: wheel turns clockwise
    delay(goStraightTime + 100);     // Keep going straight for 1000 ms
    leftMotor.stop();                // Stop left motor
    rightMotor.stop();               // Stop right motor
    delay(5);                        // Stop for 1000 ms
    // turn left again
    leftMotor.run(motorSpeed);   // Positive: wheel turns clockwise
    rightMotor.run(motorSpeed);  // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS + 20); // Keep turning left for this time duration
    leftMotor.stop();            // Stop left motor
    rightMotor.stop();           // Stop right motor
    delay(5);                    // Stop for 1000 ms
}

// blue: 1
void blueTurn()
{
    // two successive right turns in two grids
    leftMotor.run(-motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS + 20); // Keep turning left for this time duration
    leftMotor.stop();            // Stop left motor
    rightMotor.stop();           // Stop right motor
    delay(5);                    // Stop for 1000 ms
    // go forward
    leftMotor.run(-leftMotorSpeed);  // Negative: wheel turns anti-clockwise
    rightMotor.run(rightMotorSpeed); // Positive: wheel turns clockwise
    delay(goStraightTime);           // Keep going straight for 1000 ms
    leftMotor.stop();                // Stop left motor
    rightMotor.stop();               // Stop right motor
    delay(5);                        // Stop for 1000 ms
    // turn left again
    leftMotor.run(-motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS + 20); // Keep turning left for this time duration
    leftMotor.stop();            // Stop left motor
    rightMotor.stop();           // Stop right motor
    delay(5);                    // Stop for 1000 ms
}

float ultrasonicDistance()
{
    pinMode(ULTRASONIC_INTERFACE, OUTPUT);
    digitalWrite(ULTRASONIC_INTERFACE, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_INTERFACE, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_INTERFACE, LOW);

    pinMode(ULTRASONIC_INTERFACE, INPUT);
    long duration = pulseIn(ULTRASONIC_INTERFACE, HIGH, TIMEOUT);
    if (duration > 0)
    {
        return (duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100);
    }
    else
    {
        return -1;
    }
}

float irDistance()
{
    digitalWrite(INPUT_2A, HIGH);
    digitalWrite(INPUT_2B, HIGH);
    delay(5);

    float initial_reading = analogRead(IR_DETECTOR_INTERFACE) * 5.0 / 1024.0;
    // Serial.print("off:");
    // Serial.println(initial_reading);
    delay(5);

    digitalWrite(INPUT_2A, LOW);
    digitalWrite(INPUT_2B, LOW);
    delay(5);

    float final_reading = analogRead(IR_DETECTOR_INTERFACE) * 5.0 / 1024.0;
    // Serial.print("on:");
    // Serial.println(final_reading);
    return initial_reading - final_reading;
}

void goForward(int leftSpeed, int rightSpeed)
{
    leftMotor.run(leftSpeed);
    rightMotor.run(rightSpeed);
    delay(30);
}

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

    // line sensor
    lineSensorOutput = digitalRead(LINE_SENSOR_INTERFACE);

    if (lineSensorOutput == HIGH)
    {

        int alignment = 0; // -2: extreme left, -1: left, 0: center, 1: right, 2: extreme right

        float distanceToLeft = irDistance();
        float distanceToRight = ultrasonicDistance();

        Serial.println("distanceToLeft: " + String(distanceToLeft));
        // Serial.println("distanceToRight: " + String(distanceToRight));

        if (distanceToRight <= 9 && distanceToRight != -1)
        {
            alignment = 1;
            if (distanceToRight <= 7)
            {
                alignment = 2;
                if (distanceToRight <= 6)
                {
                    alignment = 3;
                }
            }
        }
        if (distanceToLeft >= 3.70)
        {
            alignment = -1;
            if (distanceToLeft >= 3.73)
            {
                alignment = -2;
            }
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
            goForward(-leftMotorSpeed + 40, rightMotorSpeed);
            break;
        case 3:
            goForward(-leftMotorSpeed + 60, rightMotorSpeed);
            break;
        case -1:
            goForward(-leftMotorSpeed, rightMotorSpeed - 40);
            break;
        case -2:
            goForward(-leftMotorSpeed, rightMotorSpeed - 80);
            break;
        default:
            goForward(-leftMotorSpeed, rightMotorSpeed);
            break;
        }
    }
    else if (lineSensorOutput == LOW)
    {                      // If push button is pushed, the value will be very low
        leftMotor.stop();  // Stop left motor
        rightMotor.stop(); // Stop right motor
        delay(10);         // Delay 500ms so that a button push won't be counted multiple times.
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
    case 6:
        whiteFinish();
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

    int redValue = turnOnRed();
    delay(5);

    int blueValue = turnOnBlue();
    delay(5);

    int greenValue = turnOnGreen();
    delay(5);

    int readings[3] = {redValue, blueValue, greenValue};

    // blue, orange, red, green, purple, white
    int calibratedReadings[6][3] = {{911, 757, 932},
                                    {935, 776, 827},
                                    {931, 732, 823},
                                    {904, 735, 853},
                                    {914, 739, 900},
                                    {937, 822, 946}};
    float results[6];
    for (int i = 0; i < 6; i++)
    {
        results[i] = calculateDistance(readings, calibratedReadings[i]);
    }

    int minIndex = 0;
    for (int i = 0; i < 6; i++)
    {
        if (results[i] < results[minIndex])
        {
            minIndex = i;
        }
    }

    // if (minIndex == 1 || minIndex == 2)
    // {
    //     int orangeDifference = abs(readings[2] - calibratedReadings[1][2]);
    //     int redDifference = abs(readings[2] - calibratedReadings[2][2]);
    //     if (orangeDifference < redDifference)
    //     {
    //         return 2;
    //     }
    //     else
    //     {
    //         return 3;
    //     }
    // }
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

int whiteFinish()
{
    celebrate();
    delay(1000000);
}

void celebrate()
{
    // Each of the following "function calls" plays a single tone.
    // The numbers in the bracket specify the frequency and the duration (ms)

    int C = 261.6;
    int D = 293.7;
    int E = 329.6;
    int F = 349.2;
    int G = 392.0;
    int A = 440;
    int B = 493.9;
    int highC = 523.3;
    int highD = 587.3;
    int highE = 659.3;
    int highF = 698.5;
    int highG = 784;

    int yipai = 600; // 80 per second
    int banpai = yipai / 2;
    int semiquaver = yipai / 4;

    buzzer.tone(highC, yipai + banpai);
    buzzer.tone(highD, yipai * 2 + banpai);
    buzzer.tone(highD, yipai + banpai);
    buzzer.tone(highE, yipai + banpai);
    buzzer.tone(highG, semiquaver);
    buzzer.tone(highF, semiquaver);
    buzzer.tone(highE, banpai);
    buzzer.tone(highC, yipai + banpai);
    buzzer.tone(highD, yipai * 2 + banpai);
    buzzer.tone(highD, yipai + banpai);
    buzzer.tone(highC, yipai * 2 + banpai);

    buzzer.tone(G, semiquaver);
    buzzer.tone(A, semiquaver);
    buzzer.tone(highC, semiquaver);
    buzzer.tone(A, semiquaver);

    buzzer.tone(highE, semiquaver + banpai);
    buzzer.tone(highE, semiquaver + banpai);
    buzzer.tone(highD, banpai + yipai);

    buzzer.tone(G, semiquaver);
    buzzer.tone(A, semiquaver);
    buzzer.tone(highC, semiquaver);
    buzzer.tone(A, semiquaver);

    buzzer.tone(highD, semiquaver + banpai);
    buzzer.tone(highD, semiquaver + banpai);
    buzzer.tone(highC, banpai + semiquaver);
    buzzer.tone(B, semiquaver);
    buzzer.tone(A, banpai);

    buzzer.tone(G, semiquaver);
    buzzer.tone(A, semiquaver);
    buzzer.tone(highC, semiquaver);
    buzzer.tone(A, semiquaver);

    buzzer.tone(highC, yipai);
    buzzer.tone(highD, banpai);
    buzzer.tone(B, banpai + semiquaver);
    buzzer.tone(A, semiquaver);
    buzzer.tone(G, banpai + semiquaver);
    buzzer.tone(G, semiquaver);
    buzzer.tone(highD, yipai);
    buzzer.tone(highC, yipai * 3);
    buzzer.noTone();
}
