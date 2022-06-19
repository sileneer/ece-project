#include "MeMCore.h"

// define the pins
#define INPUT_2A A0
#define INPUT_2B A1
#define IR_DETECTOR_INTERFACE A2
#define LDR_INTERFACE A3

#define LINE_SENSOR_INTERFACE 10
#define ULTRASONIC_INTERFACE 12

// The time duration (ms) for turning
#define TURNING_TIME_MS 355

// Max microseconds to wait; choose according to max distance of wall
#define TIMEOUT 2000

// constant speed of the sound
#define SPEED_OF_SOUND 340

// define the waiting time for LDR and LED
#define LDRWait 10  // in milliseconds
#define RGBWait 200 // in milliseconds

// define buzzer, lineFinder, and motors
MeBuzzer buzzer;
MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
MeDCMotor leftMotor(M1);           // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);          // assigning RightMotor to port M2

// the output of the line sensor; HIGH when not detecting the black line, LOW when detecting the black line
float lineSensorOutput;

// the time to go straight when making two successtive turns
const int goStraightTime = 850;

// the maximum spped of the motor
const uint8_t motorSpeed = 255;

// the alignment of the mBot in the maze, according to the distance to the left and right wall
// -2: left, -1: slightly left, 0: center, 1: slightly right, 2: right, 3: extreme right
int alignment;

// the distance of mBot to the left and right wall
float distanceToLeft;
float distanceToRight;

/**
 * Identifier for each colour:
 * 1: blue
 * 2: orange
 * 3: red
 * 4: green
 * 5: purple
 * 6: white
 */

/**
 * @brief Make the robot turn left when detecting the red colour.
 *
 */
void redTurn()
{
    // Turning left (on the spot):
    leftMotor.run(motorSpeed);
    rightMotor.run(motorSpeed);
    delay(TURNING_TIME_MS);
    leftMotor.stop();
    rightMotor.stop();
    delay(10);
}

/**
 * @brief This function is used to make the robot turn right when detecting the green colour.
 *
 */
void greenTurn()
{
    // turning right on the spot
    leftMotor.run(-motorSpeed);
    rightMotor.run(-motorSpeed);
    delay(TURNING_TIME_MS);
    leftMotor.stop();
    rightMotor.stop();
    delay(10);
}

/**
 * @brief Make the robot make a 180 degree turn when detecting the blue colour.
 * @remark When the distance to the right wall is larger than 10cm, it will turn right first. Otherwise, it will turn left first.
 */
void orangeTurn()
{
    if (ultrasonicDistance() >= 10)
    {
        // 180 degree right turn within the same grid
        leftMotor.run(-motorSpeed + 15);
        rightMotor.run(-motorSpeed);
        delay(2 * TURNING_TIME_MS);
        leftMotor.stop();
        rightMotor.stop();
        delay(10);
    }
    else
    {
        // 180 degree left turn within the same grid
        leftMotor.run(motorSpeed - 15);
        rightMotor.run(motorSpeed);
        delay(2 * TURNING_TIME_MS);
        leftMotor.stop();
        rightMotor.stop();
        delay(10);
    }
}

/**
 * @brief Make the robot turn left successively in two grids when detecting the purple colour.
 *
 */
void purpleTurn()
{
    // two successive left turns in two grids
    // turn left
    leftMotor.run(motorSpeed);
    rightMotor.run(motorSpeed);
    delay(TURNING_TIME_MS);
    leftMotor.stop();
    rightMotor.stop();
    delay(5);
    // go forward
    leftMotor.run(-motorSpeed);
    rightMotor.run(motorSpeed);
    delay(goStraightTime + 100);
    leftMotor.stop();
    rightMotor.stop();
    delay(5);
    // turn left again
    leftMotor.run(motorSpeed);
    rightMotor.run(motorSpeed);
    delay(TURNING_TIME_MS + 20);
    leftMotor.stop();
    rightMotor.stop();
    delay(5);
}

/**
 * @brief Make the robot turn right successively in two grids when detecting the blue colour.
 *
 */
void blueTurn()
{
    // two successive right turns in two grids
    // turn right
    leftMotor.run(-motorSpeed);
    rightMotor.run(-motorSpeed);
    delay(TURNING_TIME_MS + 20);
    leftMotor.stop();
    rightMotor.stop();
    delay(5);
    // go forward
    leftMotor.run(-motorSpeed);
    rightMotor.run(motorSpeed);
    delay(goStraightTime);
    leftMotor.stop();
    rightMotor.stop();
    delay(5);
    // turn right again
    leftMotor.run(-motorSpeed);
    rightMotor.run(-motorSpeed);
    delay(TURNING_TIME_MS + 20);
    leftMotor.stop();
    rightMotor.stop();
    delay(5);
}

/**
 * @brief Play the sound of the buzzer when detecting the white colour.
 *
 * @return int
 */
void whiteFinish()
{
    celebrate();
    delay(1000000);
}

/**
 * @brief Detect the distance to the right wall using the ultrasonic sensor.
 *
 * @return float the distance to the wall in cm; -1 when the distance is not available
 */
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

/**
 * @brief Detect the distance to the left wall using a IR sensor.
 *
 * @return float the difference in the voltage with IR on and with IR off
 */
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

/**
 * @brief Make the robot go forward given the left and right motor speed.
 *
 * @param leftSpeed the speed of the left motor
 * @param rightSpeed the speed of the right motor
 */
void goForward(int leftSpeed, int rightSpeed)
{
    leftMotor.run(leftSpeed);
    rightMotor.run(rightSpeed);
    delay(30);
}

/**
 * @brief Execute the turning of mBot according to the colour detected. Called when the line sensor detects a black line.
 *
 * @param detectedColour 1: blue; 2: orange; 3: red; 4: green; 5: purple; 6: white
 */
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

/**
 * @brief Use the LDR to detect the colour of the paper below, and return the colour
 *
 * @remark The colour detected and the standard reference colours are stored in 1*3 matrices. The colours are then mapped into a 3d space. The smallest distances between the colour detected and the standard reference colour is used to compare and determine the colour.
 *
 * @return int 1: blue; 2: orange; 3: red; 4: green; 5: purple; 6: white
 */
int getColour()
{
    // Serial.println("Put the colour down ...");

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
    return minIndex + 1;
}

/**
 * @brief Calculate the distance between two colours in a 3d space.
 *
 * @param readings the coordinates of the colour to detected
 * @param calibratedReadings the coordinates of the standard reference colour
 * @return float the distance between these two colours in a 3d space
 */
float calculateDistance(int *readings, int *calibratedReadings)
{
    float distance = 0;
    for (int i = 0; i < 3; i++)
    {
        distance += pow(readings[i] - calibratedReadings[i], 2);
    }
    return sqrt(distance);
}

/**
 * @brief Get the average reading of the LDR
 *
 * @param times the number of readings
 * @return int the average reading of the LDR
 */
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

/**
 * @brief Turn on the red LED and read the LDR values
 *
 * @return int the reading of LDR when red LED is turned on
 */
int turnOnRed()
{
    digitalWrite(INPUT_2A, HIGH);
    digitalWrite(INPUT_2B, HIGH);

    delay(RGBWait);
    int red = getAvgReading(5); // scan 5 times and return the average,
    // Serial.print("Red = ");
    // Serial.println(red);
    return red;
}

/**
 * @brief Turn on the blue LED and read the LDR values
 *
 * @return int the reading of LDR when blue LED is turned on
 */
int turnOnBlue()
{
    digitalWrite(INPUT_2A, HIGH);
    digitalWrite(INPUT_2B, LOW);

    delay(RGBWait);
    int blue = getAvgReading(5); // scan 5 times and return the average,
    // Serial.print("Blue = ");
    // Serial.println(blue);
    return blue;
}

/**
 * @brief Turn on the green LED and read the LDR values
 *
 * @return int the reading of LDR when green LED is turned on
 */
int turnOnGreen()
{
    digitalWrite(INPUT_2A, LOW);
    digitalWrite(INPUT_2B, HIGH);

    delay(RGBWait);
    int green = getAvgReading(5); // scan 5 times and return the average,
    // Serial.print("Green = ");
    // Serial.println(green);
    return green;
}

/**
 * @brief Play the music
 *
 */
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

// -----------------------------setup and main loop---------------------------------

/**
 * @brief Set up mBot's pinMode and Serial Monitor
 *
 */
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

/**
 * @brief The main loop of the mBot; it is used to control the mBot by detecting the black line and the distance to the left and right wall.
 *
 */
void loop()
{

    // define the line sensor
    lineSensorOutput = digitalRead(LINE_SENSOR_INTERFACE);

    // when not detecting the black line
    if (lineSensorOutput == HIGH)
    {
        // initialise the alignment
        alignment = 0;

        // get the distance to the left and right wall
        distanceToLeft = irDistance();
        distanceToRight = ultrasonicDistance();

        // Serial.println("distanceToLeft: " + String(distanceToLeft));
        // Serial.println("distanceToRight: " + String(distanceToRight));

        // change the alignment of mBot in the maze, according to the IR sensor and ultrasonic sensor
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

        // Serial.println("alignment: " + String(alignment));

        // make the mBot go forward accroding to the current alignment
        switch (alignment)
        {
        case 0:
            goForward(-motorSpeed, motorSpeed);
            break;
        case 1:
            goForward(-motorSpeed + 20, motorSpeed);
            break;
        case 2:
            goForward(-motorSpeed + 50, motorSpeed);
            break;
        case 3:
            goForward(-motorSpeed + 60, motorSpeed);
            break;
        case -1:
            goForward(-motorSpeed, motorSpeed - 40);
            break;
        case -2:
            goForward(-motorSpeed, motorSpeed - 80);
            break;
        default:
            goForward(-motorSpeed, motorSpeed);
            break;
        }
    }

    // when detecting the black line
    else if (lineSensorOutput == LOW)
    {
        leftMotor.stop();  // Stop left motor
        rightMotor.stop(); // Stop right motor
        delay(10);

        // get the colour below the mBot
        int currentColour = getColour();
        // Serial.println(currentColour);

        // execute turning according to the colour detected
        executeTurning(getColour()); // get the colour and execute the turning function
    }
}