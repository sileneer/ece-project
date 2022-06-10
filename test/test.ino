#include "MeMCore.h"

// LDR 0 and LED13 (whats that)

int inputA = 2;
int inputB = 3;
int RGBWait = 1200; // in milliseconds
int LDRWait = 10;
#define LDR 0  // LDR sensor pin at A0
#define LED 13 // Check Indicator to signal Calibration Completed

// Define colour sensor LED pins
int ledArray[] = {2, 3, 4};

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

int total;

int getAvgReading(int times)
{
  // find the average reading for the requested number of times of scanning LDR
  int reading;
  total = 0;

  // take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++)
  {
    reading = analogRead(LDR);
    total = reading + total;
    delay(LDRWait);
  }
  // calculate the average and return it
  return total / times;
}

void setBalance()
{
  // set white balance
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);            // delay for five seconds for getting sample ready
  digitalWrite(LED, LOW); // Check Indicator OFF during Calibration
  // scan the white sample.
  // go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
  analogWrite(inputA, 255);
    analogWrite(inputB, 0);
  delay(RGBWait);
  whiteArray[0] = getAvgReading(5); // scan 5 times and return the average,
  analogWrite(inputB, 0);
  analogWrite(inputA, 255);
  delay(RGBWait);
  whiteArray[1] = getAvgReading(5); // scan 5 times and return the average,
  analogWrite(inputA, 255);
  analogWrite(inputA, 255);
  delay(RGBWait);
  whiteArray[2] = getAvgReading(5);
  // done scanning white, time for the black sample.
  // set black balance
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000); // delay for five seconds for getting sample ready
  // go through one colour at a time, set the minimum reading for red, green and blue to the black array
  digitalWrite(inputA, HIGH);
  digitalWrite(inputB, LOW);
  delay(RGBWait);
  whiteArray[0] = getAvgReading(5); // scan 5 times and return the average,
  digitalWrite(inputA, LOW);
  digitalWrite(inputB, HIGH);
  delay(RGBWait);
  whiteArray[1] = getAvgReading(5); // scan 5 times and return the average,
  digitalWrite(inputA, HIGH);
  digitalWrite(inputB, HIGH);
  delay(RGBWait);
  whiteArray[2] = getAvgReading(5);
  // the differnce between the maximum and the minimum gives the range
  for (int i = 0; i <= 2; i++)
  {
    greyDiff[i] = whiteArray[i] - blackArray[i];
  }
  // delay another 5 seconds for getting ready colour objects
  Serial.println("Colour Sensor Is Ready.");
  delay(5000);
  return;
}

void setup()
{
  pinMode(inputA, OUTPUT);
  pinMode(inputB, OUTPUT);
  Serial.begin(9600);
  setBalance();
}
void loop()
{
  // turn on one colour at a time and LDR reads 5 times
  for (int c = 0; c <= 2; c++)
  {
    Serial.print(colourStr[c]);
    analogWrite(inputA, 255);
    analogWrite(inputB, 0);
    delay(LDRWait);
    analogWrite(inputA, 0);
    analogWrite(inputB, 255);
    delay(LDRWait);
    analogWrite(inputA, 255);
    analogWrite(inputB, 255);
    delay(LDRWait);
    // get the average of 5 consecutive readings for the current colour and return an average
    colourArray[c] = getAvgReading(5);
    // the average reading returned minus the lowest value divided by the maximum possible range, multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity (i.e. the colour LDR is exposed to)
    colourArray[c] = (colourArray[c] - blackArray[c]) / (greyDiff[c]) * 255;
    digitalWrite(inputA, LOW); // turn off the current LED colour
    digitalWrite(inputB, LOW);
    delay(RGBWait);
    Serial.println(int(colourArray[c])); // show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
    digitalWrite(LED, HIGH);             // Check Indicator -- ON after Calibration
  }
}
