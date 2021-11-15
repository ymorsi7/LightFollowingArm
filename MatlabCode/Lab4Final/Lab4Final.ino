/**
 * ECE 5 SP20 Lab 4 Light Following Arm Main Program
 * Controls a servo with photoresistors tracking LEDs in a row
 * written by Scott Zhao, Katie Hsieh, and Karcher Morris 
 * under GNU General Public License
 *
 * NOTE: Must be used together with provided MATLAB side software
 *
 * Copyright (C) 2020 Scott Zhao <scott@scottz.net>
 *                    https://github.com/zhaomh1998
 * 
 * Copyright (C) 2020 Katie Hsieh <kjhsieh@ucsd.edu>
 * 
 * Copyright (C) 2016 Karcher Morris <k6morris@eng.ucsd.edu>
 *
 * Version 20200529
 */

#include <Servo.h>
#include "MATLABInterface.h"
#define SERVO_TICK_MIN_MS 10     // For Algorithm 2 in updateServo()
#define SERVO_TICK_MAX_MS 50   // For Algorithm 2 in updateServo()

// Pin Definitions
const int NUM_OF_PHOTORESISTORS = 5;
const int NUM_OF_LEDS = 7;
const int SERVO_MAX_LIMIT = 35;
const short photoresistorPin[] = {A0, A1, A2, A3, A4};
const short ledPin[] = {6, 7, 8, 9, 10, 11, 12};
const short servoPin = 3;

bool autoLED = true;
int ledOnIndex = 3;
int dir = 1;

int currentPRLight[NUM_OF_PHOTORESISTORS];
int minPRLight[NUM_OF_PHOTORESISTORS];
int maxPRLight[NUM_OF_PHOTORESISTORS];
float error;
const float LOST = -10;         // A special value for program itself
const int LOST_THRESHOLD = 40;  // Mapped value < this indicates lost

// New Vars for controls
float turn = 0;
float turnDecimals = 0;
float kP = 5.0;
float kI = 1.0 / 10.0;
float kD = 1.0;
float sumError = 0;
float lastError = 0;
int currentPosition = 90; // After calibration, we will move to middle

Servo myServo;      // create servo object to control a servo

void calibrate() {
  // Turn on one LED, turn servo around and read max & min for each photoresistor
  // Then in readLight, map the readings with these calibrations
  Serial.println("Calibration started!");
  // 1. Initialize Arrays
  for (int i = 0; i < NUM_OF_PHOTORESISTORS; i++) {
    minPRLight[i] = 1023;
    maxPRLight[i] = 0;
    currentPRLight[i] = 0;
  }
  digitalWrite(ledPin[ledOnIndex], HIGH);
  myServo.write(SERVO_MAX_LIMIT);
  delay(1000);
  for (int pos = SERVO_MAX_LIMIT; pos <= 180 - SERVO_MAX_LIMIT; pos++) {
    myServo.write(pos);
    delay(15);
    for (int i = 0; i < NUM_OF_PHOTORESISTORS; i++) {
      int PRLight = analogRead(photoresistorPin[i]);
      if (PRLight < 50) {
        Serial.println("Warning! Reads < 50");
        delay(1000);
      }
      if (PRLight > maxPRLight[i])
        maxPRLight[i] = PRLight;
      if (PRLight < minPRLight[i])
        minPRLight[i] = PRLight;
    }
  }

  for (int pos = 180 - SERVO_MAX_LIMIT; pos >= SERVO_MAX_LIMIT; pos--) {
    myServo.write(pos);
    delay(15);
    for (int i = 0; i < NUM_OF_PHOTORESISTORS; i++) {
      int PRLight = analogRead(photoresistorPin[i]);
      if (PRLight < 50) {
        Serial.println("Warning! Reads < 50");
        delay(1000);
      }
      if (PRLight > maxPRLight[i])
        maxPRLight[i] = PRLight;
      if (PRLight < minPRLight[i])
        minPRLight[i] = PRLight;
    }
  }

  Serial.println("Calibration Finished!");
  Serial.println("/------Max Readings------\\");
  for (int i = 0; i < NUM_OF_PHOTORESISTORS; i++) {
    Serial.print(maxPRLight[i]); Serial.print(' ');
  }
  Serial.println();
  Serial.println("-------Min Readings-------");
  for (int i = 0; i < NUM_OF_PHOTORESISTORS; i++) {
    Serial.print(minPRLight[i]); Serial.print(' ');
  }
  Serial.println();
  Serial.println("\\------------------------/");
  // Go to center after calibration
  myServo.write(90);
  delay(3000);
}

void readLight() {
  for (short i = 0; i < NUM_OF_PHOTORESISTORS; i++) {
    currentPRLight[i] = map(analogRead(photoresistorPin[i]), minPRLight[i], maxPRLight[i], 0, 100);
  }
  calcError();
}

// Perform Weighted Average
void calcError() {
  float MxRead = -99;
  float AveRead = 0.0;
  float WeightedAve, CriteriaForMax;
  int MxIndex, im0, im1, im2;
  int centerIndex = NUM_OF_PHOTORESISTORS / 2;
  // If every photoresistor reads < LOST_THRESHOLD, set error to LOST indicating lost light
  for (short ii = 0; ii < NUM_OF_PHOTORESISTORS; ii++) {
    if (currentPRLight[ii] >= LOST_THRESHOLD)
      break;
    else if (ii == NUM_OF_PHOTORESISTORS - 1) {
      error = LOST;
      sumError = 0;
      lastError = 0;
      return;
    }
  }
  // 1. Iterate through photoresistor mapped values, find darkest/max (MxRead), it's index (im1) and weighted index (MxIndex)
  //    Weighted Index: from left to right: 1 - 0 (center) - 1
  for (short ii = 0; ii < NUM_OF_PHOTORESISTORS; ii++)
  {
    if (MxRead < currentPRLight[ii])
    {
      MxRead = currentPRLight[ii];
      MxIndex = -1 * (ii - 1);
      im1 = (float)ii;
    }
    AveRead = AveRead + (float)currentPRLight[ii] / NUM_OF_PHOTORESISTORS;
  }

  // 2. Calculate error from weighted average, based off the readings around the maximum value
  CriteriaForMax = 1.2;  // max should be at least this times as big as the other values
  if (MxRead > CriteriaForMax * AveRead)  // only when the max is big enough
  {
    if (im1 != 0 && im1 != NUM_OF_PHOTORESISTORS - 1) // max not on either ends
    {
      im0 = im1 - 1;  // index for left
      im2 = im1 + 1;  // index for right
      if (currentPRLight[im0] + currentPRLight[im1] + currentPRLight[im2] == 0)  // if the denominator calculates to 0, jump out and do not update error
        return;
      WeightedAve = ((float)(currentPRLight[im0] * im0 + currentPRLight[im1] * im1 + currentPRLight[im2] * im2)) / ((float)(currentPRLight[im0] + currentPRLight[im1] + currentPRLight[im2]));
      error = -1 * (WeightedAve - centerIndex);
    }
    else if (im1 == 0)  // max on left end
    {
      im2 = im1 + 1;
      if (currentPRLight[im1] + currentPRLight[im2] == 0)  // if the denominator calculates to 0, jump out and do not update error
        return;
      WeightedAve = ((float)(currentPRLight[im1] * im1 + currentPRLight[im2] * im2)) / ((float)(currentPRLight[im1] + currentPRLight[im2]));
      error = -1 * (WeightedAve - centerIndex);
    }
    else if (im1 == NUM_OF_PHOTORESISTORS - 1) // max on right end
    {
      im0 = im1 - 1;
      if (currentPRLight[im0] + currentPRLight[im1] == 0)  // if the denominator calculates to 0, jump out and do not update error
        return;
      WeightedAve = ((float)(currentPRLight[im0] * im0 + currentPRLight[im1] * im1)) / ((float)(currentPRLight[im0] + currentPRLight[im1]));
      error = -1 * (WeightedAve - centerIndex);
    }
  }
  else {
    Serial.println("WARNING: Can't determine a max! Error Calculation Skipped!");
  }
}

void calcPID() {
  // error holds values from -3 to 3
  // If we are lost
  if (error == LOST) {
    turn = LOST;
    return;
  }
  // Else, perform PID
  turn = error * kP + sumError * kI + (error - lastError) * kD; //PID!!!!!

  sumError = sumError + error;
  // Cap sumError (integral) at +- 5
  if (sumError > 5) {
    sumError = 5;
  }
  else if (sumError < -5) {
    sumError = -5;
  }

  lastError = error;
}

void delayNonBlock(unsigned long duration) {
  unsigned long startTime = millis();
  while(millis() - startTime < duration) {
    readLight();
    handleMatlab();
  }
}

void updateServo() {
  if (turn == LOST)
    findWayBack();

  // The servo only accept integer angles
  // Store truncated decimals in calculated "turn" and accumulate them

  turnDecimals += turn - (int) turn;
  int turnSpeed = turn + (int) turnDecimals;
  turnDecimals -= (int) turnDecimals;
  // Make sure our destination is within range
  int destination = min(max(currentPosition + turnSpeed, SERVO_MAX_LIMIT), 180-SERVO_MAX_LIMIT);

  // Algorithm 1: Set destination angle = current + turnSpeed. Then constant delay to let servo go
  myServo.write(destination);
  delayNonBlock(100);
  currentPosition = destination;

  // Algorithm 2: Move 1 degree (1 tick) each iteration, and use turnSpeed to determine
  //              how long to wait until next iteration
  // short direction =  destination > currentPosition ? 1 : -1;
  // int delayAmount = map(turnSpeed, 0, 10, SERVO_TICK_MAX_MS, SERVO_TICK_MIN_MS);
  // delayAmount = max(SERVO_TICK_MIN_MS, min(SERVO_TICK_MAX_MS, delayAmount));  // Cap at [min, max]
  // while(currentPosition != destination) {
  //   currentPosition += direction;
  //   myServo.write(currentPosition);
  //   delayNonBlock(delayAmount);
  // }
}

void findWayBack() {
  Serial.println("We are lost. Attempting to find our way back");
  currentPosition = SERVO_MAX_LIMIT;
  myServo.write(currentPosition);
  delayNonBlock(1000);
  for (; currentPosition <= 180 - SERVO_MAX_LIMIT; currentPosition++) {
    myServo.write(currentPosition);
    delayNonBlock(15);
    readLight();
    if (error != LOST) { // We found our way back!
      Serial.println("We are back!");
      calcPID();
      return;
    }
  }

  for (; currentPosition >= SERVO_MAX_LIMIT; currentPosition--) {
    myServo.write(currentPosition);
    delayNonBlock(15);
    readLight();
    if (error != LOST) { // We found our way back!
      Serial.println("We are back!");
      calcPID();
      return;
    }
  }

  // If reached this point, no luck finding our way back :( Try again in 2 seconds
  delayNonBlock(2000);
  findWayBack();
}

void nextLED() {
  // change LED randomly if autoLED is true (set with MATLAB)
  if (autoLED && round(micros()) % 100 == 0) {
    digitalWrite(ledPin[ledOnIndex], LOW);
    if (ledOnIndex == 0)
      dir = 1;
    else if (ledOnIndex == NUM_OF_LEDS - 1)
      dir = -1;
    ledOnIndex += dir;
    digitalWrite(ledPin[ledOnIndex], HIGH);
    }
  }


void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_OF_LEDS; i++) {
    pinMode(ledPin[i], OUTPUT);
    digitalWrite(ledPin[i], LOW);
  }

  myServo.attach(servoPin);
  initMatlab();  // This will block code until MATLAB connects
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  calibrate();
}

void loop() {
  handleMatlab(); // Handles MATLAB Commands
  readLight();
  calcPID();
  updateServo();
  nextLED();
}
