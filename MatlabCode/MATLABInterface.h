/**
 * MATLABInterface - Handles communication between Arduino and MATLAB
 * for ECE 5 Lab 4 SP20 Version using Arduino-SerialCommand library
 * written by Scott Zhao under GNU General Public License
 *
 * NOTE: Must be used together with provided MATLAB side software
 *
 * Copyright (C) 2020 Scott Zhao <scott@scottz.net>
 *                    https://github.com/zhaomh1998
 * 
 * Version 20200527
 */
#ifndef MATLABInterface_h
#define MATLABInterface_h

#include "SerialCommand.h"

SerialCommand sCmd; // Arduino-SerialCommand library to work with PC
extern float kP, kI, kD, error, sumError, lastError, turn;
extern int ledOnIndex;
extern int currentPRLight[];
extern int currentPosition;
extern const short ledPin[];
extern const int NUM_OF_PHOTORESISTORS, NUM_OF_LEDS;
extern bool autoLED;
bool __isMatlabConnected = false;

// Acknowledge command to MATLAB
inline void __ack() {
  Serial.println("Done");
}

// Parse PID values
bool __serialReadPID(float& theFirst, float& theSecond, float& theThird) {
  char *arg;
  // Parse First Arg
  arg = sCmd.next();
  if (arg != NULL) {
    theFirst = String(arg).toFloat();
    // Parse Second Arg
    arg = sCmd.next();
    if (arg != NULL) {
      theSecond = String(arg).toFloat();
      // Parse Third Arg
      arg = sCmd.next();
      if (arg != NULL) {
        theThird = String(arg).toFloat();
        return true;
      } else {
        Serial.println("Expected 3 arguments, 2 received!");
      }
    } else {
      Serial.println("Expected 3 arguments, 1 received!");
    }
  } else {
    Serial.println("Expected 3 arguments, 0 received!");
  }
  return false;  // Something goes wrong :/
}

// Parse LED index
bool __serialReadLEDIndex(int& theFirst) {
  char *arg;
  int temp;
  // Parse First Arg
  arg = sCmd.next();
  if (arg != NULL) {
    temp = String(arg).toInt();
    if (temp >= 0 && temp < NUM_OF_LEDS) {
      theFirst = temp;
      return true;
    } else {
      if(temp == -2) {
        autoLED = true;
        __ack();
      }
      else if(temp == -1) {
        autoLED = false;
        __ack();
      }
      else
        Serial.println("LED Index out of range!");
    }
  } else {
    Serial.println("Expected 1 argument, 0 received!");
  }
  return false;
}

// This gets set as the default handler, and gets called when no other command matches
// Not expected to be executed
void __unrecognized(const char *command) {
  Serial.println("ERROR: Unrecognized Command");
}

void __handShake() {
  Serial.println("\nHELLO");
  __isMatlabConnected = true;
}

void __setPID() {
  if (__serialReadPID(kP, kI, kD)) {
    __ack();
  }
}

void __sendToPC() {
  Serial.print("GETDATA");      Serial.print(' ');
  Serial.print(kP, 5);          Serial.print(' ');
  Serial.print(kI, 5);          Serial.print(' ');
  Serial.print(kD, 5);          Serial.print(' ');
  Serial.print(error, 5);       Serial.print(' ');
  Serial.print(sumError, 5);    Serial.print(' ');
  Serial.print(lastError, 5);   Serial.print(' ');
  Serial.print(turn, 5);        Serial.print(' ');
  Serial.print(currentPosition);Serial.print(' ');
  for(short i = 0; i < NUM_OF_PHOTORESISTORS - 1; i++) {
    Serial.print(currentPRLight[i]);
    Serial.print(' ');
  }
  Serial.println(currentPRLight[NUM_OF_PHOTORESISTORS-1]);
}

void __setLED() {
  if(__serialReadLEDIndex(ledOnIndex)) {
    for(short i = 0; i < NUM_OF_LEDS; i++) {
      if(i == ledOnIndex)
        digitalWrite(ledPin[i], HIGH);
      else
        digitalWrite(ledPin[i], LOW);
    }
    __ack();
  }
}

inline void handleMatlab() {
  sCmd.readSerial(); // Reads from Serial and update PID if received new values
}

inline void initMatlab() {
  sCmd.addCommand("HELLO", __handShake);  // Handshake, returns HELLO when received HELLO, for MATLAB to verify connection
  sCmd.addCommand("SETPID", __setPID);    // set PID, Reads float P, float I, float D, as arguments, sets PID and prints Done
  sCmd.addCommand("GETDATA", __sendToPC); // send kP, kI, kD, error, sumError, lastError and PRLights to MATLAB
  sCmd.addCommand("SETLED", __setLED);    // set LED light auto mode / turn on a specific LED
  sCmd.setDefaultHandler(__unrecognized); // Handler for command that isn't matched
  Serial.print("Waiting for Matlab");
  while (!__isMatlabConnected) {
    Serial.print(".");
    handleMatlab();
    delay(500);
  }
}

#endif //MATLABInterface_h
