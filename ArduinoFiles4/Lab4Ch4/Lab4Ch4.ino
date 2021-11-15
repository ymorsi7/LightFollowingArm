#include <Servo.h>
#define NUM_OF_PHOTORESISTORS 5
#define NUM_OF_LEDS 7
#define SERVO_MAX_LIMIT 35
// Pin Definitions
short photoresistorPin[] = {A0, A1, A2, A3, A4};
short ledPin[] = {6, 7, 8, 9, 10, 11, 12};
short servoPin = 3;

int ledOnIndex = 3;
int dir = 1;

int currentPRLight[NUM_OF_PHOTORESISTORS];
int minPRLight[NUM_OF_PHOTORESISTORS];
int maxPRLight[NUM_OF_PHOTORESISTORS];
float error;
float LOST = -10;
int LOST_THRESHOLD = 40;  // Mapped value < this indicates lost

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
  Serial.println("----------Max Readings----------");
  for (int i = 0; i < NUM_OF_PHOTORESISTORS; i++) {
    Serial.print(maxPRLight[i]); Serial.print(' ');
  }
  Serial.println();
  Serial.println("----------Min Readings----------");
  for (int i = 0; i < NUM_OF_PHOTORESISTORS; i++) {
    Serial.print(minPRLight[i]); Serial.print(' ');
  }
  Serial.println();
  Serial.println("--------------------------------");
  // Go to center after calibration
  myServo.write(90);
  delay(3000);
}

void readLight() {
  for (short i = 0; i < NUM_OF_PHOTORESISTORS; i++) {
    currentPRLight[i] = map(analogRead(photoresistorPin[i]), minPRLight[i], maxPRLight[i], 0, 100);
    Serial.print(currentPRLight[i]);
    Serial.print(' ');
  }
  calcError();
  Serial.print(error);
  Serial.println();
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
      Serial.println("LOST TRACKING");
      error = LOST;
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

void nextLED() {
  digitalWrite(ledPin[ledOnIndex], LOW);
  ledOnIndex += dir;
  if (ledOnIndex == 0 || ledOnIndex == 6) {
    dir *= -1;
  }
  digitalWrite(ledPin[ledOnIndex], HIGH);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_OF_LEDS; i++) {
    pinMode(ledPin[i], OUTPUT);
    digitalWrite(ledPin[i], LOW);
  }

  myServo.attach(servoPin);
  calibrate();
}

void loop() {
  for (int pos = SERVO_MAX_LIMIT; pos <= 180 - SERVO_MAX_LIMIT; pos++) {
    myServo.write(pos);
    delay(100);
    readLight();
  }

  for (int pos = 180 - SERVO_MAX_LIMIT; pos >= SERVO_MAX_LIMIT; pos--) {
    myServo.write(pos);
    delay(100);
    readLight();
  }
  nextLED();
}
