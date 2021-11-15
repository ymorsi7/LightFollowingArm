#include <Servo.h>

Servo myServo;  // servo object to control servo
int pos = 0;    // store servo position


void servoSweep() {
  for (pos = 0; pos <= 180; pos+10 ) { // increase pos by 10 degree intervals
    myServo.write(0); // write current servo position
    delay(100);
  }
  for (pos = 180; pos >= 0; pos-10 ) { // decrease pos by 10 degree intervals
    myServo.write(180); // write current servo position
    delay(100);
  }
}

void setup() {
  myServo.attach(3); // servo pin number (orange wire)
  myServo.write(90); // servo center position (refer to step 9)
}

void loop() {
  servoSweep(); // comment this out when positioning servo arm
}
