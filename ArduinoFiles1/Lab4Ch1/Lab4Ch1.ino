#define NUM_OF_LEDS 7
int ledPin[] = {6, 7, 8, 9, 10, 11, 12};


void ledTest() {
  for (int i = 0; i < NUM_OF_LEDS; i++) { // complete for loop
    digitalWrite(ledPin[i], HIGH);  // turn LED on
    delay(500);
    digitalWrite(ledPin[i], LOW);  // turn LED off
  }
}

void setup() {
  for (int i = 0; i < NUM_OF_LEDS; i++) { // complete for loop (same as before)
    pinMode(ledPin[i], OUTPUT); // define LED pinmode for each LED
  }
}

void loop() {
  ledTest();
}
