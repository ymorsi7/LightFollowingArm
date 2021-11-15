#define NUM_OF_PHOTORESISTORS 5
int photoresistorPin[] = {A0, A1, A2, A3, A4};


void readLight() {
  for(int i = 0; i < NUM_OF_PHOTORESISTORS ; i++) { // complete For loop
    Serial.print(analogRead(photoresistorPin[i])); // print each photoresistor value
    Serial.print(" ");
  }
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  readLight();
  Serial.println();
  delay(200); // slow down output
}
