#define BAUD_RATE 57600


void setup() {
  Serial.begin(BAUD_RATE);
}
void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    Serial.print("You sent me: ");
    Serial.println(data);
  }
}