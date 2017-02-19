/**
 * Use an electromagnet to accelerate a fidget spinner.
 */

#define PIN_EM 2
#define PIN_STATUS_LED 13

void setup() {
  Serial.begin(9600);
  pinMode(PIN_EM, OUTPUT);
  digitalWrite(PIN_EM, LOW);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_STATUS_LED, LOW);
}

void pulse(unsigned long ms) {
  digitalWrite(PIN_EM, HIGH);
  digitalWrite(PIN_STATUS_LED, HIGH);
  delay(ms);
  digitalWrite(PIN_EM, LOW);
  digitalWrite(PIN_STATUS_LED, LOW);
}

void loop() {
}
