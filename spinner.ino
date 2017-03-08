/**
 * Use an electromagnet (EM) to turn a fidget spinner, see README for overview.
 */

#include <Wire.h>
#include <EEPROM.h>
#include "SevSeg.h"

#define PIN_ELECTROMAGNET 2
#define PIN_STATUS_LED 13

// infra-red rangefinder on I2C
#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read

// VCNL4000 Register Map
#define COMMAND_0 0x80  // starts measurments, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x11
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define AMBIENT_RESULT_MSB 0x85  // high byte of ambient light measure
#define AMBIENT_RESULT_LSB 0x86  // low byte of ambient light measure
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_FREQ 0x89  // Proximity IR test signal freq, 0-3
#define PROXIMITY_MOD 0x8A  // proximity modulator timing

// Proximity thresholds for detecting when the spinner is passing over the IR
// rangefinder. Higher numbers = increased proximity = decreased distance.
#define PROX_FAR 3000
#define PROX_EDGE 18000
#define PROX_CENTER 20000

// Never keep the EM on longer than PULSE_MAX_MS (for example if the spinner
// is stuck or removed), to avoid heating up.
#define PULSE_MAX_MS 1000

// Number of arms on the spinner, for RPM calculation.
#define NUM_ARMS 3

// Overall program state transitions.
enum state_t {
  // After an arm of the spinner passes by, the electromagnet will be off.
  AFTER_PASS,

  // When we think the spinner is approaching, turn the EM on for a pulse.
  PULSE,

  // When the spinner arrives (leading edge), turn off the EM.
  PASS_ARRIVING,

  // When we see the spinner center, calculate timing (for RPM and the next
  // pulse).
  PASS_CENTER
};
enum state_t currentState;

unsigned long lastPassMs;
float rpm;
char rpmDisplayBuffer[6];
unsigned long lastIntervalMs;
unsigned long pulseOnMs;

float targetRpm;
// If it takes period T between when one arm/ball and the next goes by, the
// EM should be off for T/2 and then off for T/onFraction, then off again.
// For maximum acceleration, this is 1/2 and 1/2; when targeting a specific RPM
// the on time may be shorter (onFraction > 2).
float onFraction;

SevSeg display;


void setup() {
  Serial.begin(57600);

  pinMode(PIN_ELECTROMAGNET, OUTPUT);
  digitalWrite(PIN_ELECTROMAGNET, LOW);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  setupIr();

  byte digits[] = {A3, A2, A1, A0}; // Digit D1, D2, D3, D4
  byte segments[] = {5, 3, 6, 10, 9, 7, 4, 8}; // Segments A-G, dot
  display.begin(COMMON_ANODE, /* number of digits */ 4, digits, segments);
  display.setBrightness(100);

  // Cycle through different RPM targets each time the sketch restarts.
  byte c = EEPROM.read(0);
  EEPROM.write(0, c + 1);
  switch(c % 3) {
    case 2:
      targetRpm = 120.0f;
      break;
    case 1:
      targetRpm = 60.0f;
      break;
    default:
      targetRpm = 1e6f;
  }
  onFraction = 2.0;

  lastPassMs = millis();
  rpm = 0.0f;
  lastIntervalMs = 1000;
  currentState = AFTER_PASS;
}

void loop() {
  unsigned long t = millis();
  unsigned int prox = readProximity();
  switch(currentState) {
    case AFTER_PASS:
      if (t - lastPassMs > lastIntervalMs / 2) {
        currentState = PULSE;
        logTimeAndProx(t, prox);
        Serial.println("after => pulse");
        digitalWrite(PIN_ELECTROMAGNET, HIGH);
        digitalWrite(PIN_STATUS_LED, HIGH);
        pulseOnMs = t;
      } else if (prox >= PROX_EDGE) {
        currentState = PASS_ARRIVING;
        logTimeAndProx(t, prox);
        Serial.println("after => skip pulse => arriving");
      }
      break;
    case PULSE:
      if (prox >= PROX_EDGE ||
          t - pulseOnMs > min(lastIntervalMs / onFraction, PULSE_MAX_MS)) {
        logTimeAndProx(t, prox);
        if (prox < PROX_EDGE) {
          Serial.println("pulse => timeout");
          rpm = 0.0f;
        } else {
          Serial.println("pulse => arriving");
        }
        currentState = PASS_ARRIVING;
        digitalWrite(PIN_ELECTROMAGNET, LOW);
        digitalWrite(PIN_STATUS_LED, LOW);
      }
      break;
    case PASS_ARRIVING:
      if (prox >= PROX_CENTER) {
        logTimeAndProx(t, prox);
        Serial.println("arriving => center");
        currentState = PASS_CENTER;
      }
      break;
    case PASS_CENTER:
      if (prox <= PROX_FAR) {
        currentState = AFTER_PASS;
        lastIntervalMs = (t - lastPassMs);
        lastPassMs = t;
        rpm = (60.0 * 1000) / (lastIntervalMs * NUM_ARMS);
        display.setNumber(rpm, 1);
        logTimeAndProx(t, prox);
        Serial.print("center => after, dt=");
        Serial.print(lastIntervalMs);
        Serial.print("ms / ");
        Serial.print(rpm);
        Serial.print("rpm\n\n");

        // Adjust how long the EM stays on based on current v. target RPM.
        // This could be replaced by a PID controller.
        float rpmFraction = rpm / targetRpm;
        if (rpmFraction > 1.0) {
          onFraction = 60.0;
        } else if (rpmFraction > 0.95) {
          onFraction = 50.0;
        } else if (rpmFraction > 0.9) {
          onFraction = 40.0;
        } else if (rpmFraction > 0.5) {
          onFraction = 8.0;
        } else {
          onFraction = 2.0; // maximum power
        }
      }
      break;
  }
  display.refreshDisplay();
}

void logTimeAndProx(unsigned long t, int prox) {
  Serial.print("t=");
  Serial.print(t);
  Serial.print("\tprox=");
  Serial.print(prox);
  Serial.print("\t");
}

void setupIr() {
  Wire.begin();  // initialize I2C stuff

  /* Test that the device is working correctly */
  byte temp = readByte(PRODUCT_ID);
  if (temp != 0x11)  // Product ID Should be 0x11
  {
    Serial.print("Incorrect ID for VNCL4000: 0x");
    Serial.println(temp, HEX);
  } else {
    Serial.println("VNCL4000 Online.");
  }

  writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
  writeByte(IR_CURRENT, 20);  // Set IR current to 200mA
  writeByte(PROXIMITY_FREQ, 2);  // 781.25 kHz
  writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay
}

// readProximity() returns a 16-bit value from the VCNL4000's proximity data
// registers
unsigned int readProximity()
{
  unsigned int data;
  byte temp;

  temp = readByte(COMMAND_0);
  // command the sensor to perform a proximity measure
  writeByte(COMMAND_0, temp | 0x08);

  while(!(readByte(COMMAND_0)&0x20))
    ;  // Wait for the proximity data ready bit to be set
  data = readByte(PROXIMITY_RESULT_MSB) << 8;
  data |= readByte(PROXIMITY_RESULT_LSB);

  return data;
}

// readAmbient() returns a 16-bit value from the VCNL4000's ambient light data registers
unsigned int readAmbient()
{
  unsigned int data;
  byte temp;

  temp = readByte(COMMAND_0);
  writeByte(COMMAND_0, temp | 0x10);  // command the sensor to perform ambient measure

  while(!(readByte(COMMAND_0)&0x40))
    ;  // wait for the proximity data ready bit to be set
  data = readByte(AMBIENT_RESULT_MSB) << 8;
  data |= readByte(AMBIENT_RESULT_LSB);

  return data;
}

// writeByte(address, data) writes a single byte of data to address
void writeByte(byte address, byte data)
{
  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

// readByte(address) reads a single byte of data from address
byte readByte(byte address)
{
  byte data;

  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(VCNL4000_ADDRESS, 1);
  while(!Wire.available())
    ;
  data = Wire.read();

  return data;
}
