/**
 * Use an electromagnet to accelerate a fidget spinner.
 *
 * IR proximity sensor VCNL4000 from sparkfun.com/products/retired/10901 .
 */

#include <Wire.h>

#define PIN_EM 2
#define PIN_STATUS_LED 13

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

// Higher numbers = increased proximity.
#define PROX_FAR 3000
#define PROX_EDGE 18000
#define PROX_CENTER 20000

#define PULSE_MAX_MS 1000

#define NUM_ARMS 3

enum state_t {
  AFTER_PASS,
  PULSE,
  PASS_ARRIVING,
  PASS_CENTER
};
enum state_t currentState;

unsigned long lastPassMs;
unsigned long lastIntervalMs;
unsigned long pulseOnMs;

void setup() {
  Serial.begin(57600);

  pinMode(PIN_EM, OUTPUT);
  digitalWrite(PIN_EM, LOW);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  for (int i = 0; i < 4; i++) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(100);
    digitalWrite(PIN_STATUS_LED, LOW);
  }

  setupIr();

  lastPassMs = millis();
  lastIntervalMs = 1;
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
        digitalWrite(PIN_EM, HIGH);
        digitalWrite(PIN_STATUS_LED, HIGH);
        pulseOnMs = t;
      } else if (prox >= PROX_EDGE) {
        currentState = PASS_ARRIVING;
        logTimeAndProx(t, prox);
        Serial.println("after => skip pulse => arriving");
      }
      break;
    case PULSE:
      if (prox >= PROX_EDGE || t - pulseOnMs > PULSE_MAX_MS) {
        logTimeAndProx(t, prox);
        if (prox < PROX_EDGE) {
          Serial.println("pulse => timeout");
        } else {
          Serial.println("pulse => arriving");
        }
        currentState = PASS_ARRIVING;
        digitalWrite(PIN_EM, LOW);
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
        logTimeAndProx(t, prox);
        Serial.print("center => after, dt=");
        Serial.print(lastIntervalMs);
        Serial.print("ms / ");
        Serial.print((60.0 * 1000) / (lastIntervalMs * NUM_ARMS));
        Serial.print("rpm\n\n");
      }
      break;
  }
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
