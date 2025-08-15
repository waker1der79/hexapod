
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

#define SERVO_MIN 150
#define SERVO_MAX 600

#define CH_COXA  0
#define CH_FEMUR 1
#define CH_TIBIA 2

#define ADDR_COXA_OFFSET  0
#define ADDR_FEMUR_OFFSET 4
#define ADDR_TIBIA_OFFSET 8

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

float offsetCoxa  = 0;
float offsetFemur = 0;
float offsetTibia = 0;

void moveServo(uint8_t channel, float angle);
float readEEPROMFloat(int address);
void writeEEPROMFloat(int address, float value);
void calibrateServos();

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== One-Leg Hexapod Test ==="));

  pca.begin();
  pca.setPWMFreq(60);
  delay(10);

  offsetCoxa  = readEEPROMFloat(ADDR_COXA_OFFSET);
  offsetFemur = readEEPROMFloat(ADDR_FEMUR_OFFSET);
  offsetTibia = readEEPROMFloat(ADDR_TIBIA_OFFSET);

  Serial.println(F("Loaded servo offsets from EEPROM:"));
  Serial.print(F("Coxa: "));  Serial.println(offsetCoxa);
  Serial.print(F("Femur: ")); Serial.println(offsetFemur);
  Serial.print(F("Tibia: ")); Serial.println(offsetTibia);

  Serial.println(F("Enter 'cal' to start calibration, or 'test' to move servos."));
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("cal")) {
      calibrateServos();
    }
    else if (cmd.equalsIgnoreCase("test")) {
      Serial.println(F("Running test sequence..."));
      moveServo(CH_COXA,  45); delay(500);
      moveServo(CH_FEMUR, 30); delay(500);
      moveServo(CH_TIBIA, 60); delay(500);
      moveServo(CH_COXA,  0);  delay(500);
      moveServo(CH_FEMUR, 0);  delay(500);
      moveServo(CH_TIBIA, 0);  delay(500);
    }
  }
}

void moveServo(uint8_t channel, float angle) {
  float offset = 0;
  if (channel == CH_COXA)  offset = offsetCoxa;
  if (channel == CH_FEMUR) offset = offsetFemur;
  if (channel == CH_TIBIA) offset = offsetTibia;

  float adjAngle = angle + offset;
  if (adjAngle < 0)   adjAngle = 0;
  if (adjAngle > 180) adjAngle = 180;

  uint16_t pulselen = map((long)adjAngle, 0, 180, SERVO_MIN, SERVO_MAX);
  pca.setPWM(channel, 0, pulselen);
}

float readEEPROMFloat(int address) {
  float value = 0.0;
  EEPROM.get(address, value);
  return value;
}

void writeEEPROMFloat(int address, float value) {
  EEPROM.put(address, value);
}

void calibrateServos() {
  Serial.println(F("=== Calibration Mode ==="));
  Serial.println(F("Commands: c+/c- = Coxa, f+/f- = Femur, t+/t- = Tibia, save = store offsets, exit = quit"));

  bool running = true;
  while (running) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      if (cmd.equalsIgnoreCase("c+")) { offsetCoxa += 1; moveServo(CH_COXA, 90); }
      else if (cmd.equalsIgnoreCase("c-")) { offsetCoxa -= 1; moveServo(CH_COXA, 90); }
      else if (cmd.equalsIgnoreCase("f+")) { offsetFemur += 1; moveServo(CH_FEMUR, 90); }
      else if (cmd.equalsIgnoreCase("f-")) { offsetFemur -= 1; moveServo(CH_FEMUR, 90); }
      else if (cmd.equalsIgnoreCase("t+")) { offsetTibia += 1; moveServo(CH_TIBIA, 90); }
      else if (cmd.equalsIgnoreCase("t-")) { offsetTibia -= 1; moveServo(CH_TIBIA, 90); }
      else if (cmd.equalsIgnoreCase("save")) {
        writeEEPROMFloat(ADDR_COXA_OFFSET, offsetCoxa);
        writeEEPROMFloat(ADDR_FEMUR_OFFSET, offsetFemur);
        writeEEPROMFloat(ADDR_TIBIA_OFFSET, offsetTibia);
        Serial.println(F("Offsets saved to EEPROM."));
      }
      else if (cmd.equalsIgnoreCase("exit")) {
        Serial.println(F("Exiting calibration."));
        running = false;
      }

      Serial.print(F("Offsets: Coxa=")); Serial.print(offsetCoxa);
      Serial.print(F(", Femur=")); Serial.print(offsetFemur);
      Serial.print(F(", Tibia=")); Serial.println(offsetTibia);
    }
  }
}
