#include <Wire.h>

const uint8_t PCA9546_ADDRESS = 0x70;
const uint8_t MCP9601_ADDRESS = 0x67;

bool selectMuxChannel(uint8_t channel) {
  if (channel > 3) {
    return false;
  }

  Wire.beginTransmission(PCA9546_ADDRESS);
  Wire.write(1 << channel);

  return Wire.endTransmission() == 0;
}

void disableAllMuxChannels() {
  Wire.beginTransmission(PCA9546_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
}

void scanCurrentBus() {
  bool foundDevice = false;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t result = Wire.endTransmission();

    if (result == 0) {
      Serial.print("  Found device at 0x");

      if (address < 16) {
        Serial.print("0");
      }

      Serial.println(address, HEX);
      foundDevice = true;
    }
  }

  if (!foundDevice) {
    Serial.println("  No I2C devices found.");
  }
}

void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 5000) {
    // Wait briefly for Serial Monitor.
  }

  Wire.begin();
  delay(500);

  Serial.println();
  Serial.println("PCA9546 sensor scan");
  Serial.println("===================");

  disableAllMuxChannels();

  Serial.println("Main I2C bus:");
  scanCurrentBus();

  for (uint8_t channel = 0; channel < 4; channel++) {
    Serial.println();

    Serial.print("PCA9546 channel ");
    Serial.print(channel);
    Serial.println(":");

    if (!selectMuxChannel(channel)) {
      Serial.println(
          "  ERROR: Could not communicate with PCA9546.");
      continue;
    }

    delay(20);

    scanCurrentBus();

    disableAllMuxChannels();
  }

  Serial.println();
  Serial.println("Scan finished.");
}

void loop() {
  // The scanner runs once at startup.
}
