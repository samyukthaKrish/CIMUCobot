#include <Wire.h>
#include <Adafruit_MCP9601.h>

const uint8_t PCA9546_ADDRESS = 0x70;
const uint8_t MCP9601_ADDRESS = 0x67;
const uint8_t TEMP_CHANNEL = 0;

Adafruit_MCP9601 thermocouple;

bool selectMuxChannel(uint8_t channel) {
  if (channel > 3) {
    return false;
  }

  Wire.beginTransmission(PCA9546_ADDRESS);
  Wire.write(1 << channel);

  return Wire.endTransmission() == 0;
}

void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 5000) {
    // Wait briefly for Serial Monitor.
  }

  Wire.begin();
  delay(500);

  Serial.println();
  Serial.println("Testing MCP9601 through multiplexer");
  Serial.println("===================================");

  if (!selectMuxChannel(TEMP_CHANNEL)) {
    Serial.println("ERROR: PCA9546 multiplexer not found.");

    while (true) {
      delay(100);
    }
  }

  delay(20);

  if (!thermocouple.begin(MCP9601_ADDRESS)) {
    Serial.println("ERROR: MCP9601 not found on channel 0.");

    while (true) {
      delay(100);
    }
  }

  thermocouple.setThermocoupleType(MCP9600_TYPE_K);

  Serial.println("SUCCESS: MCP9601 found on channel 0.");
  Serial.println("K-type thermocouple configured.");
}

void loop() {
  if (!selectMuxChannel(TEMP_CHANNEL)) {
    Serial.println("ERROR: Multiplexer communication failed.");
    delay(1000);
    return;
  }

  delay(10);

  uint8_t status = thermocouple.getStatus();

  if (status & MCP9601_STATUS_OPENCIRCUIT) {
    Serial.println("Thermocouple open: no thermocouple is connected.");
  } else if (status & MCP9601_STATUS_SHORTCIRCUIT) {
    Serial.println("ERROR: Thermocouple appears shorted.");
  } else {
    float temperatureC = thermocouple.readThermocouple();

    Serial.print("Temperature: ");
    Serial.print(temperatureC, 2);
    Serial.println(" C");
  }

  delay(1000);
}