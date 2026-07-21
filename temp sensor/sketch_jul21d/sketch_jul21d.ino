#include <Wire.h>
#include <Adafruit_MCP9601.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ==================================================
// Hardware configuration
// ==================================================

const uint8_t PCA9546_ADDRESS = 0x70;
const uint8_t MCP9601_ADDRESS = 0x67;

const uint8_t SENSOR_COUNT = 2;

const uint8_t WATER_SENSOR = 0;
const uint8_t SURFACE_SENSOR = 1;

// Change these if the physical channels are different.
const uint8_t SENSOR_CHANNELS[SENSOR_COUNT] = {
  0,  // Water thermocouple
  1   // Surface thermocouple
};

const char *SENSOR_NAMES[SENSOR_COUNT] = {
  "water",
  "surface"
};

// ==================================================
// Sampling configuration
// ==================================================

uint32_t sampleIntervalMs = 1000;

const uint8_t MAX_AVERAGING_SAMPLES = 10;
uint8_t averagingSamples = 5;

// ==================================================
// Sensor objects
// ==================================================

Adafruit_MCP9601 temperatureSensors[SENSOR_COUNT];

// ==================================================
// Fault definitions
// ==================================================

enum TemperatureFault {
  TEMP_FAULT_NONE,
  TEMP_FAULT_NOT_INITIALIZED,
  TEMP_FAULT_MULTIPLEXER,
  TEMP_FAULT_OPEN,
  TEMP_FAULT_SHORT,
  TEMP_FAULT_READ,
  TEMP_FAULT_STALE
};

// ==================================================
// Stored sensor state
// ==================================================

struct TemperatureState {
  bool initialized;
  bool valid;

  float latestRawC;
  float averagedC;

  float history[MAX_AVERAGING_SAMPLES];
  uint8_t historyCount;
  uint8_t historyPosition;

  TemperatureFault fault;

  uint32_t lastAttemptMs;
  uint32_t lastGoodReadingMs;
  uint32_t successfulSamples;
  uint32_t failedSamples;
};

TemperatureState temperatures[SENSOR_COUNT];

// ==================================================
// Command buffer
// ==================================================

char commandBuffer[64];
uint8_t commandLength = 0;

// ==================================================
// Multiplexer control
// ==================================================

bool selectMuxChannel(uint8_t channel) {
  if (channel > 3) {
    return false;
  }

  Wire.beginTransmission(PCA9546_ADDRESS);
  Wire.write(1 << channel);

  return Wire.endTransmission() == 0;
}

// ==================================================
// Fault text
// ==================================================

const char *faultName(TemperatureFault fault) {
  switch (fault) {
    case TEMP_FAULT_NONE:
      return "NONE";

    case TEMP_FAULT_NOT_INITIALIZED:
      return "NOT_INITIALIZED";

    case TEMP_FAULT_MULTIPLEXER:
      return "MULTIPLEXER_ERROR";

    case TEMP_FAULT_OPEN:
      return "THERMOCOUPLE_OPEN";

    case TEMP_FAULT_SHORT:
      return "THERMOCOUPLE_SHORT";

    case TEMP_FAULT_READ:
      return "READ_FAILED";

    case TEMP_FAULT_STALE:
      return "STALE_DATA";

    default:
      return "UNKNOWN";
  }
}

// ==================================================
// Averaging
// ==================================================

void clearAverage(uint8_t sensorNumber) {
  temperatures[sensorNumber].historyCount = 0;
  temperatures[sensorNumber].historyPosition = 0;
  temperatures[sensorNumber].averagedC = 0.0;

  for (uint8_t i = 0; i < MAX_AVERAGING_SAMPLES; i++) {
    temperatures[sensorNumber].history[i] = 0.0;
  }
}

void addToAverage(
    uint8_t sensorNumber,
    float newTemperatureC) {

  TemperatureState &state = temperatures[sensorNumber];

  state.history[state.historyPosition] = newTemperatureC;

  state.historyPosition++;

  if (state.historyPosition >= averagingSamples) {
    state.historyPosition = 0;
  }

  if (state.historyCount < averagingSamples) {
    state.historyCount++;
  }

  float total = 0.0;

  for (uint8_t i = 0; i < state.historyCount; i++) {
    total += state.history[i];
  }

  state.averagedC = total / state.historyCount;
}

// ==================================================
// Sensor initialization
// ==================================================

bool initializeOneSensor(uint8_t sensorNumber) {
  TemperatureState &state = temperatures[sensorNumber];

  state.initialized = false;
  state.valid = false;
  state.latestRawC = 0.0;
  state.averagedC = 0.0;
  state.fault = TEMP_FAULT_NOT_INITIALIZED;
  state.lastAttemptMs = 0;
  state.lastGoodReadingMs = 0;
  state.successfulSamples = 0;
  state.failedSamples = 0;

  clearAverage(sensorNumber);

  if (!selectMuxChannel(SENSOR_CHANNELS[sensorNumber])) {
    state.fault = TEMP_FAULT_MULTIPLEXER;
    return false;
  }

  delay(20);

  if (!temperatureSensors[sensorNumber].begin(
          MCP9601_ADDRESS)) {
    state.fault = TEMP_FAULT_NOT_INITIALIZED;
    return false;
  }

  temperatureSensors[sensorNumber].setThermocoupleType(
      MCP9600_TYPE_K);

  state.initialized = true;
  state.fault = TEMP_FAULT_NONE;

  return true;
}

void initializeAllSensors() {
  for (uint8_t sensor = 0;
       sensor < SENSOR_COUNT;
       sensor++) {

    Serial.print("Initializing ");
    Serial.print(SENSOR_NAMES[sensor]);
    Serial.print(" temperature sensor... ");

    if (initializeOneSensor(sensor)) {
      Serial.println("OK");
    } else {
      Serial.print("FAILED: ");
      Serial.println(faultName(
          temperatures[sensor].fault));
    }
  }
}

// ==================================================
// Read one sensor
// ==================================================

void sampleOneSensor(uint8_t sensorNumber) {
  TemperatureState &state = temperatures[sensorNumber];

  state.lastAttemptMs = millis();

  if (!state.initialized) {
    state.valid = false;
    state.fault = TEMP_FAULT_NOT_INITIALIZED;
    state.failedSamples++;
    return;
  }

  if (!selectMuxChannel(SENSOR_CHANNELS[sensorNumber])) {
    state.valid = false;
    state.fault = TEMP_FAULT_MULTIPLEXER;
    state.failedSamples++;
    return;
  }

  delay(5);

  uint8_t status =
      temperatureSensors[sensorNumber].getStatus();

  if (status & MCP9601_STATUS_OPENCIRCUIT) {
    state.valid = false;
    state.fault = TEMP_FAULT_OPEN;
    state.failedSamples++;
    return;
  }

  if (status & MCP9601_STATUS_SHORTCIRCUIT) {
    state.valid = false;
    state.fault = TEMP_FAULT_SHORT;
    state.failedSamples++;
    return;
  }

  float newTemperature =
      temperatureSensors[sensorNumber]
          .readThermocouple();

  if (isnan(newTemperature)) {
    state.valid = false;
    state.fault = TEMP_FAULT_READ;
    state.failedSamples++;
    return;
  }

  state.latestRawC = newTemperature;

  addToAverage(sensorNumber, newTemperature);

  state.valid = true;
  state.fault = TEMP_FAULT_NONE;
  state.lastGoodReadingMs = millis();
  state.successfulSamples++;
}

// ==================================================
// Read both sensors
// ==================================================

void sampleAllSensors() {
  for (uint8_t sensor = 0;
       sensor < SENSOR_COUNT;
       sensor++) {

    sampleOneSensor(sensor);
  }
}

// ==================================================
// Check for stale data
// ==================================================

void checkForStaleData() {
  uint32_t staleLimitMs = sampleIntervalMs * 3;

  if (staleLimitMs < 3000) {
    staleLimitMs = 3000;
  }

  for (uint8_t sensor = 0;
       sensor < SENSOR_COUNT;
       sensor++) {

    TemperatureState &state = temperatures[sensor];

    if (state.valid &&
        millis() - state.lastGoodReadingMs >
            staleLimitMs) {

      state.valid = false;
      state.fault = TEMP_FAULT_STALE;
    }
  }
}

// ==================================================
// Print one temperature field
// ==================================================

void printTemperatureValue(uint8_t sensorNumber) {
  if (temperatures[sensorNumber].valid) {
    Serial.print(
        temperatures[sensorNumber].averagedC,
        2);
  } else {
    Serial.print("INVALID");
  }
}

// ==================================================
// Send combined data packet
// ==================================================

void sendDataPacket() {
  checkForStaleData();

  Serial.print("DATA,");

  Serial.print("water_temp_c=");
  printTemperatureValue(WATER_SENSOR);

  Serial.print(",water_valid=");
  Serial.print(
      temperatures[WATER_SENSOR].valid ? 1 : 0);

  Serial.print(",water_fault=");
  Serial.print(faultName(
      temperatures[WATER_SENSOR].fault));

  Serial.print(",water_age_ms=");

  if (temperatures[WATER_SENSOR]
          .lastGoodReadingMs == 0) {
    Serial.print("NA");
  } else {
    Serial.print(
        millis() -
        temperatures[WATER_SENSOR]
            .lastGoodReadingMs);
  }

  Serial.print(",surface_temp_c=");
  printTemperatureValue(SURFACE_SENSOR);

  Serial.print(",surface_valid=");
  Serial.print(
      temperatures[SURFACE_SENSOR].valid ? 1 : 0);

  Serial.print(",surface_fault=");
  Serial.print(faultName(
      temperatures[SURFACE_SENSOR].fault));

  Serial.print(",surface_age_ms=");

  if (temperatures[SURFACE_SENSOR]
          .lastGoodReadingMs == 0) {
    Serial.print("NA");
  } else {
    Serial.print(
        millis() -
        temperatures[SURFACE_SENSOR]
            .lastGoodReadingMs);
  }

  Serial.println();
}

// ==================================================
// Send controller status
// ==================================================

void sendStatusPacket() {
  Serial.print("STATUS,");

  Serial.print("sample_rate_ms=");
  Serial.print(sampleIntervalMs);

  Serial.print(",averaging=");
  Serial.print(averagingSamples);

  for (uint8_t sensor = 0;
       sensor < SENSOR_COUNT;
       sensor++) {

    Serial.print(",");
    Serial.print(SENSOR_NAMES[sensor]);
    Serial.print("_initialized=");
    Serial.print(
        temperatures[sensor].initialized ? 1 : 0);

    Serial.print(",");
    Serial.print(SENSOR_NAMES[sensor]);
    Serial.print("_successful=");
    Serial.print(
        temperatures[sensor].successfulSamples);

    Serial.print(",");
    Serial.print(SENSOR_NAMES[sensor]);
    Serial.print("_failed=");
    Serial.print(
        temperatures[sensor].failedSamples);
  }

  Serial.println();
}

// ==================================================
// Reinitialize sensors
// ==================================================

void reinitializeSensors() {
  Serial.println("Reinitializing temperature sensors...");

  initializeAllSensors();
  sampleAllSensors();

  Serial.println("Reinitialization complete.");
}

// ==================================================
// Help menu
// ==================================================

void sendHelp() {
  Serial.println("COMMANDS:");
  Serial.println("  GET");
  Serial.println("  STATUS");
  Serial.println("  SAMPLE");
  Serial.println("  RATE 1000");
  Serial.println("  AVG 5");
  Serial.println("  REINIT");
  Serial.println("  HELP");
}

// ==================================================
// Process commands
// ==================================================

void processCommand(char *command) {
  if (strcmp(command, "GET") == 0) {
    sendDataPacket();
  }

  else if (strcmp(command, "STATUS") == 0) {
    sendStatusPacket();
  }

  else if (strcmp(command, "SAMPLE") == 0) {
    sampleAllSensors();
    sendDataPacket();
  }

  else if (strcmp(command, "REINIT") == 0) {
    reinitializeSensors();
  }

  else if (strcmp(command, "HELP") == 0) {
    sendHelp();
  }

  else if (strncmp(command, "RATE ", 5) == 0) {
    uint32_t requestedRate =
        atol(command + 5);

    if (requestedRate < 100 ||
        requestedRate > 60000) {

      Serial.println(
          "ERROR,RATE_MUST_BE_100_TO_60000_MS");
    } else {
      sampleIntervalMs = requestedRate;

      Serial.print("OK,sample_rate_ms=");
      Serial.println(sampleIntervalMs);
    }
  }

  else if (strncmp(command, "AVG ", 4) == 0) {
    int requestedAverage =
        atoi(command + 4);

    if (requestedAverage < 1 ||
        requestedAverage >
            MAX_AVERAGING_SAMPLES) {

      Serial.println(
          "ERROR,AVG_MUST_BE_1_TO_10");
    } else {
      averagingSamples =
          (uint8_t)requestedAverage;

      for (uint8_t sensor = 0;
           sensor < SENSOR_COUNT;
           sensor++) {

        clearAverage(sensor);
      }

      Serial.print("OK,averaging=");
      Serial.println(averagingSamples);
    }
  }

  else if (command[0] != '\0') {
    Serial.println("ERROR,UNKNOWN_COMMAND");
  }
}

// ==================================================
// Receive commands
// ==================================================

void receiveCommands() {
  while (Serial.available() > 0) {
    char incomingCharacter = Serial.read();

    if (incomingCharacter == '\r') {
      continue;
    }

    if (incomingCharacter == '\n') {
      commandBuffer[commandLength] = '\0';

      processCommand(commandBuffer);

      commandLength = 0;
    }

    else if (commandLength <
             sizeof(commandBuffer) - 1) {

      commandBuffer[commandLength] =
          incomingCharacter;

      commandLength++;
    }

    else {
      commandLength = 0;

      Serial.println(
          "ERROR,COMMAND_TOO_LONG");
    }
  }
}

// ==================================================
// Arduino setup
// ==================================================

void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 5000) {
    // Wait briefly for Serial Monitor.
  }

  Wire.begin();
  delay(500);

  Serial.println();
  Serial.println(
      "Two-Temperature End Effector Controller");
  Serial.println(
      "=======================================");

  initializeAllSensors();
  sampleAllSensors();

  sendHelp();
}

// ==================================================
// Arduino main loop
// ==================================================

void loop() {
  receiveCommands();

  static uint32_t lastAutomaticSampleMs = 0;

  if (millis() - lastAutomaticSampleMs >=
      sampleIntervalMs) {

    lastAutomaticSampleMs = millis();

    sampleAllSensors();
  }

  checkForStaleData();
}
