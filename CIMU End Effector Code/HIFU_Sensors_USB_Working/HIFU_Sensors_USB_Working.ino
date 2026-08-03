#include <Wire.h>
#include <Adafruit_MCP9601.h>
#include <Adafruit_NAU7802.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ==================================================
// Hardware configuration
// ==================================================

const uint8_t PCA9546_ADDRESS = 0x70;
const uint8_t MCP9601_ADDRESS = 0x67;

// Multiplexer channels
const uint8_t WATER_CHANNEL = 0;
const uint8_t SURFACE_CHANNEL = 1;
const uint8_t PRESSURE_CHANNEL = 2;
const uint8_t FORCE_CHANNEL = 3;

// Temperature sensor indexes
const uint8_t WATER_SENSOR = 0;
const uint8_t SURFACE_SENSOR = 1;
const uint8_t TEMPERATURE_SENSOR_COUNT = 2;

// Load sensor indexes
const uint8_t PRESSURE_SENSOR = 0;
const uint8_t FORCE_SENSOR = 1;
const uint8_t LOAD_SENSOR_COUNT = 2;

const uint8_t TEMPERATURE_CHANNELS[
    TEMPERATURE_SENSOR_COUNT] = {
  WATER_CHANNEL,
  SURFACE_CHANNEL
};

const char *TEMPERATURE_NAMES[
    TEMPERATURE_SENSOR_COUNT] = {
  "water",
  "surface"
};

const uint8_t LOAD_SENSOR_CHANNELS[
    LOAD_SENSOR_COUNT] = {
  PRESSURE_CHANNEL,
  FORCE_CHANNEL
};

const char *LOAD_SENSOR_NAMES[
    LOAD_SENSOR_COUNT] = {
  "pressure",
  "force"
};

// ==================================================
// Sampling configuration
// ==================================================

uint32_t sampleIntervalMs = 1000;

const uint8_t MAX_AVERAGING_SAMPLES = 10;
uint8_t averagingSamples = 5;

// Automatically print one packet per sample
bool streamEnabled = true;

// ==================================================
// Sensor objects
// ==================================================

Adafruit_MCP9601 temperatureSensors[
    TEMPERATURE_SENSOR_COUNT];

Adafruit_NAU7802 loadSensors[
    LOAD_SENSOR_COUNT];

// ==================================================
// Temperature fault definitions
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

const char *temperatureFaultName(
    TemperatureFault fault) {

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
// Temperature state
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

TemperatureState temperatures[
    TEMPERATURE_SENSOR_COUNT];

// ==================================================
// Load sensor state
// ==================================================

struct LoadSensorState {
  bool initialized;
  bool valid;

  int32_t latestRaw;

  uint32_t lastAttemptMs;
  uint32_t lastGoodReadingMs;
  uint32_t successfulSamples;
  uint32_t failedSamples;
};

LoadSensorState loadStates[LOAD_SENSOR_COUNT];

// ==================================================
// Serial command buffer
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
// Temperature averaging
// ==================================================

void clearTemperatureAverage(uint8_t sensorNumber) {
  TemperatureState &state =
      temperatures[sensorNumber];

  state.historyCount = 0;
  state.historyPosition = 0;
  state.averagedC = 0.0;

  for (uint8_t i = 0;
       i < MAX_AVERAGING_SAMPLES;
       i++) {

    state.history[i] = 0.0;
  }
}

void addTemperatureToAverage(
    uint8_t sensorNumber,
    float newTemperatureC) {

  TemperatureState &state =
      temperatures[sensorNumber];

  state.history[state.historyPosition] =
      newTemperatureC;

  state.historyPosition++;

  if (state.historyPosition >=
      averagingSamples) {

    state.historyPosition = 0;
  }

  if (state.historyCount <
      averagingSamples) {

    state.historyCount++;
  }

  float total = 0.0;

  for (uint8_t i = 0;
       i < state.historyCount;
       i++) {

    total += state.history[i];
  }

  state.averagedC =
      total / state.historyCount;
}

// ==================================================
// Initialize one temperature sensor
// ==================================================

bool initializeOneTemperatureSensor(
    uint8_t sensorNumber) {

  TemperatureState &state =
      temperatures[sensorNumber];

  state.initialized = false;
  state.valid = false;
  state.latestRawC = 0.0;
  state.averagedC = 0.0;

  state.fault =
      TEMP_FAULT_NOT_INITIALIZED;

  state.lastAttemptMs = 0;
  state.lastGoodReadingMs = 0;
  state.successfulSamples = 0;
  state.failedSamples = 0;

  clearTemperatureAverage(sensorNumber);

  if (!selectMuxChannel(
          TEMPERATURE_CHANNELS[
              sensorNumber])) {

    state.fault =
        TEMP_FAULT_MULTIPLEXER;

    return false;
  }

  delay(20);

  if (!temperatureSensors[
          sensorNumber].begin(
              MCP9601_ADDRESS)) {

    state.fault =
        TEMP_FAULT_NOT_INITIALIZED;

    return false;
  }

  temperatureSensors[
      sensorNumber].setThermocoupleType(
          MCP9600_TYPE_K);

  state.initialized = true;
  state.fault = TEMP_FAULT_NONE;

  return true;
}

void initializeAllTemperatureSensors() {
  for (uint8_t sensor = 0;
       sensor < TEMPERATURE_SENSOR_COUNT;
       sensor++) {

    Serial.print("Initializing ");
    Serial.print(
        TEMPERATURE_NAMES[sensor]);
    Serial.print(
        " temperature sensor... ");

    if (initializeOneTemperatureSensor(
            sensor)) {

      Serial.println("OK");
    } else {
      Serial.print("FAILED: ");

      Serial.println(
          temperatureFaultName(
              temperatures[sensor].fault));
    }
  }
}

// ==================================================
// Wait for an NAU7802 reading
// ==================================================

bool waitForLoadReading(
    uint8_t sensorNumber,
    uint32_t timeoutMs = 1000) {

  uint32_t startMs = millis();

  while (millis() - startMs <
         timeoutMs) {

    if (!selectMuxChannel(
            LOAD_SENSOR_CHANNELS[
                sensorNumber])) {

      return false;
    }

    if (loadSensors[
            sensorNumber].available()) {

      return true;
    }

    delay(2);
  }

  return false;
}

// ==================================================
// Calibrate one NAU7802 mode
// ==================================================

bool calibrateLoadSensor(
    uint8_t sensorNumber,
    NAU7802_Calibration mode) {

  const uint8_t MAX_ATTEMPTS = 5;

  for (uint8_t attempt = 0;
       attempt < MAX_ATTEMPTS;
       attempt++) {

    if (!selectMuxChannel(
            LOAD_SENSOR_CHANNELS[
                sensorNumber])) {

      return false;
    }

    if (loadSensors[
            sensorNumber].calibrate(mode)) {

      return true;
    }

    delay(250);
  }

  return false;
}

// ==================================================
// Initialize one load sensor
// ==================================================

bool initializeOneLoadSensor(
    uint8_t sensorNumber) {

  LoadSensorState &state =
      loadStates[sensorNumber];

  state.initialized = false;
  state.valid = false;
  state.latestRaw = 0;
  state.lastAttemptMs = 0;
  state.lastGoodReadingMs = 0;
  state.successfulSamples = 0;
  state.failedSamples = 0;

  if (!selectMuxChannel(
          LOAD_SENSOR_CHANNELS[
              sensorNumber])) {

    return false;
  }

  delay(20);

  if (!loadSensors[
          sensorNumber].begin()) {

    return false;
  }

  selectMuxChannel(
      LOAD_SENSOR_CHANNELS[
          sensorNumber]);

  loadSensors[sensorNumber].setLDO(
      NAU7802_3V0);

  selectMuxChannel(
      LOAD_SENSOR_CHANNELS[
          sensorNumber]);

  loadSensors[sensorNumber].setGain(
      NAU7802_GAIN_128);

  selectMuxChannel(
      LOAD_SENSOR_CHANNELS[
          sensorNumber]);

  loadSensors[sensorNumber].setRate(
      NAU7802_RATE_10SPS);

  // Discard the first ten readings.
  for (uint8_t i = 0; i < 10; i++) {
    if (!waitForLoadReading(
            sensorNumber)) {

      return false;
    }

    selectMuxChannel(
        LOAD_SENSOR_CHANNELS[
            sensorNumber]);

    loadSensors[sensorNumber].read();
  }

  // Internal NAU7802 calibration
  if (!calibrateLoadSensor(
          sensorNumber,
          NAU7802_CALMOD_INTERNAL)) {

    return false;
  }

  // Zero-load offset calibration
  if (!calibrateLoadSensor(
          sensorNumber,
          NAU7802_CALMOD_OFFSET)) {

    return false;
  }

  state.initialized = true;

  return true;
}

void initializeAllLoadSensors() {
  Serial.println(
      "Keep both load sensors unloaded during calibration.");

  for (uint8_t sensor = 0;
       sensor < LOAD_SENSOR_COUNT;
       sensor++) {

    Serial.print("Initializing ");
    Serial.print(LOAD_SENSOR_NAMES[sensor]);
    Serial.print(
        " sensor on mux channel ");

    Serial.print(
        LOAD_SENSOR_CHANNELS[sensor]);

    Serial.print("... ");

    if (initializeOneLoadSensor(sensor)) {
      Serial.println("OK");
    } else {
      Serial.println("FAILED");
    }
  }
}

// ==================================================
// Sample one temperature sensor
// ==================================================

void sampleOneTemperatureSensor(
    uint8_t sensorNumber) {

  TemperatureState &state =
      temperatures[sensorNumber];

  state.lastAttemptMs = millis();

  if (!state.initialized) {
    state.valid = false;

    state.fault =
        TEMP_FAULT_NOT_INITIALIZED;

    state.failedSamples++;

    return;
  }

  if (!selectMuxChannel(
          TEMPERATURE_CHANNELS[
              sensorNumber])) {

    state.valid = false;

    state.fault =
        TEMP_FAULT_MULTIPLEXER;

    state.failedSamples++;

    return;
  }

  delay(5);

  uint8_t status =
      temperatureSensors[
          sensorNumber].getStatus();

  if (status &
      MCP9601_STATUS_OPENCIRCUIT) {

    state.valid = false;
    state.fault = TEMP_FAULT_OPEN;
    state.failedSamples++;

    return;
  }

  if (status &
      MCP9601_STATUS_SHORTCIRCUIT) {

    state.valid = false;
    state.fault = TEMP_FAULT_SHORT;
    state.failedSamples++;

    return;
  }

  float newTemperature =
      temperatureSensors[
          sensorNumber]
              .readThermocouple();

  if (isnan(newTemperature)) {
    state.valid = false;
    state.fault = TEMP_FAULT_READ;
    state.failedSamples++;

    return;
  }

  state.latestRawC = newTemperature;

  addTemperatureToAverage(
      sensorNumber,
      newTemperature);

  state.valid = true;
  state.fault = TEMP_FAULT_NONE;
  state.lastGoodReadingMs = millis();
  state.successfulSamples++;
}

// ==================================================
// Sample one load sensor
// ==================================================

void sampleOneLoadSensor(
    uint8_t sensorNumber) {

  LoadSensorState &state =
      loadStates[sensorNumber];

  state.lastAttemptMs = millis();

  if (!state.initialized) {
    state.valid = false;
    state.failedSamples++;

    return;
  }

  if (!selectMuxChannel(
          LOAD_SENSOR_CHANNELS[
              sensorNumber])) {

    state.valid = false;
    state.failedSamples++;

    return;
  }

  if (!loadSensors[
          sensorNumber].available()) {

    state.valid = false;
    state.failedSamples++;

    return;
  }

  state.latestRaw =
      loadSensors[sensorNumber].read();

  state.valid = true;
  state.lastGoodReadingMs = millis();
  state.successfulSamples++;
}

// ==================================================
// Sample all four sensors
// ==================================================

void sampleAllDevices() {
  for (uint8_t sensor = 0;
       sensor < TEMPERATURE_SENSOR_COUNT;
       sensor++) {

    sampleOneTemperatureSensor(sensor);
  }

  for (uint8_t sensor = 0;
       sensor < LOAD_SENSOR_COUNT;
       sensor++) {

    sampleOneLoadSensor(sensor);
  }
}

// ==================================================
// Check for stale readings
// ==================================================

void checkForStaleData() {
  uint32_t staleLimitMs =
      sampleIntervalMs * 3;

  if (staleLimitMs < 3000) {
    staleLimitMs = 3000;
  }

  for (uint8_t sensor = 0;
       sensor < TEMPERATURE_SENSOR_COUNT;
       sensor++) {

    TemperatureState &state =
        temperatures[sensor];

    if (state.valid &&
        millis() -
            state.lastGoodReadingMs >
                staleLimitMs) {

      state.valid = false;
      state.fault =
          TEMP_FAULT_STALE;
    }
  }

  for (uint8_t sensor = 0;
       sensor < LOAD_SENSOR_COUNT;
       sensor++) {

    LoadSensorState &state =
        loadStates[sensor];

    if (state.valid &&
        millis() -
            state.lastGoodReadingMs >
                staleLimitMs) {

      state.valid = false;
    }
  }
}

// ==================================================
// Packet printing helpers
// ==================================================

void printTemperatureValue(
    uint8_t sensorNumber) {

  if (temperatures[
          sensorNumber].valid) {

    Serial.print(
        temperatures[
            sensorNumber].averagedC,
        2);
  } else {
    Serial.print("INVALID");
  }
}

void printLoadValue(
    uint8_t sensorNumber) {

  if (loadStates[
          sensorNumber].valid) {

    Serial.print(
        loadStates[
            sensorNumber].latestRaw);
  } else {
    Serial.print("INVALID");
  }
}

void printReadingAge(
    uint32_t lastGoodReadingMs) {

  if (lastGoodReadingMs == 0) {
    Serial.print("NA");
  } else {
    Serial.print(
        millis() - lastGoodReadingMs);
  }
}

// ==================================================
// Send combined data packet
// ==================================================

void sendDataPacket() {
  checkForStaleData();

  Serial.print("DATA,");

  // Water temperature
  Serial.print("water_temp_c=");
  printTemperatureValue(WATER_SENSOR);

  Serial.print(",water_valid=");
  Serial.print(
      temperatures[WATER_SENSOR].valid
          ? 1
          : 0);

  Serial.print(",water_fault=");
  Serial.print(
      temperatureFaultName(
          temperatures[
              WATER_SENSOR].fault));

  Serial.print(",water_age_ms=");
  printReadingAge(
      temperatures[
          WATER_SENSOR]
              .lastGoodReadingMs);

  // Surface temperature
  Serial.print(",surface_temp_c=");
  printTemperatureValue(SURFACE_SENSOR);

  Serial.print(",surface_valid=");
  Serial.print(
      temperatures[SURFACE_SENSOR].valid
          ? 1
          : 0);

  Serial.print(",surface_fault=");
  Serial.print(
      temperatureFaultName(
          temperatures[
              SURFACE_SENSOR].fault));

  Serial.print(",surface_age_ms=");
  printReadingAge(
      temperatures[
          SURFACE_SENSOR]
              .lastGoodReadingMs);

  // Pressure sensor raw counts
  Serial.print(",pressure_raw=");
  printLoadValue(PRESSURE_SENSOR);

  Serial.print(",pressure_valid=");
  Serial.print(
      loadStates[PRESSURE_SENSOR].valid
          ? 1
          : 0);

  Serial.print(",pressure_age_ms=");
  printReadingAge(
      loadStates[
          PRESSURE_SENSOR]
              .lastGoodReadingMs);

  // Force sensor raw counts
  Serial.print(",force_raw=");
  printLoadValue(FORCE_SENSOR);

  Serial.print(",force_valid=");
  Serial.print(
      loadStates[FORCE_SENSOR].valid
          ? 1
          : 0);

  Serial.print(",force_age_ms=");
  printReadingAge(
      loadStates[
          FORCE_SENSOR]
              .lastGoodReadingMs);

  Serial.println();
}

// ==================================================
// Send status packet
// ==================================================

void sendStatusPacket() {
  Serial.print("STATUS,");

  Serial.print("sample_rate_ms=");
  Serial.print(sampleIntervalMs);

  Serial.print(",averaging=");
  Serial.print(averagingSamples);

  Serial.print(",stream=");
  Serial.print(streamEnabled ? "ON" : "OFF");

  for (uint8_t sensor = 0;
       sensor < TEMPERATURE_SENSOR_COUNT;
       sensor++) {

    Serial.print(",");
    Serial.print(
        TEMPERATURE_NAMES[sensor]);
    Serial.print("_initialized=");

    Serial.print(
        temperatures[sensor].initialized
            ? 1
            : 0);

    Serial.print(",");
    Serial.print(
        TEMPERATURE_NAMES[sensor]);
    Serial.print("_successful=");

    Serial.print(
        temperatures[
            sensor].successfulSamples);

    Serial.print(",");
    Serial.print(
        TEMPERATURE_NAMES[sensor]);
    Serial.print("_failed=");

    Serial.print(
        temperatures[
            sensor].failedSamples);
  }

  for (uint8_t sensor = 0;
       sensor < LOAD_SENSOR_COUNT;
       sensor++) {

    Serial.print(",");
    Serial.print(
        LOAD_SENSOR_NAMES[sensor]);
    Serial.print("_initialized=");

    Serial.print(
        loadStates[sensor].initialized
            ? 1
            : 0);

    Serial.print(",");
    Serial.print(
        LOAD_SENSOR_NAMES[sensor]);
    Serial.print("_successful=");

    Serial.print(
        loadStates[
            sensor].successfulSamples);

    Serial.print(",");
    Serial.print(
        LOAD_SENSOR_NAMES[sensor]);
    Serial.print("_failed=");

    Serial.print(
        loadStates[
            sensor].failedSamples);
  }

  Serial.println();
}

// ==================================================
// Reinitialize all sensors
// ==================================================

void reinitializeSensors() {
  Serial.println(
      "Reinitializing all sensors...");

  initializeAllTemperatureSensors();
  initializeAllLoadSensors();
  sampleAllDevices();

  Serial.println(
      "Reinitialization complete.");
}

// ==================================================
// Help menu
// ==================================================

void sendHelp() {
  Serial.println("COMMANDS:");
  Serial.println("  GET");
  Serial.println("  STATUS");
  Serial.println("  SAMPLE");
  Serial.println("  STREAM ON");
  Serial.println("  STREAM OFF");
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

  else if (strcmp(
               command,
               "STATUS") == 0) {

    sendStatusPacket();
  }

  else if (strcmp(
               command,
               "SAMPLE") == 0) {

    sampleAllDevices();
    sendDataPacket();
  }

  else if (strcmp(
               command,
               "STREAM ON") == 0) {

    streamEnabled = true;

    Serial.println(
        "OK,stream=ON");
  }

  else if (strcmp(
               command,
               "STREAM OFF") == 0) {

    streamEnabled = false;

    Serial.println(
        "OK,stream=OFF");
  }

  else if (strcmp(
               command,
               "REINIT") == 0) {

    reinitializeSensors();
  }

  else if (strcmp(
               command,
               "HELP") == 0) {

    sendHelp();
  }

  else if (strncmp(
               command,
               "RATE ",
               5) == 0) {

    uint32_t requestedRate =
        atol(command + 5);

    if (requestedRate < 100 ||
        requestedRate > 60000) {

      Serial.println(
          "ERROR,RATE_MUST_BE_100_TO_60000_MS");
    } else {
      sampleIntervalMs =
          requestedRate;

      Serial.print(
          "OK,sample_rate_ms=");

      Serial.println(
          sampleIntervalMs);
    }
  }

  else if (strncmp(
               command,
               "AVG ",
               4) == 0) {

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
           sensor <
               TEMPERATURE_SENSOR_COUNT;
           sensor++) {

        clearTemperatureAverage(sensor);
      }

      Serial.print("OK,averaging=");
      Serial.println(averagingSamples);
    }
  }

  else if (command[0] != '\0') {
    Serial.println(
        "ERROR,UNKNOWN_COMMAND");
  }
}

// ==================================================
// Receive commands
// ==================================================

void receiveCommands() {
  while (Serial.available() > 0) {
    char incomingCharacter =
        Serial.read();

    if (incomingCharacter == '\r') {
      continue;
    }

    if (incomingCharacter == '\n') {
      commandBuffer[commandLength] =
          '\0';

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
      "HIFU End Effector Sensor Controller");

  Serial.println(
      "===================================");

  initializeAllTemperatureSensors();
  initializeAllLoadSensors();

  sampleAllDevices();

  sendHelp();
}

// ==================================================
// Arduino main loop
// ==================================================

void loop() {
  receiveCommands();

  static uint32_t
      lastAutomaticSampleMs = 0;

  if (millis() -
          lastAutomaticSampleMs >=
      sampleIntervalMs) {

    lastAutomaticSampleMs = millis();

    sampleAllDevices();

    if (streamEnabled) {
      sendDataPacket();
    }
  }

  checkForStaleData();
}