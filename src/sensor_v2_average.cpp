#include "sensor_v2.h"
#include <esp_task_wdt.h>

/* Average-processing variant of sensor_v2.cpp.
 * Function names stay getMedian/getMedian32 so this file can be selected
 * by build_src_filter without changing the shared sensor_v2.h API.
 */
uint16_t dataProcess::getMedian(uint16_t* values, uint8_t count) {
  if (count == 0) return 0;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < count; i++) sum += values[i];

  return (uint16_t)((sum + (count / 2)) / count);
}

uint32_t dataProcess::getMedian32(uint32_t* values, uint8_t count) {
  if (count == 0) return 0;

  uint64_t sum = 0;
  for (uint8_t i = 0; i < count; i++) sum += values[i];

  return (uint32_t)((sum + (count / 2)) / count);
}

static uint16_t getMedianStable16(uint16_t* values, uint8_t count) {
  if (count == 0) return 0;
  if (count == 1) return values[0];

  uint16_t sorted[soilReadAttempt];
  for (uint8_t i = 0; i < count; i++) sorted[i] = values[i];

  for (uint8_t i = 0; i < count - 1; i++) {
    for (uint8_t j = 0; j < count - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        uint16_t tmp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = tmp;
      }
    }
  }

  if (count % 2 == 1) return sorted[count / 2];
  return (uint16_t)(((uint32_t)sorted[count / 2 - 1] + sorted[count / 2]) / 2);
}

static void clearSensorSection(uint8_t sensorType, SensorData* data) {
  if (data == nullptr) return;

  if (sensorType == SOIL) {
    data->soil_humi = 0;
    data->soil_temp = 0;
    data->soil_ec = 0;
    data->soil_ph = 0;
    data->soil_N = 0;
    data->soil_P = 0;
    data->soil_K = 0;
  } else if (sensorType == WEATHER) {
    data->windSpeed = 0;
    data->windDir_Deg = 0;
    data->air_humidity = 0;
    data->air_temperature = 0;
    data->CO2 = 0;
    data->pressure = 0;
    data->illuminance = 0;
    data->rainfall = 0;
    data->solar = 0;
  }
}

void RS485sensor::begin(Stream* serialPort) {
  _serial = serialPort;

  modbus.preTransmission([]() {});
  modbus.postTransmission([]() { delayMicroseconds(500); });
}

void RS485sensor::_uartRecovery(Stream* serialPort) {
  HardwareSerial* hwSerial = static_cast<HardwareSerial*>(serialPort);
  hwSerial->end();
  delay(100);
  hwSerial->begin(SERIAL_RS485, SERIAL_8N1, RS_485_RX_PIN, RS_485_TX_PIN);
  delay(500);

  while (serialPort->available()) { serialPort->read(); }
  serialPort->flush();
  delay(200);

  modbus.preTransmission([]() {});
  modbus.postTransmission([]() { delayMicroseconds(1000); });
}

void RS485sensor::_flushAndSettle(Stream* serialPort) {
  while (serialPort->available()) { serialPort->read(); }
  serialPort->flush();
  delay(200);
}

bool RS485sensor::read(uint8_t sensorType, uint8_t slaveID, uint16_t address, uint16_t length, Stream* serialPort) {

  uint8_t startAttempt = 0;
  uint8_t retry_FLAGS = 0;
  uint8_t result;
  const uint8_t targetReadAttempt = (sensorType == SOIL) ? soilReadAttempt : readAttempt;
  const uint16_t sampleDelayMs = (sensorType == SOIL) ? SOIL_SAMPLE_DELAY_MS : SENSOR_SAMPLE_DELAY_MS;
  uint16_t rawBuffer[soilReadAttempt][16] = {0};

  if (!_uartReady) {
    _uartRecovery(serialPort);
    _uartReady = true;
  } else {
    _flushAndSettle(serialPort);
  }

  modbus.begin(slaveID, *serialPort);

  while (startAttempt < targetReadAttempt && retry_FLAGS < maxRetry) {
    esp_task_wdt_reset() ;
    result = modbus.readHoldingRegisters(address, length);
    if (result == modbus.ku8MBSuccess) {
      for (int cnt = 0; cnt < length; cnt++) {
        if (cnt < 16) {
          rawBuffer[startAttempt][cnt] = modbus.getResponseBuffer(cnt);
        }
      }
      delay(sampleDelayMs);
      modbus.clearResponseBuffer();
      startAttempt++;
    }
    else {
      retry_FLAGS++;
      Serial.printf("[MODBUS] Slave 0x%02X read FAILED, err=0x%02X (retry %u/%u)\n",
                     slaveID, result, retry_FLAGS, maxRetry);

      if (result == 0xE0) {
        Serial.println("[MODBUS] InvalidSlaveID â€” full UART recovery");
        _uartRecovery(serialPort);
      } else {
        delay(1000);
      }
      modbus.begin(slaveID, *serialPort);
    }
  }

  if (startAttempt == 0) {
    _consecutiveFailCount++;
    Serial.printf("[ERROR] No successful reads! (consecutive: %u/%u)\n", _consecutiveFailCount, _maxConsecutiveFail);
    if (_consecutiveFailCount >= _maxConsecutiveFail) {
      Serial.println("[WARN] Max consecutive failures reached. Clearing failed sensor section.");
      clearSensorSection(sensorType, &currentSensor);
      _consecutiveFailCount = 0;
      return true;
    }
    return false;
  }

  Serial.printf("[MODBUS] Slave 0x%02X successful samples: %u/%u\n",
                slaveID, startAttempt, targetReadAttempt);

  _consecutiveFailCount = 0;

  if (sensorType == SOIL) {
    currentSensor.soil_humi = 0;
    currentSensor.soil_temp = 0;
    currentSensor.soil_ec = 0;
    currentSensor.soil_ph = 0;
    currentSensor.soil_N = 0;
    currentSensor.soil_P = 0;
    currentSensor.soil_K = 0;

    uint16_t buf[soilReadAttempt];

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][0];
    currentSensor.soil_humi = postProcessing.getMedian(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][1];
    currentSensor.soil_temp = postProcessing.getMedian(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][2];
    currentSensor.soil_ec = getMedianStable16(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][3];
    uint16_t ph_median = postProcessing.getMedian(buf, startAttempt);
    if (ph_median > 255) ph_median = 255;
    currentSensor.soil_ph = (uint8_t)ph_median;

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][4];
    currentSensor.soil_N = getMedianStable16(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][5];
    currentSensor.soil_P = getMedianStable16(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][6];
    currentSensor.soil_K = getMedianStable16(buf, startAttempt);

    return true;
  }

  else if (sensorType == WEATHER) {
    currentSensor.windSpeed = 0;
    currentSensor.windDir_Deg = 0;
    currentSensor.air_humidity = 0;
    currentSensor.air_temperature = 0;
    currentSensor.CO2 = 0;
    currentSensor.pressure = 0;
    currentSensor.illuminance = 0;
    currentSensor.rainfall = 0;
    currentSensor.solar = 0;

    uint16_t buf[soilReadAttempt];
    uint32_t buf32[soilReadAttempt];

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][0];
    currentSensor.windSpeed = postProcessing.getMedian(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][3];
    currentSensor.windDir_Deg = postProcessing.getMedian(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][4];
    currentSensor.air_humidity = postProcessing.getMedian(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][5];
    currentSensor.air_temperature = postProcessing.getMedian(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][7];
    currentSensor.CO2 = postProcessing.getMedian(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][9];
    currentSensor.pressure = postProcessing.getMedian(buf, startAttempt);

    for (uint8_t i = 0; i < startAttempt; i++) {
      buf32[i] = ((uint32_t)rawBuffer[i][10] << 16) | rawBuffer[i][11];
    }
    currentSensor.illuminance = postProcessing.getMedian32(buf32, startAttempt);

    currentSensor.rainfall = rawBuffer[startAttempt - 1][13];

    for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][15];
    currentSensor.solar = postProcessing.getMedian(buf, startAttempt);

    uint8_t zeroRetry = 0;
    while ((currentSensor.air_humidity == 0 ||
            currentSensor.air_temperature == 0 ||
            currentSensor.CO2 == 0) && zeroRetry < maxRetry) {
      esp_task_wdt_reset() ;
      zeroRetry++;
      Serial.print("[WARN] Air humidity/temp/CO2 zero, retry #");
      Serial.println(zeroRetry);
      delay(1500) ;
      startAttempt = 0;
      retry_FLAGS = 0;
      memset(rawBuffer, 0, sizeof(rawBuffer));

      _flushAndSettle(serialPort);
      modbus.begin(slaveID, *serialPort);

      while (startAttempt < targetReadAttempt && retry_FLAGS < maxRetry) {
        esp_task_wdt_reset() ;
        result = modbus.readHoldingRegisters(address, length);
        if (result == modbus.ku8MBSuccess) {
          for (int cnt = 0; cnt < length; cnt++) {
            if (cnt < 16) {
              rawBuffer[startAttempt][cnt] = modbus.getResponseBuffer(cnt);
            }
          }
          delay(100);
          modbus.clearResponseBuffer();
          startAttempt++;
        }
        else {
          retry_FLAGS++;
          delay(1000);
        }
      }

      if (startAttempt == 0) break;

      for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][0];
      currentSensor.windSpeed = postProcessing.getMedian(buf, startAttempt);

      for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][3];
      currentSensor.windDir_Deg = postProcessing.getMedian(buf, startAttempt);

      for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][4];
      currentSensor.air_humidity = postProcessing.getMedian(buf, startAttempt);

      for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][5];
      currentSensor.air_temperature = postProcessing.getMedian(buf, startAttempt);

      for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][7];
      currentSensor.CO2 = postProcessing.getMedian(buf, startAttempt);

      for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][9];
      currentSensor.pressure = postProcessing.getMedian(buf, startAttempt);

      for (uint8_t i = 0; i < startAttempt; i++) {
        buf32[i] = ((uint32_t)rawBuffer[i][10] << 16) | rawBuffer[i][11];
      }
      currentSensor.illuminance = postProcessing.getMedian32(buf32, startAttempt);

      currentSensor.rainfall = rawBuffer[startAttempt - 1][13];

      for (uint8_t i = 0; i < startAttempt; i++) buf[i] = rawBuffer[i][15];
      currentSensor.solar = postProcessing.getMedian(buf, startAttempt);
    }

    if (currentSensor.air_humidity == 0 ||
        currentSensor.air_temperature == 0 ||
        currentSensor.CO2 == 0) {
      Serial.println("[ERROR] Air humidity/temp/CO2 still zero after 3 retries");
    }

    return true;
  }

  return false;
}

bool RS485sensor::write(uint8_t sensorType, uint8_t slaveID, uint16_t address, uint16_t value, Stream* serialPort) {
  uint8_t retry_FLAGS = 0;
  uint8_t result;
  modbus.begin(slaveID, *serialPort);

  while (retry_FLAGS < maxRetry) {
    result = modbus.writeSingleRegister(address, value);
    if (result == modbus.ku8MBSuccess) {
      Serial.println("Write Success");
      return true;
    }
    else {
      retry_FLAGS++;
      delay(50);
    }
  }
  return false;
}

uint16_t batteryRead() {
    esp_task_wdt_reset() ;
    delay(1000) ;
    esp_task_wdt_reset() ;
    Serial.println("Reading Battery Voltage...");
    uint32_t sumMilliVolts = 0;
    const int numSamples = 64;
    for(int i = 0; i < numSamples; i++) {
      sumMilliVolts += analogReadMilliVolts(A0);
      delay(2);
    }
    float pinVoltage = (sumMilliVolts / (float)numSamples) / 1000.0;
    float dividerRatio = (float)(R1 + R2) / (float)R2 ;
    float calibrationFactor = 1.000;
    float batteryVoltage = pinVoltage * dividerRatio * calibrationFactor;
    uint16_t batteryMilliVolts = (uint16_t)(batteryVoltage * 1000.0);
    Serial.printf("Pin A0: %.3f V | Battery: %.3f V (%u mV)\n", pinVoltage, batteryVoltage, batteryMilliVolts);
    return batteryMilliVolts;
}
