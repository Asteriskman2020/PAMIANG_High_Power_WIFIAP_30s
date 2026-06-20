#ifndef RS485SENSOR_H_
#define RS485SENSOR_H_

#include "utilities.h"
#include "ModbusMaster.h"

typedef enum {
  SOIL = 1,
  WEATHER = 2
} sensorList;

typedef struct {
  uint8_t date;
  uint8_t month;
  uint16_t year;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  char dateStr[12];
  char timeStr[9];
} timeStruct;

typedef struct __attribute__((packed)) {
  // --- Soil Sensor (13 Bytes) ---
  uint16_t soil_humi;      
  int16_t  soil_temp;      
  uint16_t soil_ec;        
  uint8_t  soil_ph;        
  uint16_t soil_N;         
  uint16_t soil_P;         
  uint16_t soil_K;         
  
  // --- Weather Station (20 Bytes) ---
  uint16_t windSpeed;       
  uint16_t windDir_Deg;     
  uint16_t air_humidity;    
  int16_t  air_temperature; 
  uint16_t CO2;             
  uint16_t pressure;        
  uint32_t illuminance;     
  uint16_t rainfall;        
  uint16_t solar;           
} SensorData;

/* BLE NUS sensor data (from Sniffer Portal) */
typedef struct __attribute__((packed)) {
  int16_t  ble_temp;    // temperature * 10
  uint16_t ble_humi;    // humidity * 10
  int16_t  ble_tmp117;  // TMP117 * 10
  uint16_t ble_rain;
  uint16_t ble_leaf;
  uint16_t ble_par;
  uint16_t ble_soil;
} BleSensorData;

typedef struct __attribute__((packed)) {
  uint8_t date;
  uint8_t month;
  uint8_t year;
  uint8_t hour;
  uint8_t minute;
  SensorData data;
  BleSensorData ble;
  uint8_t ble_valid;     // 1 if BLE data present
  uint8_t valid;
} DataRecord;

typedef SensorData soilData;
typedef SensorData weatherData;
typedef DataRecord Packet;

class dataProcess {
  public:
    /** @brief Calculate a Median of three values. */
    uint16_t getMedian(uint16_t* values, uint8_t count);
    
    /** @brief Calculate a Median of three values (32-bit version). */
    uint32_t getMedian32(uint32_t* values, uint8_t count);
};

class RS485sensor {
  public: 
    SensorData currentSensor;
    
    /** @brief Send the Serial2 in this class. */
    void begin(Stream* serialPort);
    
    /** @brief Read a data with Modbus RS485 Protocol */
    bool read(uint8_t sensorType, uint8_t slaveID, uint16_t address, uint16_t length, Stream* serialPort);
    
    /** @brief Write the register in order to setting the RS485 Sensor. */
    bool write(uint8_t sensorType, uint8_t slaveID, uint16_t address, uint16_t value, Stream* serialPort);
    
  private:
    ModbusMaster modbus;
    dataProcess postProcessing;
    Stream* _serial = nullptr;
    uint8_t _consecutiveFailCount = 0;
    static const uint8_t _maxConsecutiveFail = 3;
    bool _uartReady = false;
    void _uartRecovery(Stream* serialPort);
    void _flushAndSettle(Stream* serialPort);
};

/** @brief  */
uint16_t batteryRead() ;

#endif
