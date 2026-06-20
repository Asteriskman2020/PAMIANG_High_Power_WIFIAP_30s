#ifndef MEMORY_H_
#define MEMORY_H_

#include "FS.h"
#include <LittleFS.h>
#include "sensor_v2.h"

class Memory {
  public:
    /** @brief Write File If no file is detected it call it. It will generate File. */
    void write(fs::FS &fs, const char * path, const char * message);

    /** @brief Insert the data to the path file. */
    bool append(fs::FS &fs, const char * path, const char * message);
    
    /** @brief Save sensor data to LittleFS (with optional BLE data) */
    bool saveData(const char* fileName, timeStruct* time_val, SensorData* sensor_val,
                  BleSensorData* ble_val = nullptr);

    /** @brief Check available space */
    uint64_t getAvailableSpace();
    
    /** @brief Check if file is too large */
    bool isFileTooLarge(const char* path, uint32_t maxSizeBytes);

    /** @brief Count data lines in CSV file (excluding header) */
    int countDataLines(const char* fileName);

    /** @brief Read data records from CSV file back into DataRecord structs */
    bool readDataRecords(const char* fileName, DataRecord* records, int maxRecords);

    /** @brief Read one data record by zero-based CSV data-row index, excluding header */
    bool readDataRecordAt(const char* fileName, uint16_t dataLineIndex, DataRecord* record);

    /** @brief Remove already delivered data rows while preserving later buffered rows */
    bool removeFirstDataLines(const char* fileName, uint16_t linesToRemove);

    /** @brief Move the oldest data row to the end of the CSV queue */
    bool moveFirstDataLineToEnd(const char* fileName);

    /** @brief Remove delivered data rows by zero-based CSV data-row indexes, excluding header */
    bool removeDataLines(const char* fileName, const uint16_t* lineIndexes, uint16_t indexCount);

    /** @brief Remove rows older than maxAgeMinutes compared with a reference record */
    uint16_t removeDataOlderThan(const char* fileName, const DataRecord& reference,
                                 uint16_t maxAgeMinutes);

    /** @brief Check if LittleFS usage exceeds given percentage */
    bool isUsageOverThreshold(uint8_t thresholdPercent);

    /** @brief Clear all data rows from a CSV file, preserving the header */
    bool clearDataRows(const char* fileName);
};

#endif
