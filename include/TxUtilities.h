/* TX Board utility functions — time, CSV, WDT, battery, runtime config.
 * Extracted from main.cpp to keep only setup()/loop() there. */
#ifndef TX_UTILITIES_H_
#define TX_UTILITIES_H_

#include <Arduino.h>
#include <Preferences.h>
#include "sensor_v2.h"
#include "Memory.h"

/* Context: pointers to globals in main.cpp.
 * Fill in setup() before calling begin(). */
struct TxUtilsCtx {
    Preferences* nvs;
    Memory*      memory;
    RS485sensor* modbusSensor;
    timeStruct*  currentTime;
    char*        dailyCsv;          /* main.cpp dailyCsv[24] buffer */
};

class TxUtils {
public:
    TxUtils();

    /* Call once in setup() after filling TxUtilsCtx */
    void begin(TxUtilsCtx* ctx);

    /* ── WDT ── */
    void feedWDT();

    /* ── Weather ── */
    void clearWeatherData();

    /* ── Time ── */
    void incrementTime(timeStruct* t, uint16_t addMinutes);

    /* ── File helpers ── */
    bool   readLastTimeFromBackup(timeStruct* outTime);
    String getLastDataLine(const char* fileName);
    void   updateDailyCsv();

    /* ── NVS runtime config ── */
    void loadRuntimeConfig(uint8_t* soilId, uint8_t* weathId,
                           uint8_t* batchSize, uint8_t* pubMode);

    /* ── TPL5110 done pin ── */
    void pulseDoneTPL5110();

private:
    TxUtilsCtx* _ctx;
};

#endif
