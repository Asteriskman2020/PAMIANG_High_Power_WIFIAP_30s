/**
 * @file    main.cpp
 * @brief   Entry point for PAMIANG High-Power Weather Station (TX Board).
 *
 * Delegates all application logic to AppManager.
 * This file intentionally contains only setup() and loop().
 */


/* Sent 1 Package in 1 time not stack */
#include "AppManager.h"

static AppManager app;

void setup() {
    app.setup();
}

void loop() {
    app.loop();
}
