#pragma once

// ---- Servos ----
#define PIN_SERVO_PAN   13   // GPIO13 -> MG90S #1 orange (pan)
#define PIN_SERVO_TILT  12   // GPIO12 -> MG90S #2 orange (tilt)

// ---- UART link to Core ESP32 ----
// NOTE: these are the CAM's native USB-serial pins (UART0). Once wired to
// Core, the normal `idf.py monitor` serial log will not show CAM output
// unless UART is physically disconnected during debugging, or logging is
// moved to WiFi/telnet later.
#define PIN_UART_TX     1    // GPIO1 -> Core RX
#define PIN_UART_RX     3    // GPIO3 -> Core TX

#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200

// ---- SD card (SDMMC 1-bit mode, onboard wiring, no extra GPIO config) ----
// CLK = GPIO14, CMD = GPIO15, D0 = GPIO2 (handled by SDMMC driver, not here)