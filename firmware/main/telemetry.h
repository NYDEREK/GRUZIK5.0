#pragma once

// =============================================================================
//  telemetry.h - periodic data stream for the app
// =============================================================================
//
//  Telemetry=odom  -> ODOM line every 100 ms + battery every second
//  Telemetry=debug -> additionally a DBG line with line sensors, encoders
//                     and gyro
// =============================================================================

void telemetry_init(void);

// Greeting sent when the phone connects.
void telemetry_send_hello(void);
