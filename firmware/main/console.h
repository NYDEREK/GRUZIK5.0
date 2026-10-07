#pragma once

// =============================================================================
//  console.h - USB serial console for bench tests
// =============================================================================
//
//  Accepts the same text commands as the app (e.g. "Manual=150,0") and, while
//  a USB host is attached, prints a status report every 2 s: battery, timing,
//  motor outputs, encoders and line sensors.
// =============================================================================

void console_init(void);
