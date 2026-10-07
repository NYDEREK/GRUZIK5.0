#pragma once

// =============================================================================
//  link.h - Wi-Fi access point and TCP command link
// =============================================================================
//
//  The robot hosts its own access point (SSID/password in menuconfig) and a
//  TCP server on 192.168.4.1. The app joins this network without internet,
//  so the phone keeps mobile data for everything else.
//
//  Protocol: plain text lines ending with '\n', "Key=Value" from the app,
//  replies and telemetry lines back. One client at a time; a new connection
//  replaces the old one.
// =============================================================================

#include <stdbool.h>
#include "esp_err.h"

typedef void (*link_line_handler_t)(const char *line);
typedef void (*link_event_handler_t)(void);

esp_err_t link_init(link_line_handler_t on_line, link_event_handler_t on_connect,
                    link_event_handler_t on_disconnect);

bool link_connected(void);

// Queues text for the phone and mirrors it to the USB log. Never blocks; text
// is dropped if the transmit buffer is full.
void link_send(const char *text);

// Like link_send, but waits for buffer space (bulk transfers such as map
// dumps). Command context only.
void link_send_blocking(const char *text);

// Telemetry: not mirrored to the USB log, dropped when the buffer is full.
void link_send_quiet(const char *text);
