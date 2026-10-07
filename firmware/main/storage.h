#pragma once

// =============================================================================
//  storage.h - map files on the internal flash
// =============================================================================
//
//  FAT filesystem with wear levelling on the "storage" partition (see
//  partitions.csv), mounted at /fs and used through standard C file I/O.
// =============================================================================

#include "esp_err.h"

#define STORAGE_RECORDED_MAP_PATH   "/fs/gruzik.txt"   // GRUZIK.txt in the app
#define STORAGE_PLAYBACK_MAP_PATH   "/fs/map.txt"

esp_err_t storage_init(void);
