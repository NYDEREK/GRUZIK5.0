// =============================================================================
//  storage.c - FAT on flash
// =============================================================================

#include "storage.h"

#include "esp_vfs_fat.h"
#include "wear_levelling.h"

static wl_handle_t s_wl = WL_INVALID_HANDLE;

esp_err_t storage_init(void)
{
    // An empty or corrupted partition is formatted on the first boot.
    const esp_vfs_fat_mount_config_t cfg = {
        .format_if_mount_failed = true,
        .max_files = 4,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
    };
    return esp_vfs_fat_spiflash_mount_rw_wl("/fs", "storage", &cfg, &s_wl);
}
