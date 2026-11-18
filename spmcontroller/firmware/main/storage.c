#include "storage.h"
#include "nor_log.h"

#include <string.h>
#include <sys/stat.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>

#define TAG "storage"
#define NS "storage"

esp_err_t storage_nvs_init(void) {
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition damaged/outdated - erasing and re-initialising");
        nvs_flash_erase();
        r = nvs_flash_init();
    }
    if (r != ESP_OK) ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(r));
    return r;
}

uint32_t storage_session_begin(void) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return 1;
    uint32_t n = 0;
    nvs_get_u32(h, "session", &n);
    n++;
    nvs_set_u32(h, "session", n);
    nvs_commit(h);
    nvs_close(h);
    return n;
}

void storage_session_floor(uint32_t n) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    uint32_t cur = 0;
    nvs_get_u32(h, "session", &cur);
    if (n > cur) { nvs_set_u32(h, "session", n); nvs_commit(h); }
    nvs_close(h);
}

// ---- boot log -------------------------------------------------------------------
#define BOOT_LOG_ENTRIES 10
#define BL_F_NOR_OK     0x01u
#define BL_F_SD_OK      0x02u
#define BL_F_DUMP_DONE  0x04u
#define BL_F_DUMP_NONE  0x08u
#define BL_F_FINISHED   0x10u

typedef struct __attribute__((packed)) {
    uint32_t session;
    uint32_t nor_pending;
    uint8_t  reset_reason;
    uint8_t  flags;
    uint16_t _pad;
} bl_entry_t;

typedef struct __attribute__((packed)) {
    uint8_t count, next;
    uint16_t _pad;
    bl_entry_t e[BOOT_LOG_ENTRIES];
} bootlog_t;

_Static_assert(sizeof(bootlog_t) == 124, "bootlog size");

static const char *const RESET_NAMES[] = {
    "UNKNOWN","POWERON","EXT","SW","PANIC","INT_WDT","TASK_WDT","WDT",
    "DEEPSLEEP","BROWNOUT","SDIO","USB","JTAG","EFUSE","PWR_GLITCH","CPU_LOCKUP"
};
static const char *reset_name(uint8_t r) {
    return r < sizeof(RESET_NAMES) / sizeof(RESET_NAMES[0]) ? RESET_NAMES[r] : "?";
}

static uint8_t s_slot = 0xFF;

static bool bl_load(bootlog_t *bl) {
    memset(bl, 0, sizeof(*bl));
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = sizeof(*bl);
    esp_err_t r = nvs_get_blob(h, "bootlog", bl, &len);
    nvs_close(h);
    if (r != ESP_OK || len != sizeof(*bl)) { memset(bl, 0, sizeof(*bl)); return false; }
    return true;
}

static void bl_store(const bootlog_t *bl) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, "bootlog", bl, sizeof(*bl)) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}

void storage_bootlog_begin(uint32_t session, bool nor_ok, uint32_t nor_pending) {
    bootlog_t bl;
    bl_load(&bl);
    if (bl.next >= BOOT_LOG_ENTRIES) bl.next = 0;
    bl_entry_t *e = &bl.e[bl.next];
    memset(e, 0, sizeof(*e));
    e->session = session;
    e->reset_reason = (uint8_t)esp_reset_reason();
    e->nor_pending = nor_ok ? nor_pending : 0;
    e->flags = nor_ok ? BL_F_NOR_OK : 0;
    s_slot = bl.next;
    bl.next = (uint8_t)((bl.next + 1) % BOOT_LOG_ENTRIES);
    if (bl.count < BOOT_LOG_ENTRIES) bl.count++;
    bl_store(&bl);

    ESP_LOGI(TAG, "---- boot log: last %u boots, oldest first ----", bl.count);
    for (uint8_t i = 0; i < bl.count; i++) {
        uint8_t idx = (uint8_t)((bl.next + BOOT_LOG_ENTRIES - bl.count + i) % BOOT_LOG_ENTRIES);
        const bl_entry_t *r = &bl.e[idx];
        const char *dump = (idx == s_slot) ? "pending" : (r->flags & BL_F_DUMP_NONE) ? "none"
                         : (r->flags & BL_F_DUMP_DONE) ? "DONE" : (r->nor_pending > 0) ? "INCOMPLETE" : "-";
        ESP_LOGI(TAG, "  sess %-6lu reset=%-9s nor_pending=%-9lu dump=%-10s sd=%s%s",
                 (unsigned long)r->session, reset_name(r->reset_reason), (unsigned long)r->nor_pending, dump,
                 !(r->flags & BL_F_FINISHED) ? "?" : (r->flags & BL_F_SD_OK) ? "ok" : "FAIL",
                 idx == s_slot ? "   <= this boot" : "");
    }
}

void storage_bootlog_finish(bool sd_ok, bool dump_done, bool nothing_to_dump) {
    if (s_slot >= BOOT_LOG_ENTRIES) return;
    bootlog_t bl;
    if (!bl_load(&bl)) return;
    bl_entry_t *e = &bl.e[s_slot];
    e->flags |= BL_F_FINISHED;
    if (sd_ok) e->flags |= BL_F_SD_OK;
    if (dump_done) e->flags |= BL_F_DUMP_DONE;
    if (nothing_to_dump) e->flags |= BL_F_DUMP_NONE;
    bl_store(&bl);
}

// ---- NOR -> SD dump at boot -------------------------------------------------------
bool storage_boot_dump_and_wipe(const char *mount_point, uint32_t session) {
    if (!nor_log_ok()) {
        ESP_LOGW(TAG, "NOR not ready - no dump, no wipe");
        storage_bootlog_finish(true, false, false);
        return false;
    }
    uint32_t pending = nor_log_bytes_pending();
    if (pending == 0) {
        ESP_LOGI(TAG, "NOR empty - nothing to dump");
        storage_bootlog_finish(true, false, true);
        return true;
    }

    char path[64];
    struct stat st;
    snprintf(path, sizeof(path), "%s/nor_dump_%06lu.bin", mount_point, (unsigned long)session);
    for (uint32_t suffix = 1; stat(path, &st) == 0; suffix++) {
        ESP_LOGW(TAG, "%s exists - using a suffixed name", path);
        snprintf(path, sizeof(path), "%s/nor_dump_%06lu_%lu.bin", mount_point, (unsigned long)session, (unsigned long)suffix);
    }

    ESP_LOGI(TAG, "dumping NOR backlog (%lu bytes) to %s ...", (unsigned long)pending, path);
    esp_err_t r = nor_log_dump_to_file(path);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "NOR dump FAILED (%s) - NOR is NOT wiped, data will be re-dumped next boot", esp_err_to_name(r));
        storage_bootlog_finish(true, false, false);
        return false;
    }
    storage_bootlog_finish(true, true, false);
    if (nor_log_wipe() != ESP_OK) {
        ESP_LOGE(TAG, "NOR wipe failed - data stays; dump will repeat next boot");
        return false;
    }
    return true;
}

FILE *storage_open_session_file(const char *mount_point, uint32_t *session, size_t buffer_size) {
    char path[64];
    struct stat st;
    for (;;) {
        snprintf(path, sizeof(path), "%s/session_%06lu.bin", mount_point, (unsigned long)*session);
        if (stat(path, &st) != 0) break;
        ESP_LOGW(TAG, "%s exists - advancing session number (NVS reset, or card from another board?)", path);
        (*session)++;
    }
    storage_session_floor(*session);
    FILE *f = fopen(path, "wb");
    if (!f) { ESP_LOGE(TAG, "cannot open %s", path); return NULL; }
    if (buffer_size) setvbuf(f, NULL, _IOFBF, buffer_size);
    ESP_LOGI(TAG, "logging to %s", path);
    return f;
}
