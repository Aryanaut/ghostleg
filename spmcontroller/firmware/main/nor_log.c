#include "nor_log.h"

#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <esp_flash.h>
#include <esp_flash_spi_init.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <nvs.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define TAG "nor_log"

#define NOR_SECTOR            4096u
#define NOR_PAGE              256u
#define NOR_FORMAT_VERSION    1u       // bump => one-time full chip erase on next boot
// Persisting the pointer writes the ESP32's internal flash, which stalls both
// cores for ~10-20 ms (cache disabled) - on the Training Board that is a
// visible IMU-stream hiccup. The pointer only bounds the blank-page scan at
// the next boot (scan_for_end recovers everything written since), so commit
// rarely: every N records or T ms, whichever comes first.
#define NOR_COMMIT_RECORDS    4096u
#define NOR_COMMIT_MS         60000u

static esp_flash_t *s_flash = NULL;
static bool         s_ok = false;
static uint32_t     s_size = 0;        // chip size = end of data region
static uint32_t     s_pos = NOR_LOG_DATA_OFFSET;   // next byte to write
static uint32_t     s_clean = NOR_LOG_DATA_OFFSET; // [s_pos, s_clean) is known erased
static uint32_t     s_high_water = NOR_LOG_DATA_OFFSET; // highest offset ever written since the last wipe
static nvs_handle_t s_nvs = 0;

typedef struct {
    uint16_t len;
    uint8_t  data[NOR_LOG_MAX_RECORD];
} nor_entry_t;

static QueueHandle_t s_queue = NULL;
static volatile uint32_t s_records = 0, s_drops = 0, s_write_err = 0;
static volatile uint32_t s_max_write_us = 0;

static inline uint32_t align_up(uint32_t v, uint32_t a) { return (v + a - 1) / a * a; }

// ---- NVS ------------------------------------------------------------------
static void nvs_store_pointers(void) {
    if (s_pos > s_high_water) s_high_water = s_pos;
    nvs_set_u32(s_nvs, "nor_hw", s_high_water);
    esp_err_t r1 = nvs_set_u32(s_nvs, "nor_pos", s_pos);
    esp_err_t r2 = nvs_set_u32(s_nvs, "nor_clean", s_clean);
    esp_err_t r3 = nvs_commit(s_nvs);
    if (r1 != ESP_OK || r2 != ESP_OK || r3 != ESP_OK) {
        ESP_LOGE(TAG, "NVS pointer store failed: pos %s, clean %s, commit %s",
                 esp_err_to_name(r1), esp_err_to_name(r2), esp_err_to_name(r3));
    }
}

static void nvs_load_pointers(void) {
    uint32_t pos = NOR_LOG_DATA_OFFSET, clean = NOR_LOG_DATA_OFFSET, hw = NOR_LOG_DATA_OFFSET;
    nvs_get_u32(s_nvs, "nor_pos", &pos);
    nvs_get_u32(s_nvs, "nor_clean", &clean);
    nvs_get_u32(s_nvs, "nor_hw", &hw);
    if (pos < NOR_LOG_DATA_OFFSET || pos > s_size) pos = NOR_LOG_DATA_OFFSET;
    if (clean < pos || clean > s_size) clean = align_up(pos, NOR_SECTOR);   // unknown => rely on JIT erase
    if (hw < pos || hw > s_size) hw = pos;
    s_pos = pos;
    s_clean = clean;
    s_high_water = hw;
}

// ---- recovery: find the true end of data after the saved pointer -----------
// The NVS pointer is committed every few records; anything written after the
// last commit is found by walking page by page until a blank (0xFF) page.
static esp_err_t scan_for_end(void) {
    static uint8_t page[NOR_PAGE];
    uint32_t start = s_pos;
    uint32_t limit = s_clean > s_pos ? s_clean : s_pos;
    while (s_pos < limit) {
        uint32_t n = NOR_PAGE - (s_pos % NOR_PAGE);
        if (n > limit - s_pos) n = limit - s_pos;
        esp_err_t r = esp_flash_read(s_flash, page, s_pos, n);
        if (r != ESP_OK) return r;
        bool blank = true;
        for (uint32_t i = 0; i < n; i++) if (page[i] != 0xFF) { blank = false; break; }
        if (blank) break;
        s_pos += n;
    }
    if (s_pos != start) {
        ESP_LOGW(TAG, "write pointer advanced by scan: 0x%08lx -> 0x%08lx (%lu bytes were written after the last NVS commit)",
                 (unsigned long)start, (unsigned long)s_pos, (unsigned long)(s_pos - start));
        nvs_store_pointers();
    }
    return ESP_OK;
}

// ---- erase helpers ----------------------------------------------------------
// Sectors above the clean limit are normally still blank from the format erase
// or an earlier wipe (the limit only collapses when the NVS copy was stale, e.g.
// after an interrupted wipe). Reading 4 KB costs ~1 ms of bus time; erasing a
// sector costs 15-45 ms (up to 400 ms). So check before erasing.
static bool sector_is_blank(uint32_t addr) {
    static uint8_t buf[512];
    for (uint32_t off = 0; off < NOR_SECTOR; off += sizeof(buf)) {
        if (esp_flash_read(s_flash, buf, addr + off, sizeof(buf)) != ESP_OK) return false;
        for (uint32_t i = 0; i < sizeof(buf); i++) if (buf[i] != 0xFF) return false;
    }
    return true;
}

// Make sure [s_pos, s_pos+len) is erased. Normally a no-op: the boot wipe
// leaves the whole region clean.
static esp_err_t ensure_clean(uint32_t len) {
    uint32_t need = s_pos + len;
    // The remainder of the sector containing s_pos was erased when that sector
    // was first written (or wiped), so it is clean beyond s_pos.
    uint32_t cur_sector_end = (s_pos % NOR_SECTOR == 0) ? s_pos : align_up(s_pos, NOR_SECTOR);
    if (s_clean < cur_sector_end) s_clean = cur_sector_end;
    while (s_clean < need) {
        if (sector_is_blank(s_clean)) {
            s_clean += NOR_SECTOR;
            continue;
        }
        int64_t t0 = esp_timer_get_time();
        esp_err_t r = esp_flash_erase_region(s_flash, s_clean, NOR_SECTOR);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "JIT sector erase at 0x%08lx failed: %s", (unsigned long)s_clean, esp_err_to_name(r));
            return r;
        }
        ESP_LOGW(TAG, "JIT erased sector 0x%08lx (%lld us) - a previous wipe was interrupted",
                 (unsigned long)s_clean, (long long)(esp_timer_get_time() - t0));
        s_clean += NOR_SECTOR;
    }
    return ESP_OK;
}

// ---- init -------------------------------------------------------------------
esp_err_t nor_log_init(const nor_log_config_t *cfg) {
    esp_err_t r;

    if (cfg->wp_gpio >= 0 || cfg->hold_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (cfg->wp_gpio >= 0 ? (1ULL << cfg->wp_gpio) : 0) | (cfg->hold_gpio >= 0 ? (1ULL << cfg->hold_gpio) : 0),
            .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        if (cfg->wp_gpio >= 0) gpio_set_level(cfg->wp_gpio, 1);      // /WP high: writes enabled
        if (cfg->hold_gpio >= 0) gpio_set_level(cfg->hold_gpio, 1);  // /HOLD high: not paused
    }

    esp_flash_spi_device_config_t dev = {
        .host_id = cfg->host,
        .cs_io_num = cfg->cs_gpio,
        .io_mode = SPI_FLASH_FASTRD,
        .cs_id = cfg->cs_id,
        .freq_mhz = cfg->freq_mhz,
        .input_delay_ns = 0,
    };
    if ((r = spi_bus_add_flash_device(&s_flash, &dev)) != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_flash_device failed: %s", esp_err_to_name(r));
        return r;
    }
    if ((r = esp_flash_init(s_flash)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_flash_init failed: %s", esp_err_to_name(r));
        return r;
    }
    if ((r = esp_flash_get_size(s_flash, &s_size)) != ESP_OK || s_size <= NOR_LOG_DATA_OFFSET) {
        ESP_LOGE(TAG, "esp_flash_get_size failed: %s", esp_err_to_name(r));
        return r != ESP_OK ? r : ESP_FAIL;
    }
    uint32_t id = 0;
    esp_flash_read_id(s_flash, &id);
    ESP_LOGI(TAG, "NOR flash JEDEC 0x%06lx, %lu MB", (unsigned long)id, (unsigned long)(s_size / (1024 * 1024)));

    if ((r = nvs_open("storage", NVS_READWRITE, &s_nvs)) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(r));
        return r;
    }

    uint32_t fmt = 0;
    nvs_get_u32(s_nvs, "nor_fmt", &fmt);
    if (fmt != NOR_FORMAT_VERSION) {
        // First boot with this log format on this chip (or NVS was erased):
        // the chip may hold anything (e.g. the old FATFS image), so erase it all once.
        ESP_LOGW(TAG, "NOR log format %lu != %u: one-time full chip erase (%lu MB, this takes a while)...",
                 (unsigned long)fmt, NOR_FORMAT_VERSION, (unsigned long)(s_size / (1024 * 1024)));
        int64_t t0 = esp_timer_get_time();
        if ((r = esp_flash_erase_chip(s_flash)) != ESP_OK) {
            ESP_LOGE(TAG, "chip erase failed: %s", esp_err_to_name(r));
            return r;
        }
        ESP_LOGW(TAG, "chip erase done in %.1f s", (esp_timer_get_time() - t0) / 1e6);
        s_pos = NOR_LOG_DATA_OFFSET;
        s_clean = s_size;
        s_high_water = NOR_LOG_DATA_OFFSET;
        nvs_set_u32(s_nvs, "nor_fmt", NOR_FORMAT_VERSION);
        nvs_store_pointers();
        s_high_water = NOR_LOG_DATA_OFFSET;
    } else {
        nvs_load_pointers();
        if ((r = scan_for_end()) != ESP_OK) {
            ESP_LOGE(TAG, "end-of-data scan failed: %s", esp_err_to_name(r));
            return r;
        }
    }

    s_ok = true;
    ESP_LOGI(TAG, "ready: pos=0x%08lx (%lu bytes pending dump), clean up to 0x%08lx, free %lu KB",
             (unsigned long)s_pos, (unsigned long)(s_pos - NOR_LOG_DATA_OFFSET), (unsigned long)s_clean,
             (unsigned long)((s_size - s_pos) / 1024));
    return ESP_OK;
}

bool     nor_log_ok(void)            { return s_ok; }
uint32_t nor_log_chip_size(void)     { return s_size; }
uint32_t nor_log_bytes_pending(void) { return s_pos - NOR_LOG_DATA_OFFSET; }
uint32_t nor_log_records_written(void) { return s_records; }
uint32_t nor_log_drops(void)         { return s_drops; }
uint32_t nor_log_write_errors(void)  { return s_write_err; }
uint32_t nor_log_max_write_us(void)  { uint32_t v = s_max_write_us; s_max_write_us = 0; return v; }

void nor_log_commit_position(void) { if (s_ok) nvs_store_pointers(); }

// ---- dump ---------------------------------------------------------------------
esp_err_t nor_log_dump_to_file(const char *path) {
    if (!s_ok) return ESP_ERR_INVALID_STATE;
    uint32_t total = s_pos - NOR_LOG_DATA_OFFSET;
    if (total == 0) return ESP_OK;

    FILE *f = fopen(path, "wb");
    if (!f) { ESP_LOGE(TAG, "cannot create %s", path); return ESP_FAIL; }
    setvbuf(f, NULL, _IOFBF, 8192);

    static uint8_t chunk[4096];
    int64_t t0 = esp_timer_get_time();
    uint32_t rd = NOR_LOG_DATA_OFFSET;
    while (rd < s_pos) {
        uint32_t n = s_pos - rd;
        if (n > sizeof(chunk)) n = sizeof(chunk);
        esp_err_t r = esp_flash_read(s_flash, chunk, rd, n);
        if (r != ESP_OK) { ESP_LOGE(TAG, "read at 0x%08lx failed: %s", (unsigned long)rd, esp_err_to_name(r)); fclose(f); return r; }
        if (fwrite(chunk, 1, n, f) != n) { ESP_LOGE(TAG, "short SD write during dump"); fclose(f); return ESP_FAIL; }
        rd += n;
        if ((rd & 0xFFFF) == 0) vTaskDelay(1);   // let other tasks breathe on long dumps
    }
    fflush(f);
    fsync(fileno(f));
    fclose(f);

    struct stat st;
    if (stat(path, &st) != 0 || (uint32_t)st.st_size != total) {
        ESP_LOGE(TAG, "dump verification failed: %s is %ld bytes, expected %lu", path, (long)(stat(path, &st) == 0 ? st.st_size : -1), (unsigned long)total);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "dumped %lu bytes to %s in %.2f s (verified)", (unsigned long)total, path, (esp_timer_get_time() - t0) / 1e6);
    return ESP_OK;
}

// ---- wipe ---------------------------------------------------------------------
esp_err_t nor_log_wipe(void) {
    if (!s_ok) return ESP_ERR_INVALID_STATE;
    // Erase everything ever written since the last wipe, not just up to the
    // current pointer: an earlier session may have written further before its
    // own dump failed, and those stale bytes would otherwise force a 20-45 ms
    // just-in-time erase in the middle of the next flight.
    uint32_t dirty_end = s_high_water > s_pos ? s_high_water : s_pos;
    uint32_t erase_end = align_up(dirty_end, NOR_SECTOR);
    if (erase_end > s_size) erase_end = s_size;
    uint32_t old_clean = s_clean;

    // Publish the reset pointer FIRST: if the erase below is interrupted, the
    // next boot resumes at data_start, and ensure_clean() erases what it must.
    s_pos = NOR_LOG_DATA_OFFSET;
    s_clean = NOR_LOG_DATA_OFFSET;
    s_high_water = erase_end;   // still potentially dirty until the erase succeeds
    nvs_store_pointers();
    s_high_water = NOR_LOG_DATA_OFFSET;

    if (erase_end > NOR_LOG_DATA_OFFSET) {
        int64_t t0 = esp_timer_get_time();
        esp_err_t r = esp_flash_erase_region(s_flash, NOR_LOG_DATA_OFFSET, erase_end - NOR_LOG_DATA_OFFSET);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "erase of used region failed: %s (JIT erase will cover it)", esp_err_to_name(r));
            s_high_water = erase_end;
            nvs_store_pointers();
            return r;
        }
        ESP_LOGI(TAG, "wiped %lu KB in %.2f s", (unsigned long)((erase_end - NOR_LOG_DATA_OFFSET) / 1024), (esp_timer_get_time() - t0) / 1e6);
    }
    // Nothing above erase_end was ever written, and everything below it was
    // just erased: the whole data region is blank, so no erase or blank-check
    // is needed for the rest of this session.
    (void)old_clean;
    s_clean = s_size;
    s_high_water = NOR_LOG_DATA_OFFSET;
    nvs_store_pointers();
    return ESP_OK;
}

// ---- writer task ------------------------------------------------------------------
// Records are staged in RAM and programmed in page-bounded chunks (<= 256 B,
// never crossing a page), so a single esp_flash_write holds the shared SPI bus
// for one page program (~0.3-0.6 ms) at most. A partial trailing page is only
// programmed after NOR_STAGE_FLUSH_MS of silence (programming the remaining
// bytes of a page later is fine: they are still 0xFF).
#define NOR_STAGE_SIZE      4096u
#define NOR_STAGE_FLUSH_MS  500u

static uint8_t  s_stage[NOR_STAGE_SIZE];
static uint32_t s_stage_len = 0;

static esp_err_t nor_write_chunk(const uint8_t *data, uint32_t len) {
    if (s_pos + len > s_size) {
        ESP_LOGE(TAG, "NOR full (pos 0x%08lx) - logging to NOR stops", (unsigned long)s_pos);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t r = ensure_clean(len);
    if (r != ESP_OK) return r;
    int64_t t0 = esp_timer_get_time();
    r = esp_flash_write(s_flash, data, s_pos, len);
    uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
    if (dt > s_max_write_us) s_max_write_us = dt;
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "write at 0x%08lx failed: %s", (unsigned long)s_pos, esp_err_to_name(r));
        return r;
    }
    s_pos += len;
    if (s_pos > s_high_water) s_high_water = s_pos;
    return ESP_OK;
}

// Program staged bytes. all=false leaves an incomplete trailing page staged.
static esp_err_t flush_stage(bool all) {
    uint32_t off = 0;
    esp_err_t r = ESP_OK;
    while (off < s_stage_len) {
        uint32_t page_room = NOR_PAGE - (s_pos % NOR_PAGE);
        uint32_t avail = s_stage_len - off;
        uint32_t n = avail < page_room ? avail : page_room;
        if (!all && n < page_room) break;
        r = nor_write_chunk(s_stage + off, n);
        if (r != ESP_OK) break;
        off += n;
        if (off < s_stage_len) taskYIELD();   // let a latency-critical SPI user in between pages
    }
    if (off) {
        memmove(s_stage, s_stage + off, s_stage_len - off);
        s_stage_len -= off;
    }
    return r;
}

static void nor_log_task(void *arg) {
    nor_entry_t e;
    uint32_t since_commit = 0;
    int64_t last_commit = esp_timer_get_time();
    int64_t last_data = esp_timer_get_time();
    while (1) {
        bool got = xQueueReceive(s_queue, &e, pdMS_TO_TICKS(100)) == pdTRUE;
        int64_t now = esp_timer_get_time();
        if (got && s_ok) {
            if (s_stage_len + e.len > NOR_STAGE_SIZE) {
                if (flush_stage(true) != ESP_OK) goto fail;
            }
            memcpy(s_stage + s_stage_len, e.data, e.len);
            s_stage_len += e.len;
            s_records++;
            since_commit++;
            last_data = now;
            if (flush_stage(false) != ESP_OK) goto fail;
        } else if (s_ok && s_stage_len && now - last_data > (int64_t)NOR_STAGE_FLUSH_MS * 1000) {
            if (flush_stage(true) != ESP_OK) goto fail;
        }
        if (s_ok && since_commit && (since_commit >= NOR_COMMIT_RECORDS || now - last_commit > (int64_t)NOR_COMMIT_MS * 1000)) {
            nvs_store_pointers();
            since_commit = 0;
            last_commit = now;
        }
        continue;
fail:
        s_write_err++;
        if (s_write_err >= 3 || s_pos + NOR_STAGE_SIZE > s_size) {
            ESP_LOGE(TAG, "giving up on NOR writes");
            nvs_store_pointers();
            s_ok = false;
        }
    }
}

esp_err_t nor_log_task_start(int priority, int core) {
    if (!s_queue) s_queue = xQueueCreate(NOR_LOG_QUEUE_DEPTH, sizeof(nor_entry_t));
    if (!s_queue) return ESP_ERR_NO_MEM;
    BaseType_t x = xTaskCreatePinnedToCore(nor_log_task, "nor_log", 4096, NULL, priority, NULL, core);
    return x == pdPASS ? ESP_OK : ESP_FAIL;
}

esp_err_t nor_log_submit(const void *record, size_t len) {
    if (!s_ok || !s_queue) return ESP_ERR_INVALID_STATE;
    if (len == 0 || len > NOR_LOG_MAX_RECORD) return ESP_ERR_INVALID_ARG;
    nor_entry_t e;
    e.len = (uint16_t)len;
    memcpy(e.data, record, len);
    if (xQueueSend(s_queue, &e, 0) != pdTRUE) { s_drops++; return ESP_ERR_TIMEOUT; }
    return ESP_OK;
}
