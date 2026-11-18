#include "sdcard.h"
#include "pins.h"
#include <esp_log.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdspi_host.h>
#include <driver/gpio.h>

#define TAG "sdcard"

static sdmmc_card_t *card = NULL;
static bool mounted = false;

bool sdcard_detected(void) {
    gpio_config_t io = { .pin_bit_mask = 1ULL << SD_DET_PIN, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&io);
    // Most microSD sockets close the detect switch to GND when a card is present.
    return gpio_get_level(SD_DET_PIN) == 0;
}

esp_err_t sdcard_init(void) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 0,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 20000;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI2_SDC_CS_PIN;
    slot_config.host_id = SPI2_HOST;

    ESP_LOGI(TAG, "Mounting SD card (detect switch: %s)...", sdcard_detected() ? "card present" : "no card");
    esp_err_t ret = esp_vfs_fat_sdspi_mount(SDCARD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "SD mount failed: card detected but no readable filesystem - format it FAT32 on a PC (exFAT is not supported)");
        } else {
            ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    mounted = true;
    ESP_LOGI(TAG, "SD card mounted at %s", SDCARD_MOUNT_POINT);
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

bool sdcard_is_mounted(void) { return mounted; }
