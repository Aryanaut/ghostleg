#ifndef SDCARD_H
#define SDCARD_H
#include <esp_err.h>
#include <stdbool.h>

#define SDCARD_MOUNT_POINT "/sdcard"

// Mount the microSD (SPI mode on SPI2, CS2). The SPI bus must already be
// initialised. Never formats a card: an unreadable filesystem is an error.
esp_err_t sdcard_init(void);
bool      sdcard_is_mounted(void);
bool      sdcard_detected(void);   // card-detect switch

#endif // SDCARD_H
