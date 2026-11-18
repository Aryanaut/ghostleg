#ifndef PINS_H
#define PINS_H
#include <driver/gpio.h>

// Pin map for Marcus's ESP32-S3 Dev Board V1.0 (Space Bar Avionics Training Board).

// ---- SPI2 bus: IMU (CS0), NOR flash (CS1), microSD (CS2) ----
#define SPI2_CLK_PIN          GPIO_NUM_10
#define SPI2_MOSI_PIN         GPIO_NUM_11
#define SPI2_MISO_PIN         GPIO_NUM_13
#define SPI2_IMU_CS_PIN       GPIO_NUM_41  // SPI2 CS0 - LSM6DSV80X
#define SPI2_NOR_FLASH_CS_PIN GPIO_NUM_38  // SPI2 CS1 - GD25Q128E (16 MB)
#define SPI2_SDC_CS_PIN       GPIO_NUM_12  // SPI2 CS2 - microSD
#define FLASH_WP_PIN          GPIO_NUM_14  // /WP  (board pull-down: must be driven HIGH to write)
#define FLASH_HOLD_PIN        GPIO_NUM_15  // /HOLD (board pull-up)
#define SD_DET_PIN            GPIO_NUM_42  // card detect
#define IMU_INT1_PIN          GPIO_NUM_39  // LSM6DSV80X INT1 (data-ready)

// ---- IMU link to Avionics C: UART0 TX on header J4 pin 15 ("TX0") ----
// J4.1 = +3.3V, J4.2 = GND, J4.15 = TX0 (GPIO43), J4.16 = RX0 (GPIO44, unused)
#define IMU_LINK_UART_PORT UART_NUM_0
#define IMU_LINK_TX_PIN    GPIO_NUM_43

// ---- I2C1 bus: barometer (BMP390) + magnetometer (MMC5983MA) ----
#define I2C1_SDA_PIN  GPIO_NUM_1
#define I2C1_SCL_PIN  GPIO_NUM_2

#endif // PINS_H
