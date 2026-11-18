/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_bmp390_interface_template.c
 * @brief     driver bmp390 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-05-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/05/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_bmp390_interface.h"
#include "sdkconfig.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>

#define TAG "bmp390_driver_if"

static i2c_master_bus_handle_t master_handle = NULL;
static i2c_master_dev_handle_t device_handle = NULL;
static uint16_t i2c_address = 0;
static uint32_t i2c_scl_speed_hz = 0;

void bmp390_set_i2c_config(i2c_master_bus_handle_t bus, uint16_t address, uint32_t scl_speed_hz) {
    master_handle = bus;
    i2c_address = address;
    i2c_scl_speed_hz = scl_speed_hz;
}


/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t bmp390_interface_iic_init(void)
{
    esp_err_t ret = i2c_master_probe(master_handle, i2c_address, 500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c probe failed for BMP390 at address 0x%02x: %s", i2c_address, esp_err_to_name(ret));
        return 1;
    }
    ret = i2c_master_bus_add_device(master_handle, &(i2c_device_config_t){
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = i2c_address,
        .scl_speed_hz       = i2c_scl_speed_hz, // use configured SCL speed
    }, &device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c add device failed for BMP390 at address 0x%02x: %s", i2c_address, esp_err_to_name(ret));
        return 1;
    }
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t bmp390_interface_iic_deinit(void)
{   
   esp_err_t ret = i2c_master_bus_rm_device(device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c remove device failed: %s", esp_err_to_name(ret));
        return 1;
    }
    device_handle = NULL;
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp390_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (device_handle == NULL) {
        ESP_LOGE(TAG, "i2c read failed: device handle is NULL");
        return 1;
    }
    uint8_t buf_reg[1] = {reg};

    esp_err_t ret = i2c_master_transmit_receive(device_handle, buf_reg, 1, buf, len, 500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c receive failed: %s", esp_err_to_name(ret));
        return 1;
    }
    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp390_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    
    if (device_handle == NULL) {
        ESP_LOGE(TAG, "i2c write failed: device handle is NULL");
        return 1;
    }
    uint8_t buf_reg[256]; // hopefully this is enough
    if (len > 255) {
        ESP_LOGE(TAG, "i2c write failed: data length exceeds buffer size");
        return 1;
    }
    buf_reg[0] = reg;
    memcpy(buf_reg + 1, buf, len);

    esp_err_t ret = i2c_master_transmit(device_handle, buf_reg, len + 1, 500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c transmit failed: %s", esp_err_to_name(ret));
        return 1;
    }
    return 0;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t bmp390_interface_spi_init(void)
{
    ESP_LOGE(TAG, "init: SPI interface not implemented");
    return 1;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t bmp390_interface_spi_deinit(void)
{   
    ESP_LOGE(TAG, "deinit: SPI interface not implemented");
    return 1;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp390_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    ESP_LOGE(TAG, "read: SPI interface not implemented");
    return 1;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp390_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
    ESP_LOGE(TAG, "write: SPI interface not implemented");
    return 1;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void bmp390_interface_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void bmp390_interface_debug_print(const char *const fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    ESP_LOGI(TAG, "%s", buf);
    va_end(args);
}

/**
 * @brief     interface receive callback
 * @param[in] type interrupt type
 * @note      none
 */
void bmp390_interface_receive_callback(uint8_t type)
{
    switch (type)
    {
        case BMP390_INTERRUPT_STATUS_FIFO_WATERMARK :
        {
            bmp390_interface_debug_print("bmp390: irq fifo watermark.\n");
            
            break;
        }
        case BMP390_INTERRUPT_STATUS_FIFO_FULL :
        {
            bmp390_interface_debug_print("bmp390: irq fifo full.\n");
            
            break;
        }
        case BMP390_INTERRUPT_STATUS_DATA_READY :
        {
            bmp390_interface_debug_print("bmp390: irq data ready.\n");
            
            break;
        }
        default :
        {
            bmp390_interface_debug_print("bmp390: unknown code.\n");
            
            break;
        }
    }
}