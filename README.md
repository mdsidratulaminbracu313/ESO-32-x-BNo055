/*
 * BNO055 IMU Interfacing with ESP32 using ESP-IDF
 * Reads Euler Angles (Heading, Roll, Pitch) using I2C
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BNO055_APP";

/* I2C configuration */
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          50000   // 50kHz to avoid BNO055 clock stretching issues

/* BNO055 I2C Address */
#define BNO055_ADDR                 0x28

/* BNO055 Registers */
#define BNO055_PAGE_ID_ADDR         0x07
#define BNO055_CHIP_ID_ADDR         0x00
#define BNO055_CHIP_ID              0xA0
#define BNO055_OPR_MODE_ADDR        0x3D
#define BNO055_EULER_H_LSB_ADDR     0x1A

/* Operation Modes */
#define OPERATION_MODE_CONFIG       0x00
#define OPERATION_MODE_NDOF         0x0C

/* ---------------------------------------------------------
   I2C MASTER INITIALIZATION
--------------------------------------------------------- */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/* ---------------------------------------------------------
   WRITE ONE BYTE TO BNO055
--------------------------------------------------------- */
esp_err_t bno055_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        BNO055_ADDR,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(100));
}

/* ---------------------------------------------------------
   READ DATA FROM BNO055
--------------------------------------------------------- */
esp_err_t bno055_read_data(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        BNO055_ADDR,
        &reg_addr,
        1,
        data,
        len,
        pdMS_TO_TICKS(100));
}

/* ---------------------------------------------------------
   BNO055 SENSOR INITIALIZATION
--------------------------------------------------------- */
void bno055_init(void)
{
    // 1. Give the sensor time to breathe after power-on (Crucial for BNO055)
    vTaskDelay(pdMS_TO_TICKS(800)); 

    // 2. Ensure we are on Page 0 to read the correct registers
    bno055_write_byte(BNO055_PAGE_ID_ADDR, 0x00);
    vTaskDelay(pdMS_TO_TICKS(25));

    uint8_t chip_id = 0;
    ESP_LOGI(TAG, "Checking BNO055 chip ID...");
    
    esp_err_t err = bno055_read_data(BNO055_CHIP_ID_ADDR, &chip_id, 1);

    if (err != ESP_OK || chip_id != BNO055_CHIP_ID)
    {
        ESP_LOGE(TAG, "BNO055 not detected! Chip ID: 0x%02X, Error: %s", chip_id, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "BNO055 detected successfully");

    // 3. Switch to CONFIG mode (required before changing to a fusion mode)
    ESP_ERROR_CHECK(bno055_write_byte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG));
    vTaskDelay(pdMS_TO_TICKS(25));

    // 4. Switch to NDOF mode (Nine Degrees of Freedom sensor fusion mode)
    ESP_ERROR_CHECK(bno055_write_byte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF));
    vTaskDelay(pdMS_TO_TICKS(25));

    ESP_LOGI(TAG, "BNO055 set to NDOF fusion mode");
}

/* ---------------------------------------------------------
   MAIN APPLICATION
--------------------------------------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");

    bno055_init();

    uint8_t raw_data[6];

    while (1)
    {
        // Read 6 bytes starting from Heading LSB
        if (bno055_read_data(BNO055_EULER_H_LSB_ADDR, raw_data, 6) == ESP_OK)
        {
            // Reconstruct the 16-bit values (Little Endian)
            int16_t heading = (raw_data[1] << 8) | raw_data[0];
            int16_t roll    = (raw_data[3] << 8) | raw_data[2];
            int16_t pitch   = (raw_data[5] << 8) | raw_data[4];

            // 1 degree = 16 LSB in Euler format
            float heading_deg = heading / 16.0f;
            float roll_deg    = roll / 16.0f;
            float pitch_deg   = pitch / 16.0f;

            ESP_LOGI(TAG,
                     "Heading: %6.2f° | Roll: %6.2f° | Pitch: %6.2f°",
                     heading_deg,
                     roll_deg,
                     pitch_deg);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read Euler data");
        }

        // Delay between readings
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
