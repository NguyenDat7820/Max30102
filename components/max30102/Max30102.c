#include "max30102.h"
#include <stdio.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include <inttypes.h>  
#include "esp_log.h"


// Define necessary constants and internal functions
#define I2C_MASTER_NUM I2C_NUM_0 // Define I2C port number for master dev
#define I2C_MASTER_SCL_IO 22     // Define I2C GPIO for SCL
#define I2C_MASTER_SDA_IO 21     // Define I2C GPIO for SDA
#define I2C_MASTER_FREQ_HZ 100000 // Define I2C clock frequency

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static const char *TAG = "MAX30102";

// Function to write a value to a register
esp_err_t max30102_write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Function to read a value from a register
esp_err_t max30102_read_register(uint8_t reg, uint8_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Function to initialize the MAX30102
esp_err_t max30102_init(max30102_config_t* this, i2c_port_t i2c_num,
                        max30102_mode_t mode, max30102_sampling_rate_t sampling_rate,
                        max30102_pulse_width_t pulse_width, max30102_current_t ir_current,
                        max30102_current_t red_current, uint8_t mean_filter_size,
                        uint8_t pulse_bpm_sample_size, max30102_adc_range_t high_res_mode,
                        max30102_sample_averaging_t sample_averaging, bool overflow_enable,
                        max30102_almost_full_t almost_full, bool debug) {
    this->i2c_num = i2c_num;
    this->debug = debug;

    // Reset the sensor
    esp_err_t err = max30102_write_register(MAX30102_REG_MODE_CONFIG, 0x40);
    if (err != ESP_OK) return err;

    // Set the mode
    err = max30102_write_register(MAX30102_REG_MODE_CONFIG, mode);
    if (err != ESP_OK) return err;

    // Set the SpO2 configuration
    uint8_t spo2_config = (high_res_mode << 5) | (sampling_rate << 2) | pulse_width;
    err = max30102_write_register(MAX30102_REG_SPO2_CONFIG, spo2_config);
    if (err != ESP_OK) return err;

    // Set the LED pulse amplitude
    err = max30102_write_register(MAX30102_REG_LED1_PA, ir_current);
    if (err != ESP_OK) return err;
    err = max30102_write_register(MAX30102_REG_LED2_PA, red_current);
    if (err != ESP_OK) return err;

    // Set FIFO configuration
    uint8_t fifo_config = (sample_averaging << 5) | (overflow_enable << 4) | almost_full;
    err = max30102_write_register(MAX30102_REG_FIFO_WR_PTR, 0x00);
    if (err != ESP_OK) return err;
    err = max30102_write_register(MAX30102_REG_OVF_COUNTER, 0x00);
    if (err != ESP_OK) return err;
    err = max30102_write_register(MAX30102_REG_FIFO_RD_PTR, 0x00);
    if (err != ESP_OK) return err;
    err = max30102_write_register(MAX30102_REG_FIFO_DATA, fifo_config);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

// Function to update the sensor data
esp_err_t max30102_update(max30102_config_t* this, max30102_data_t* data) {
    uint8_t buffer[6];
    uint32_t red_led;
    uint32_t ir_led;

    // Read data from FIFO
    esp_err_t err = max30102_read_register(MAX30102_REG_FIFO_DATA, buffer);
    if (err != ESP_OK) return err;

    // Extract the Red LED and IR LED values
    red_led = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    ir_led = (buffer[3] << 16) | (buffer[4] << 8) | buffer[5];

    // Convert raw values to heart rate and SpO2 (this needs more complex calculations in reality)
    data->heart_bpm = (float)ir_led / 1000;  // Placeholder conversion
    data->spO2 = (float)red_led / 1000;  // Placeholder conversion

    if (this->debug) {
          ESP_LOGI(TAG, "IR: %" PRIu32 ", Red: %" PRIu32, ir_led, red_led);
    }

    return ESP_OK;
}
