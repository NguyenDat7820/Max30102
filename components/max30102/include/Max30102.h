#ifndef MAX30102_H
#define MAX30102_H

#include "driver/i2c.h"
#include "esp_err.h"

// I2C address for MAX30102
#define MAX30102_ADDRESS 0x57

// Register addresses
#define MAX30102_REG_INTR_STATUS_1 0x00
#define MAX30102_REG_INTR_STATUS_2 0x01
#define MAX30102_REG_INTR_ENABLE_1 0x02
#define MAX30102_REG_INTR_ENABLE_2 0x03
#define MAX30102_REG_FIFO_WR_PTR 0x04
#define MAX30102_REG_OVF_COUNTER 0x05
#define MAX30102_REG_FIFO_RD_PTR 0x06
#define MAX30102_REG_FIFO_DATA 0x07
#define MAX30102_REG_MODE_CONFIG 0x09
#define MAX30102_REG_SPO2_CONFIG 0x0A
#define MAX30102_REG_LED1_PA 0x0C
#define MAX30102_REG_LED2_PA 0x0D
#define MAX30102_REG_PILOT_PA 0x10
#define MAX30102_REG_MULTI_LED_CTRL1 0x11
#define MAX30102_REG_MULTI_LED_CTRL2 0x12
#define MAX30102_REG_TEMP_INTR 0x1F
#define MAX30102_REG_TEMP_FRAC 0x20
#define MAX30102_REG_TEMP_CONFIG 0x21
#define MAX30102_REG_PROX_INT_THRESH 0x30
#define MAX30102_REG_REV_ID 0xFE
#define MAX30102_REG_PART_ID 0xFF

// Sensor mode options
typedef enum {
    MAX30102_MODE_HR_ONLY = 0x02,
    MAX30102_MODE_SPO2_HR = 0x03,
    MAX30102_MODE_MULTI_LED = 0x07,
} max30102_mode_t;

// Sample rate options
typedef enum {
    MAX30102_SR_50HZ = 0x00,
    MAX30102_SR_100HZ = 0x01,
    MAX30102_SR_200HZ = 0x02,
    MAX30102_SR_400HZ = 0x03,
    MAX30102_SR_800HZ = 0x04,
    MAX30102_SR_1000HZ = 0x05,
    MAX30102_SR_1600HZ = 0x06,
    MAX30102_SR_3200HZ = 0x07,
} max30102_sampling_rate_t;

// LED Pulse Width options
typedef enum {
    MAX30102_PW_69US = 0x00,
    MAX30102_PW_118US = 0x01,
    MAX30102_PW_215US = 0x02,
    MAX30102_PW_411US = 0x03,
} max30102_pulse_width_t;

// ADC Range options
typedef enum {
    MAX30102_ADC_RANGE_2048 = 0x00,
    MAX30102_ADC_RANGE_4096 = 0x01,
    MAX30102_ADC_RANGE_8192 = 0x02,
    MAX30102_ADC_RANGE_16384 = 0x03,
} max30102_adc_range_t;

// LED Current options
typedef enum {
    MAX30102_CURRENT_0MA = 0x00,
    MAX30102_CURRENT_4_4MA = 0x01,
    MAX30102_CURRENT_7_6MA = 0x02,
    MAX30102_CURRENT_11MA = 0x03,
    MAX30102_CURRENT_14_2MA = 0x04,
    MAX30102_CURRENT_17_4MA = 0x05,
    MAX30102_CURRENT_20_8MA = 0x06,
    MAX30102_CURRENT_24MA = 0x07,
    MAX30102_CURRENT_27_1MA = 0x08,
    MAX30102_CURRENT_30_6MA = 0x09,
    MAX30102_CURRENT_33_8MA = 0x0A,
    MAX30102_CURRENT_37MA = 0x0B,
    MAX30102_CURRENT_40_2MA = 0x0C,
    MAX30102_CURRENT_43_6MA = 0x0D,
    MAX30102_CURRENT_46_8MA = 0x0E,
    MAX30102_CURRENT_50MA = 0x0F,
} max30102_current_t;

// Almost Full options
typedef enum {
    MAX30102_ALMOST_FULL_0 = 0x00,
    MAX30102_ALMOST_FULL_1 = 0x01,
    MAX30102_ALMOST_FULL_2 = 0x02,
    MAX30102_ALMOST_FULL_3 = 0x03,
    MAX30102_ALMOST_FULL_4 = 0x04,
    MAX30102_ALMOST_FULL_5 = 0x05,
    MAX30102_ALMOST_FULL_6 = 0x06,
    MAX30102_ALMOST_FULL_7 = 0x07,
    MAX30102_ALMOST_FULL_8 = 0x08,
    MAX30102_ALMOST_FULL_9 = 0x09,
    MAX30102_ALMOST_FULL_10 = 0x0A,
    MAX30102_ALMOST_FULL_11 = 0x0B,
    MAX30102_ALMOST_FULL_12 = 0x0C,
    MAX30102_ALMOST_FULL_13 = 0x0D,
    MAX30102_ALMOST_FULL_14 = 0x0E,
    MAX30102_ALMOST_FULL_15 = 0x0F,
} max30102_almost_full_t;

// Sample Averaging options
typedef enum {
    MAX30102_SAMPLE_AVERAGING_1 = 0x00,
    MAX30102_SAMPLE_AVERAGING_2 = 0x01,
    MAX30102_SAMPLE_AVERAGING_4 = 0x02,
    MAX30102_SAMPLE_AVERAGING_8 = 0x03,
    MAX30102_SAMPLE_AVERAGING_16 = 0x04,
    MAX30102_SAMPLE_AVERAGING_32 = 0x05,
} max30102_sample_averaging_t;

// Configuration structure
typedef struct {
    i2c_port_t i2c_num;
    bool debug;
    // Add other configuration fields as needed
} max30102_config_t;

// Data structure
typedef struct {
    float heart_bpm;
    float spO2;
} max30102_data_t;

// Function declarations
esp_err_t max30102_init(max30102_config_t* this, i2c_port_t i2c_num, 
                        max30102_mode_t mode, max30102_sampling_rate_t sampling_rate, 
                        max30102_pulse_width_t pulse_width, max30102_current_t ir_current,
                        max30102_current_t start_red_current, uint8_t mean_filter_size, 
                        uint8_t pulse_bpm_sample_size, max30102_adc_range_t high_res_mode, max30102_sample_averaging_t sample_averaging,
                        bool overflow_enable,
                        max30102_almost_full_t almost_full, bool debug);

esp_err_t max30102_update(max30102_config_t* this, max30102_data_t* data);

esp_err_t max30102_write_register(uint8_t reg, uint8_t value);
esp_err_t max30102_read_register(uint8_t reg, uint8_t *value);

#endif // MAX30102_H
