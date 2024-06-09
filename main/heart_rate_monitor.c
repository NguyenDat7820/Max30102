#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "max30102.h"


void app_main() {
    max30102_config_t max30102;
    max30102_data_t data;

    esp_err_t err = max30102_init(&max30102, I2C_NUM_0, MAX30102_MODE_SPO2_HR, 
                                  MAX30102_SR_100HZ, MAX30102_PW_411US, 
                                  MAX30102_CURRENT_24MA, MAX30102_CURRENT_24MA,
                                  4, 10, MAX30102_ADC_RANGE_16384, 
                                  MAX30102_SAMPLE_AVERAGING_4, true, 
                                  MAX30102_ALMOST_FULL_15, true);
    if (err != ESP_OK) {
        printf("Failed to initialize MAX30102\n");
        return;
    }

    while (1) {
        err = max30102_update(&max30102, &data);
        if (err == ESP_OK) {
            printf("Heart Rate: %.2f bpm, SpO2: %.2f%%\n", data.heart_bpm, data.spO2);
        } else {
            printf("Failed to update sensor data\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
    }
}
