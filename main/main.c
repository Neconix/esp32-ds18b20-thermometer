#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "time.h"
#include "math.h"

#include "ds18b20.h"

#define WAIT vTaskDelay(2000/portTICK_PERIOD_MS)

#define CONFIG_DATA_PIN    5

static const char *TAG = "MAIN";

double getTimeSec( void )
{
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    double time = spec.tv_sec + spec.tv_nsec / 1e9;
    return time;
}

void DS18B20_Test(void *pvParameters)
{
    sensor_t sensor;
    uint8_t sensorId[8];
    scratchpad_t scratchpad;
    float tempC;

    sensor_config_t sensorConfig = {
        .dataPin = CONFIG_DATA_PIN
    };

    for (;;)
    {
        bool presense = sensorInit(&sensor, &sensorConfig);
        ESP_LOGI(TAG, "Sensor online = %d", presense);

        if (!presense) {
            WAIT;
            continue;
        }
        
        sensorGetId(&sensor, &sensorId);
        ESP_LOG_BUFFER_HEX("Sensor ID", sensorId, 8);

        sensorSetConfig(&sensor, R_10_BIT, 0, 0);

        sensorGetScratchpad(&sensor, &scratchpad);
        ESP_LOG_BUFFER_HEX("Scratchpad raw", &scratchpad, 5);

        sensorGetTempSync(&sensor, &tempC);
        ESP_LOGI(TAG, "tempC = %.4f", tempC);

        WAIT;   
    }
}

void app_main(void)
{
    xTaskCreate(DS18B20_Test, "DS18B20_Test", 1024 * 6, NULL, 2, NULL);
}
