#include <string.h>
#include <math.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "ds18b20.h"

#define TAG "DS18B20"

static void sensorWriteBit(sensor_t *sensor, bool bit)
{
    
    gpio_set_level(sensor->dataPin, 0);

    if (bit) {
        esp_rom_delay_us(1);
        gpio_set_level(sensor->dataPin, 1);
        /* Must pull high within 15 us */
        esp_rom_delay_us(15 + 45);
        return;
    }

    /* Write slot duration at least 60 us */
    esp_rom_delay_us(60);
    gpio_set_level(sensor->dataPin, 1);
}

static bool sensorReadBit(sensor_t *sensor)
{
    bool bit;

    /* Pull low minimum 1 us */
    gpio_set_level(sensor->dataPin, 0);
    esp_rom_delay_us(1);
    gpio_set_level(sensor->dataPin, 1);

    /* Must sample within 15 us of the failing edge */
    esp_rom_delay_us(5);
    bit = gpio_get_level(sensor->dataPin);

    /* Read slot duration at least 60 us */
    esp_rom_delay_us(55);

    return bit;
}

void sensorWriteByte(sensor_t *sensor, uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        sensorWriteBit(sensor, (data >> i) & 0x1);
        esp_rom_delay_us(1);
    }
}

uint8_t sensorReadByte(sensor_t *sensor)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data |=  sensorReadBit(sensor) << i;
    }
    return data;
}

bool sensorGetPresense(sensor_t *sensor)
{
    uint8_t signal;

    gpio_set_level(sensor->dataPin, 0);
    esp_rom_delay_us(480);
    gpio_set_level(sensor->dataPin, 1);

    // DS18B20 waits 15-60 µs and then transmits the presence pulse (a low signal for 60-240 µs)
    esp_rom_delay_us(60);

    signal = gpio_get_level(sensor->dataPin);
    esp_rom_delay_us(420);
    
    return signal == 0;
}

bool sensorGetPresenseTimingsTest(sensor_t *sensor)
{
    uint8_t presense = 0;
    int64_t start;
    int64_t answerDelay;
    int64_t presensePeriod;
    uint8_t  signal;

    gpio_set_level(sensor->dataPin, 0);
    esp_rom_delay_us(480);
    gpio_set_level(sensor->dataPin, 1);

    // esp_rom_delay_us(15);

    // DS18B20 waits 15-60 µs and then transmits the presence pulse (a low signal for 60-240 µs)
    start = esp_timer_get_time();
    do {
        signal = gpio_get_level(sensor->dataPin);
        answerDelay = esp_timer_get_time() - start;
    } while (signal == 1 && answerDelay < 70);

    start = esp_timer_get_time();
    do {
        signal = gpio_get_level(sensor->dataPin);
        presensePeriod = esp_timer_get_time() - start;
    } while (signal == 0 && presensePeriod < 480);

    ESP_LOGI(TAG, "answerDelay=%lld us, presensePeriod=%lld us", answerDelay, presensePeriod);
    
    return (bool) presense;
}

uint8_t calcCRC(uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;
	
	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

/**
 * @brief Initializes pin, and send presense pulse
 * 
 * @param sensor
 * @param sensorConfig 
 * @return true on success sensor answer
 * @return false on failed sensor answer
 */
bool sensorInit(sensor_t *sensor, sensor_config_t *sensorConfig)
{
    bool presense;

    gpio_config_t pinConfig = {
        .pin_bit_mask = BIT64(sensorConfig->dataPin),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&pinConfig);

    sensor->dataPin = sensorConfig->dataPin;
    // Default resolution. Sensor actually may have another value.
    sensor->resolution = R_12_BIT;
    sensor->tConv = 750;

    presense = sensorGetPresense(sensor);

    return presense;
}

/**
 * @brief Reads the DS18B20’s 8-bit family code, unique 48-bit serial number, and 8-bit CRC.
 *  This command can only be used if there is a single DS18B20 on the bus.
 * 
 * @param sensor 
 * @param id 8-byte array with sensor ID and CRC
 * @return 0 - sensor id not found, or incorrect. 1 - success id reading 
 */
bool sensorGetId(sensor_t *sensor, uint8_t *id)
{
    bool presense = sensorGetPresense(sensor);
    if (presense == false) {
        return false;
    }

    // Writing read ROM command for single sensor
    sensorWriteByte(sensor, CMD_READ_ROM);

    // Getting ID
    for (uint8_t i = 0; i < 8; i++)
    {
        id[i] = sensorReadByte(sensor);
    }

    // Checking family code and CRC
    if (
        id[0] != 0x28 || 
        calcCRC(id, 7) != id[7]
    ) {
        return false;
    }
    
    return true;
}

bool sensorGetScratchpad(sensor_t *sensor, scratchpad_t *scratchpad)
{
    uint8_t scratchpadBytes[9];

    bool presense = sensorGetPresense(sensor);
    if (presense == false) {
        return false;
    }

    // Skipping ROM reading
    sensorWriteByte(sensor, CMD_SKIP_ROM);
    // Signal to read measured value
    sensorWriteByte(sensor, CMD_READ_SCRATCHPAD);

    // Getting scratchpad bytes with CRC
    for (uint8_t i = 0; i < 9; i++)
    {
        scratchpadBytes[i] = sensorReadByte(sensor);
    }

    // Checking CRC for 8 bytes
    if (calcCRC(scratchpadBytes, 8) != scratchpadBytes[8]) {
        return false;
    }

    scratchpad->temp1  = scratchpadBytes[0];
    scratchpad->temp2  = scratchpadBytes[1];
    scratchpad->th     = scratchpadBytes[2];
    scratchpad->tl     = scratchpadBytes[3];
    scratchpad->config = scratchpadBytes[4];
    
    return true;
}

bool sensorSetConfig(sensor_t *sensor, sensor_resolution_t resolution, uint8_t tempHigh, uint8_t tempLow)
{
    bool presense = sensorGetPresense(sensor);
    if (presense == false) {
        return false;
    }

    // Writing skip read ROM command. Writing only single sensor.
    sensorWriteByte(sensor, CMD_SKIP_ROM);
    // Start write 3 bytes scratchpad
    sensorWriteByte(sensor, CMD_WRITE_SCRATCHPAD);
    // Set sensor config bytes
    sensorWriteByte(sensor, tempHigh);        // th byte
    sensorWriteByte(sensor, tempLow);         // tl byte
    sensorWriteByte(sensor, resolution << 5); // config

    sensor->resolution = resolution;

    switch(sensor->resolution) {
        case R_9_BIT:  sensor->tConv = 94;  break;
        case R_10_BIT: sensor->tConv = 188; break;
        case R_11_BIT: sensor->tConv = 375; break;
        case R_12_BIT: sensor->tConv = 750; break;
    }

    return true;
}

/**
 * @brief Reads temperature mesurement from sensors scratchpad (in Celsius)
 * 
 * @param sensor 
 * @param temp 
 * @return true 
 * @return false 
 */
bool sensorGetTempSync(sensor_t *sensor, float *temp)
{
    scratchpad_t scratchpad;
    int8_t integerT;
    float floatT;

    bool presense = sensorGetPresense(sensor);
    if (presense == false) {
        return false;
    }

    // Writing skip read ROM command. Reading only single sensor
    sensorWriteByte(sensor, CMD_SKIP_ROM);
    // Signal to sensor to start temperature conversion
    sensorWriteByte(sensor, CMD_CONVERT_T);
    // Wait for tconv
    esp_rom_delay_us(sensor->tConv);

    presense = sensorGetScratchpad(sensor, &scratchpad);
    if (presense == false) {
        return false;
    }

    integerT = (scratchpad.temp2 << 4) | (scratchpad.temp1 >> 4);

    floatT  = ((scratchpad.temp1 >> 3) & 0x1) * 0.5;
    floatT += ((scratchpad.temp1 >> 2) & 0x1) * 0.25;
    floatT += ((scratchpad.temp1 >> 1) & 0x1) * 0.125;
    floatT +=  (scratchpad.temp1       & 0x1) * 0.0625;

    floatT = integerT > 0 ? floatT : -floatT;
    
    *temp = (float) integerT + floatT;

    return true;
}