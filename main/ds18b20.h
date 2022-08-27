#include "hal/gpio_types.h"

#define CMD_READ_ROM         0x33
#define CMD_SKIP_ROM         0xCC
#define CMD_CONVERT_T        0x44
#define CMD_READ_SCRATCHPAD  0xBE
#define CMD_WRITE_SCRATCHPAD 0x4E

#define T_CONV_MS 750 

typedef enum {
    R_9_BIT  = 0x00, ///< 0.5°C resolution
    R_10_BIT = 0x01, ///< 0.25°C resolution
    R_11_BIT = 0x02, ///< 0.125°C resolution
    R_12_BIT = 0x03, ///< 0.0625°C resolution
} sensor_resolution_t;

typedef struct {
    gpio_num_t dataPin;
} sensor_config_t;

typedef struct {
    gpio_num_t dataPin;
    sensor_resolution_t resolution;
    uint16_t tConv;                  ///< Convertation max time in ms. Depends on resolution.
} sensor_t;

typedef struct {
	uint8_t temp1;  ///< temperature lsb
    uint8_t temp2;  ///< Temperature msb
    uint8_t th;     ///< TH/User byte 1
    uint8_t tl;     ///< TL/User byre 2 
    uint8_t config; ///< Config 
} scratchpad_t;

bool sensorInit(sensor_t *sensor, sensor_config_t *sensorConfig);
bool sensorGetPresense(sensor_t *sensor);
bool sensorGetId(sensor_t *sensor, uint8_t *id);
bool sensorGetScratchpad(sensor_t *sensor, scratchpad_t *scratchpad);
bool sensorGetTempSync(sensor_t *sensor, float *temp);
bool sensorSetConfig(sensor_t *sensor, sensor_resolution_t resolution, uint8_t tempHigh, uint8_t tempLow);
