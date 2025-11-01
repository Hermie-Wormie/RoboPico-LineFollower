#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hmc5883l.h"

#define HMC5883L_ADDR 0x1E
#define HMC_I2C      i2c1

void hmc5883l_init(void)
{
    // ==== I2C INIT (REQUIRED) ====
    i2c_init(HMC_I2C, 400000);       // 400kHz
    gpio_set_function(2, GPIO_FUNC_I2C); // SDA1 (GP2)
    gpio_set_function(3, GPIO_FUNC_I2C); // SCL1 (GP3)
    gpio_pull_up(2);
    gpio_pull_up(3);

    // ==== SETUP HMC5883L REGISTERS ====
    uint8_t configA[2] = {0x00, 0b01110000};   // 8 samples avg, 15Hz
    i2c_write_blocking(HMC_I2C, HMC5883L_ADDR, configA, 2, false);

    uint8_t configB[2] = {0x01, 0x20}; // gain = 1.3 Ga
    i2c_write_blocking(HMC_I2C, HMC5883L_ADDR, configB, 2, false);

    uint8_t mode[2] = {0x02, 0x00};   // continuous conversion
    i2c_write_blocking(HMC_I2C, HMC5883L_ADDR, mode, 2, false);
}

bool hmc5883l_read_heading(float *heading_deg)
{
    uint8_t reg = 0x03;
    uint8_t raw[6];

    // pointer
    if (i2c_write_blocking(HMC_I2C, HMC5883L_ADDR, &reg, 1, true) < 0)
        return false;

    // read 6 bytes
    if (i2c_read_blocking(HMC_I2C, HMC5883L_ADDR, raw, 6, false) < 0)
        return false;

    int16_t x = (raw[0] << 8) | raw[1];
    int16_t z = (raw[2] << 8) | raw[3];
    int16_t y = (raw[4] << 8) | raw[5];

    // compute heading
    float heading = atan2f((float)y, (float)x) * (180.0f / M_PI);
    if (heading < 0) heading += 360.0f;

    // ---- add smoothing ----
    static float filt = 0.0f;
    filt = filt * 0.90f + heading * 0.10f;  // 90% old, 10% new

    *heading_deg = filt;
    return true;
}
