#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hmc5883l.h"

#define HMC5883L_ADDR 0x1E

void hmc5883l_init(void)
{
    // CRA: 8-sample average, 15 Hz
    uint8_t configA[2] = {0x00, 0b01110000};
    i2c_write_blocking(i2c1, HMC5883L_ADDR, configA, 2, false);

    // CRB: gain = 1.3 Ga (default)
    uint8_t configB[2] = {0x01, 0x20};
    i2c_write_blocking(i2c1, HMC5883L_ADDR, configB, 2, false);

    // Mode: continuous conversion
    uint8_t mode[2] = {0x02, 0x00};
    i2c_write_blocking(i2c1, HMC5883L_ADDR, mode, 2, false);
}

bool hmc5883l_read_heading(float *heading_deg)
{
    uint8_t reg = 0x03;
    uint8_t raw[6];

    // set pointer
    if (i2c_write_blocking(i2c1, HMC5883L_ADDR, &reg, 1, true) < 0) return false;

    // read 6 bytes: X Z Y
    if (i2c_read_blocking(i2c1, HMC5883L_ADDR, raw, 6, false) < 0) return false;

    int16_t x = (raw[0] << 8) | raw[1];
    int16_t z = (raw[2] << 8) | raw[3];
    int16_t y = (raw[4] << 8) | raw[5];

    // compute heading  (flat / car level → ignore Z)
    float heading = atan2((float)y, (float)x) * (180.0f / M_PI);

    if (heading < 0) heading += 360.0f;

    *heading_deg = heading;
    return true;
}
