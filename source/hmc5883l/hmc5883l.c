#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hmc5883l.h"
#include "telemetry.h"

#define HMC5883L_ADDR 0x1E
#define HMC_I2C      i2c1

// Initializes accelerometer
void imu_accel_init(void) {
    // Enable X/Y/Z axes, set datarate (e.g., 50 Hz)
    // 0x57 (01010111) = 50 Hz, all axes enabled, normal mode
    uint8_t config[2] = {CTRL_REG1_A, 0x57}; 
    i2c_write_blocking(HMC_I2C, LSM303_ACCEL_ADDR, config, 2, false);
}

// Reads 3-axis accleration data from IMU
bool imu_read_accel(int16_t *x, int16_t *y, int16_t *z){
    // Auto-increment bit (0x80) on starting register address
    uint8_t reg = ACCEL_OUT_X_L | 0x80; 
    uint8_t raw[6];

    if (i2c_write_blocking(HMC_I2C, LSM303_ACCEL_ADDR, &reg, 1, true) < 0) return false;
    if (i2c_read_blocking(HMC_I2C, LSM303_ACCEL_ADDR, raw, 6, false) < 0) return false;

    // Combine LSB and MSB (assuming little endian for LSM303DLHC output)
    *x = (raw[1] << 8) | raw[0];
    *y = (raw[3] << 8) | raw[2];
    *z = (raw[5] << 8) | raw[4];
    
    return true;
}

// Initializes magnetometer and I2C peripheral
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

    imu_accel_init();

}

// Reads raw magnetic field data, reads accleration data, performs
// tilt compensation, and calculates final magnetic heading
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

    // Read Accelerometer
    int16_t ax_raw, ay_raw, az_raw;
    if (!imu_read_accel(&ax_raw, &ay_raw, &az_raw)) return false;

    // Convert raw to float/G's (assuming +/-2G range for simplicity, scale factor ignored for angle calc)
    float ax = (float)ax_raw;
    float ay = (float)ay_raw;
    float az = (float)az_raw;
    
    // 3. CALCULATE ROLL (phi) and PITCH (theta)
    // Ensure atan2f is used for full quadrant accuracy
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // 4. TILT COMPENSATION: Project magnetometer vector to horizontal plane
    // Convert magnetic readings to float (Mx, My, Mz)
    float mx = (float)x;
    float my = (float)y;
    float mz = (float)z;

    // Apply Hard-Iron Calibration Offsets (Mx_comp = Mx_raw - Offset_x, etc.)
    float mx_comp = (float)x - 154.0f;
    float my_comp = (float)y - 15.0f;
    float mz_comp = (float)z - 230.0f;

    float Bx_forward = -my_comp;
    float By_left = mx_comp;
    float Bz_up = mz_comp;

    // Tilt-compensated X' and Y' magnetic components
    //float Bx_horiz = mx * cosf(pitch) + my * sinf(roll) * sinf(pitch) + mz * cosf(roll) * sinf(pitch);
    //float By_horiz = my * cosf(roll) - mz * sinf(roll);
    float Bx_horiz = Bx_forward * cosf(pitch) + By_left * sinf(roll) * sinf(pitch) + Bz_up * cosf(roll) * sinf(pitch);
    float By_horiz = By_left * cosf(roll) - Bz_up * sinf(roll);

    // compute heading
    //float heading = atan2f((float)y, (float)x) * (180.0f / M_PI);
    float heading = atan2f(By_horiz, Bx_horiz) * (180.0f / M_PI);
    float final_heading = heading + 53.0f;

    if (final_heading < 0) final_heading += 360.0f;
    if (final_heading >= 360.0f) final_heading -= 360.0f;

    // ---- add smoothing ----
    static float filt = 0.0f;
    filt = filt * 0.90f + heading * 0.10f;  // 90% old, 10% new

    g_telemetry_data.filtered_heading_deg = filt;

    *heading_deg = filt;
    return true;
}

