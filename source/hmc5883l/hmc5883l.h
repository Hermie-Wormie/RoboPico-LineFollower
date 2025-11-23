#pragma once

#define LSM303_ACCEL_ADDR 0x19
#define CTRL_REG1_A       0x20
#define ACCEL_OUT_X_L     0x28

void hmc5883l_init(void);
bool hmc5883l_read_heading_deg(float *heading_deg);
void imu_accel_init(void);
bool imu_read_accel(int16_t *x, int16_t *y, int16_t *z);
