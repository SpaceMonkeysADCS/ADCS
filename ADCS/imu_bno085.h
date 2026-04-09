#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t t_ms;
  float r, i, j, k;
} quaternion;

typedef struct{
  uint32_t t_ms;
  float x, y, z; 
} velocity_vec;

typedef struct {
  uint32_t t_ms;
  float x, y, z;
} gravity_vec;

/**
 * Initialize BNO085 on I2C using given SDA/SCL pins.
 * addr is usually 0x4A or 0x4B.
 * Returns 1 on success, 0 on failure.
 */
int imu_init(int sda_pin, int scl_pin, int addr, uint32_t i2c_hz);

/**
 * Poll for a new quaternion.
 * Returns 1 if out_quat was written with a NEW sample, else 0.
 */
int get_quaternion(quaternion* out_quat);

/**
 * Poll for a new velocity vector.
 * Returns 1 if out_vel was written with a NEW sample, else 0.
 */
int get_angular_velocity(velocity_vec* out_vel);

int get_gravity(gravity_vec* out_g);



#ifdef __cplusplus
}
#endif