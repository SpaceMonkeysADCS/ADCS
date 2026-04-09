#include "imu_bno085.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO08x.h>

#ifndef BNO08X_RESET
#define BNO08X_RESET -1
#endif

static Adafruit_BNO08x imu(BNO08X_RESET);
static sh2_SensorValue_t data;

// Cached latest values
static quaternion last_quat;
static velocity_vec last_vel;
static bool quat_valid = false;
static bool vel_valid = false;
static gravity_vec last_g;
static bool grav_valid = false;

static void imu_update_cache() {
  // If sensor resets, re-enable reports
  if (imu.wasReset()) {
    imu.enableReport(SH2_GAME_ROTATION_VECTOR);
    imu.enableReport(SH2_GYROSCOPE_CALIBRATED);
  }

  // Read all pending events and update caches
  while (imu.getSensorEvent(&data)) {
    if (data.sensorId == SH2_GAME_ROTATION_VECTOR) {
      last_quat.t_ms = millis();
      last_quat.r = data.un.gameRotationVector.real;
      last_quat.i = data.un.gameRotationVector.i;
      last_quat.j = data.un.gameRotationVector.j;
      last_quat.k = data.un.gameRotationVector.k;
      quat_valid = true;
    } else if (data.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      last_vel.t_ms = millis();
      last_vel.x = data.un.gyroscope.x;
      last_vel.y = data.un.gyroscope.y;
      last_vel.z = data.un.gyroscope.z;
      vel_valid = true;
    } else if (data.sensorId == SH2_GRAVITY) {
      last_g.t_ms = millis();
      last_g.x = data.un.gravity.x;
      last_g.y = data.un.gravity.y;
      last_g.z = data.un.gravity.z;
      grav_valid = true;
    }
  }
}

int imu_init(int sda_pin, int scl_pin, int addr, uint32_t i2c_hz) {
  Wire.begin(sda_pin, scl_pin);
  Wire.setClock(i2c_hz);

  if (!imu.begin_I2C((uint8_t)addr, &Wire)) {
    return 0;
  }

  Wire.setClock(i2c_hz);

  if (!imu.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    return 0;
  }

  if (!imu.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    return 0;
  }

  if (!imu.enableReport(SH2_GRAVITY)) {
    return 0;
  }

  quat_valid = false;
  vel_valid = false;
  grav_valid = false;

  return 1;
}

int get_quaternion(quaternion* out_quat) {
  if (!out_quat) return 0;

  imu_update_cache();

  if (!quat_valid) return 0;

  *out_quat = last_quat;
  return 1;
}

int get_angular_velocity(velocity_vec* out_vel) {
  if (!out_vel) return 0;

  imu_update_cache();

  if (!vel_valid) return 0;

  *out_vel = last_vel;
  return 1;
}

int get_gravity(gravity_vec* out_g)
{
  if (!out_g) return 0;

  imu_update_cache();

  if (!grav_valid) return 0;

  *out_g = last_g;
  return 1;
}