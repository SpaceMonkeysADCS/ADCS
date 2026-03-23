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

int imu_init(int sda_pin, int scl_pin, int addr, uint32_t i2c_hz)
{
  Wire.begin(sda_pin, scl_pin);
  Wire.setClock(i2c_hz);

  if (!imu.begin_I2C((uint8_t)addr, &Wire)) {
    return 0;
  }

  Wire.setClock(i2c_hz);

  // fused orientation quaternion
  if (!imu.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    return 0;
  }

  return 1;
}

int get_quaternion(quaternion* out_quat) {
  if (!out_quat) return 0;

  // If sensor resets, re-enable report
  if (imu.wasReset())
  {
    imu.enableReport(SH2_GAME_ROTATION_VECTOR);
  }

  if (!imu.getSensorEvent(&data))
  {
    return 0;
  }

  if (data.sensorId != SH2_GAME_ROTATION_VECTOR) {
    return 0;
  }

  out_quat->t_ms = millis();
  out_quat->r = data.un.gameRotationVector.real;
  out_quat->i = data.un.gameRotationVector.i;
  out_quat->j = data.un.gameRotationVector.j;
  out_quat->k = data.un.gameRotationVector.k;

  return 1;
}
