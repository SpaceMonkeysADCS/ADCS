#include <Arduino.h>
#include "motor.h"
#include <math.h>

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static inline float clampf(float x, float xmin, float xmax) {
  if (x < xmin) return xmin;
  if (x > xmax) return xmax;
  return x;
}

static inline uint32_t max_duty(uint8_t bits) {
  if (bits >= 31) return 0x7FFFFFFF;
  return ((uint32_t)1u << bits) - 1u;
}

bool maxon_motor_init(maxon_motor_t* m,
                      uint8_t pin_pwm, uint8_t pin_en, uint8_t pin_dir,
                      uint8_t pwm_chan,
                      uint32_t pwm_hz, uint8_t pwm_bits,
                      bool invert_en, bool invert_dir, float kt) {
  if (!m) return false;

  m->pin_pwm = pin_pwm;
  m->pin_en = pin_en;
  m->pin_dir = pin_dir;

  m->pwm_chan = pwm_chan;  // kept for compatibility, not used by LEDC 3.x
  m->pwm_hz = pwm_hz;
  m->pwm_bits = pwm_bits;

  m->invert_en = invert_en;
  m->invert_dir = invert_dir;
  m->kt = kt;

  pinMode(m->pin_en, OUTPUT);
  pinMode(m->pin_dir, OUTPUT);

  if (!ledcAttach(m->pin_pwm, m->pwm_hz, m->pwm_bits)) {
    return false;
  }

  maxon_motor_enable(m, false);
  maxon_motor_set_dir_ccw(m, false);  // default CW
  ledcWrite(m->pin_pwm, 0);

  return true;
}

void maxon_motor_enable(maxon_motor_t* m, bool en) {
  if (!m) return;
  bool level = en;
  if (m->invert_en) level = !level;
  digitalWrite(m->pin_en, level ? HIGH : LOW);
}

void maxon_motor_set_dir_ccw(maxon_motor_t* m, bool ccw) {
  if (!m) return;
  bool level = ccw;
  if (m->invert_dir) level = !level;
  digitalWrite(m->pin_dir, level ? HIGH : LOW);
}

void maxon_motor_set_speed(maxon_motor_t* m, float speed01) {
  if (!m) return;

  speed01 = clamp01(speed01);
  // 0.0 -> 10% PWM
  // 1.0 -> 100% PWM
  float pwm_frac = 0.10f + 0.90f * speed01;

  uint32_t duty = (uint32_t)(pwm_frac * (float)max_duty(m->pwm_bits));
  ledcWrite(m->pin_pwm, duty);
}

void maxon_motor_set(maxon_motor_t* m, bool ccw, float speed01) {
  if (!m) return;
  maxon_motor_set_dir_ccw(m, ccw);
  maxon_motor_set_speed(m, speed01);
}

void maxon_motor_set_current(maxon_motor_t* m, float current_A, float current_limit_A) {
  if (!m) return;
  if (current_limit_A <= 0.0f) return;

  // set direction from sign
  bool ccw = (current_A >= 0.0f);
  maxon_motor_set_dir_ccw(m, ccw);

  // magnitude only for PWM command
  float current_mag_A = fabsf(current_A);
  current_mag_A = clampf(current_mag_A, 0.0f, current_limit_A);

  // ESCON mapping:
  // 10% PWM -> 0 A
  // 90% PWM -> current_limit_A
  float current_frac = current_mag_A / current_limit_A;
  float pwm_frac = 0.101f + 0.899f * current_frac; //TODO change 

  uint32_t duty = (uint32_t)(pwm_frac * (float)max_duty(m->pwm_bits));
  ledcWrite(m->pin_pwm, duty);
}