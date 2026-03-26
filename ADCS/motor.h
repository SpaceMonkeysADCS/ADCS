#pragma once
#include <Arduino.h>

typedef struct {
  uint8_t pin_pwm;   // PWM set value input
  uint8_t pin_en;    // Enable input
  uint8_t pin_dir;   // Direction input

  uint8_t pwm_chan;  // kept for compatibility
  uint32_t pwm_hz;
  uint8_t pwm_bits;

  bool invert_en;
  bool invert_dir;
  float kt;
} maxon_motor_t;

bool maxon_motor_init(maxon_motor_t* m,
                      uint8_t pin_pwm, uint8_t pin_en, uint8_t pin_dir,
                      uint8_t pwm_chan,
                      uint32_t pwm_hz, uint8_t pwm_bits,
                      bool invert_en, bool invert_dir, float kt);

void maxon_motor_enable(maxon_motor_t* m, bool en);
void maxon_motor_set_dir_ccw(maxon_motor_t* m, bool ccw);
void maxon_motor_set_speed(maxon_motor_t* m, float speed01);
void maxon_motor_set(maxon_motor_t* m, bool ccw, float speed01);
void maxon_motor_set_current(maxon_motor_t* m, float current_A, float current_limit_A);