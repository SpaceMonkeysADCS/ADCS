#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <BasicLinearAlgebra.h>
#include "imu_bno085.h"
#include "motor.h"

// IMU I2C MACROS
#define SDA1_PIN 35
#define SCL1_PIN 36
#define IMU_ADDR 0x4A
#define I2C_HZ 400000

maxon_motor_t x_mot;
maxon_motor_t y_mot;
maxon_motor_t z_mot;

float wheel_rpm[3] = {0, 0, 0};
float tauGB_out[3] = {};

////** GLOBAL PD CONTROLLER VARS **////
float Kp[3][3] = { { 0.3f, 0, 0 }, { 0, 0.3f, 0 }, { 0, 0, 0.3f } };
float Kd[3][3] = { { 0.35f, 0, 0 }, { 0, 0.35f, 0 }, { 0, 0, 0.35f } }; //0.35

float q_e[4] = {};
float q_0[4] = {};
float grav0[3] = {};
float q_Tilt[4] = {};
float desG[3] = { 0, 0, -1 }; // change back to -1
bool q0_set = false;
bool grav_set = false;
float grav[3] = {};
float q_WB[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
float q_BW[4] = {};
float omega[3] = { 0.0f, 0.0f, 0.0f };
float wheel_tau[3] = { 0.0f, 0.0f, 0.0f };

float r_COMB[3] = { 0.0242, -0.022, -0.02365}; //change z back to -0.0236
float r_COMmag = sqrt(r_COMB[0] * r_COMB[0] + r_COMB[1] * r_COMB[1] + r_COMB[2] * r_COMB[2]);
float r_BalW[3] = { 0, 0, -r_COMmag };
float q_des[4] = { 0.31922, 0.00573, -0.89154, 0.321350};

//Initializing gravity vector in world frame
float g = 9.81;
float mass = 0.732;  //Kg
float FgW[3] = { 0, 0, mass* g };

float wheel_I = 5.59e-6;  //kg/m^2
float tense_COM[3][3] = { { 0.00437103, 0.00135069, -0.00152319 }, { 0.00135069, 0.00445678, -0.00151239 }, { -0.00152319, -0.00151239, 0.00376271 } };
////** END GLOBAL PD CONTROLLER VARS **////

////** ESP-NOW SETUP **////
static const uint8_t ESPNOW_CHANNEL = 1;
static const uint8_t RCVR_MAC_ADDR[6] = { 0x1C, 0xDB, 0xD4, 0x9C, 0x35, 0x30 };

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float r, i, j, k;
  float ix, iy, iz;
  float gx, gy, gz;
  float x_rpm, y_rpm, z_rpm;
  float theta;
} pkt;

static pkt tx_pkt;
static uint32_t last_send_ms = 0;
static const uint32_t SEND_PERIOD_MS = 50;
static uint32_t espnow_attempts = 0;
////** END ESP-NOW SETUP **////

void setup() {
  Serial.begin(115200);
  delay(2000);

  bool mot_z_ok = maxon_motor_init(
    &z_mot,
    4,      // pwm
    5,      // enable
    6,      // direction
    0,      // channel
    16,      // speed analog pin
    3000,   // pwm freq
    10,     // num bits
    false,  // invert enable
    true,   // invert direction
    0.00823f);

  bool mot_x_ok = maxon_motor_init(
    &x_mot,
    8,        // pwm
    9,        // enable
    10,       // direction
    0,        // channel
    12,       // speed analog pin
    3000,     // pwm freq
    10,       // num bits 
    false,    // invert enable
    true,     // invert direction
    0.00823f  // kt
  );

  bool mot_y_ok = maxon_motor_init(
    &y_mot,
    18,     // pwm
    21,     // enable
    47,     // direction
    0,      // channel
    14,      // speed analog pin
    3000,   // pwm freq
    10,     // num bits
    false,  // invert enable
    false,  // invert direction
    0.00823f);

  // Initialize motors, but set enable to false; motors are enabled at end of setup
  maxon_motor_enable(&x_mot, true);
  maxon_motor_enable(&y_mot, true);
  maxon_motor_enable(&z_mot, true);

  // send 10% pwm to esc while esp finishes booting to avoid invalid pwm input
  maxon_motor_set_current(&x_mot, 0.0f, 2.8f);
  maxon_motor_set_current(&y_mot, 0.0f, 2.8f);
  maxon_motor_set_current(&z_mot, 0.0f, 2.8f);

  // ** ESPNOW SETUP **//
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RCVR_MAC_ADDR, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ESP-NOW add peer failed");
    while (1) delay(10);
  }
  Serial.println("ESP-NOW ready");
  // ** END ESPNOW SETUP **//

  // Initialize IMU. ** If it fails, it blocks and the system will not work **
  if (!imu_init(SDA1_PIN, SCL1_PIN, IMU_ADDR, I2C_HZ)) {
    Serial.println("BNO085/IMU init FAILED");
    while (1) delay(10);
  }
  Serial.println("BNO085 init OK");
}

void loop() {

  static bool have_q = false;
  static bool have_w = false;
  static bool have_g = false;
  static uint32_t last_us = micros();

  quaternion q;
  velocity_vec v;
  gravity_vec g;

  bool q_ok = get_quaternion(&q);
  bool v_ok = get_angular_velocity(&v);
  bool g_ok = get_gravity(&g);

  float q_curr[4] = {};
  float q_currBW[4] = {};
  float q_rot[4] = {};
  float q_inv[4] = {};

  if (v_ok) {
    omega[0] = v.x;
    omega[1] = v.y;
    omega[2] = v.z;
    have_w = true;
  }

  
  if (g_ok) {
    // if (!grav_set) {
    //   grav0[0] = g.x;
    //   grav0[1] = g.y;
    //   grav0[2] = g.z;
    //   norm(grav0);
    //   quatFromVec(grav0, desG, q_Tilt);
    //   Serial.print("q_tilt 0 ");
    //   Serial.print(q_Tilt[0], 4);
    //   Serial.print(q_Tilt[1], 4);
    //   Serial.print(q_Tilt[2], 4);
    //   Serial.println(q_Tilt[3], 4);
    //   grav_set = true;
    // }
    grav[0] = g.x;
    grav[1] = g.y;
    grav[2] = g.z;
    have_g = true;
  }

  // Update persistent state
  if (q_ok) {
    q_curr[0] = q.r;  //q_WB
    q_curr[1] = q.i;
    q_curr[2] = q.j;
    q_curr[3] = q.k;
    //quatINV(q_curr, q_currBW);
    
    //   if (!q0_set) {
    //     q_0[0] = q_currBW[0];
    //     q_0[1] = q_currBW[1];
    //     q_0[2] = q_currBW[2];
    //     q_0[3] = q_currBW[3];
    //     q0_set = true;
    // }

    // frameReset(q_currBW, q_0, q_Tilt);
    q_BW[0] = q_curr[0];
    q_BW[1] = q_curr[1];
    q_BW[2] = q_curr[2];
    q_BW[3] = q_curr[3];
    have_q = true;
  }

  // Fixed-step timing
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  
  if (have_q && have_w && have_g && dt > 0.0f) {
    last_us = now_us;

    get_wheel_speed(&x_mot, &y_mot, &z_mot, wheel_rpm);

    errorQuaternion(q_BW, q_des, q_e);
    Attitude_PD(q_BW, q_e, omega, Kp, Kd, tense_COM, wheel_tau, FgW, r_COMB, grav, wheel_rpm, tauGB_out);

    // calculate current to send to each motor given torque using K_t
    float ix_cmd = wheel_tau[0] / x_mot.kt;
    float iy_cmd = wheel_tau[1] / y_mot.kt;
    float iz_cmd = wheel_tau[2] / z_mot.kt;

    //controller saturation
    if (ix_cmd > 2.8f) ix_cmd = 2.8f;
    if (ix_cmd < -2.8f) ix_cmd = -2.8f;
    if (iy_cmd > 2.8f) iy_cmd = 2.8f;
    if (iy_cmd < -2.8f) iy_cmd = -2.8f;
    if (iz_cmd > 2.8f) iz_cmd = 2.8f;
    if (iz_cmd < -2.8f) iz_cmd = -2.8f;

    // command motor current to x, y, z motors
    float theta = 2 * acos(fabs(q_e[0]));
    if (theta >= 0.2) {
      ix_cmd = 0.0f;
      iy_cmd = 0.0f;
      iz_cmd = 0.0f;
    }

    // // // TEST
    // static uint32_t last_toggle_ms = 0;
    // static bool y_positive = true;

    // uint32_t now_test_ms = millis();
    // if (now_test_ms - last_toggle_ms >= 500) {
    //   last_toggle_ms = now_test_ms;
    //   y_positive = !y_positive;
    // }
    //  // END TEST
    // float y_test_cmd = y_positive ? 2.5f : -2.5f;

    maxon_motor_set_current(&x_mot, ix_cmd, 2.8f);
    maxon_motor_set_current(&y_mot, iy_cmd, 2.8f);
    maxon_motor_set_current(&z_mot, iz_cmd, 2.8f);


    // Transmit data packet at 20Hz
    uint32_t now_ms = millis();
    if (now_ms - last_send_ms >= SEND_PERIOD_MS) {
      last_send_ms = now_ms;

      tx_pkt.t_ms = now_ms;
      tx_pkt.r = q.r;
      tx_pkt.i = q.i;
      tx_pkt.j = q.j;
      tx_pkt.k = q.k;
      tx_pkt.theta = theta;
      tx_pkt.ix = ix_cmd;
      tx_pkt.iy = iy_cmd;
      tx_pkt.iz = iz_cmd;
      tx_pkt.x_rpm = wheel_rpm[0];
      tx_pkt.y_rpm = wheel_rpm[1];
      tx_pkt.z_rpm = wheel_rpm[2];
      tx_pkt.gx = g_ok ? g.x : 0.0f;
      tx_pkt.gy = g_ok ? g.y : 0.0f;
      tx_pkt.gz = g_ok ? g.z : 0.0f;

      esp_err_t result = esp_now_send(RCVR_MAC_ADDR, (uint8_t*)&tx_pkt, sizeof(tx_pkt));
      if (result != ESP_OK) {
        Serial.println("ESP-NOW send failed");
      }
    }

    // Print rotation and current commands to motors
    Serial.print("ix: ");
    Serial.print(ix_cmd, 4);
    Serial.print("    iy: ");
    Serial.print(iy_cmd, 4);
    Serial.print("    iz: ");
    Serial.print(iz_cmd, 4);
    Serial.print("  theta: ");
    Serial.print(theta, 4);

    Serial.print("y speed: ");
    Serial.print(wheel_rpm[1],4);

    Serial.print("Ang Vel = [ ");
    Serial.print(v.x, 4);
    Serial.print(", ");
    Serial.print(v.y, 4);
    Serial.print(", ");
    Serial.print(v.z, 4);
    Serial.print(" ] ");

    Serial.print(" q raw = [");
    Serial.print(q.r, 4);
    Serial.print(", ");
    Serial.print(q.i, 4);
    Serial.print(", ");
    Serial.print(q.j, 4);
    Serial.print(", ");
    Serial.print(q.k, 4);
    Serial.print("]");

    Serial.print("tau = [");
    Serial.print(tauGB_out[0]);
    Serial.print(", ");
    Serial.print(tauGB_out[1]);
    Serial.print(", ");
    Serial.print(tauGB_out[2]);
    Serial.print("] ");


    Serial.print("  err: ");
    Serial.print(q_e[0], 4);
    Serial.print(q_e[1], 4);
    Serial.print(q_e[2], 4);
    Serial.println(q_e[3], 4);


  }
}