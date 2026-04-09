#define OTA 1

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>
#include <BasicLinearAlgebra.h>
#include "imu_bno085.h"
#include "motor.h"

#if OTA
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* wifi_ssid = "SteinPhone";
const char* wifi_password = "stein2003";
#endif

// IMU I2C MACROS
#define SDA1_PIN 35
#define SCL1_PIN 36
#define IMU_ADDR 0x4A
#define I2C_HZ 400000

maxon_motor_t x_mot;
maxon_motor_t y_mot;
maxon_motor_t z_mot;

////** GLOBAL PD CONTROLLER VARS **////
float Kp[3][3] = { { .05, 0, 0 }, { 0, .05, 0 }, { 0, 0, .05 } };
float Kd[3][3] = { { 0.05, 0, 0 }, { 0, 0.05, 0 }, { 0, 0, 0.05 } };

float q_e[4] = {};
float q_0[4] = {};
bool q0_set = false;
float q_BW[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
float omega[3] = { 0.0f, 0.0f, 0.0f };
float wheel_tau[3] = { 0.0f, 0.0f, 0.0f };

float r_COMB[3] = { 0.0490, -0.0471, -0.0562};
float r_COMmag = sqrt(r_COMB[0] * r_COMB[0] + r_COMB[1] * r_COMB[1] + r_COMB[2] * r_COMB[2]);
float r_BalW[3] = { 0, 0, -r_COMmag };
float q_des[4] = { 0.9048, 0.2954, 0.3070, 0.0 };
float mag_des = sqrt(q_des[0] * q_des[0] + q_des[1] * q_des[1] + q_des[2] * q_des[2] + q_des[3] * q_des[3]);

//Initializing gravity vector in world frame
float g = 9.81;
float mass = 0.732;  //Kg
float FgW[3] = { 0, 0, mass* g };

float wheel_I = 5.59e-6;  //kg/m^2
float tense_COM[3][3] = { { 0.00437103, 0.00135069, -0.00152319 }, { 0.00135069, 0.00445678, -0.00151239 }, { -0.00152319, -0.00151239, 0.00376271 } };
////** END GLOBAL PD CONTROLLER VARS **////

////** ESP-NOW SETUP **////
static const uint8_t RCVR_MAC_ADDR[6] = { 0x1C, 0xDB, 0xD4, 0x9C, 0x35, 0x30 };

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float r, i, j, k;
  float ix, iy, iz;
  float gx, gy, gz;
} pkt;

static pkt tx_pkt;
static uint32_t last_send_ms = 0;
static const uint32_t SEND_PERIOD_MS = 50;
static uint32_t espnow_attempts = 0;
////** END ESP-NOW SETUP **////

////** OTA SOFTWARE UPDATE SETUP **////
#if OTA
static void ota_setup() {
  Serial.println("ota start");
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  unsigned int nAttempts = 0;
  while (WiFi.waitForConnectResult() != WL_CONNECTED && nAttempts < 3) {
    delay(1000);
    nAttempts++;
  }

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("OTA: WiFi connect FAILED (OTA unavailable).");
    return;
  }

  ArduinoOTA.setHostname("CubeSAT-ESP32");
  Serial.print("OTA: Connected (IP ");
  Serial.print(WiFi.localIP());
  Serial.println(")");
  ArduinoOTA.begin();
}
#endif
////** END OTA SOFTWARE UPDATE SETUP **////

void setup() {
  Serial.begin(115200);
  delay(2000);

  bool mot_z_ok = maxon_motor_init(
    &z_mot,
    4,      // pwm
    5,      // enable
    6,      // direction
    0,      // channel
    3000,   // pwm freq
    10,     // num bits
    false,  // invert enable
    true,   // invert direction
    0.00823);

  bool mot_x_ok = maxon_motor_init(
    &x_mot,
    8,      // pwm
    9,      // enable
    10,     // direction
    0,      // channel
    3000,   // pwm freq
    10,     // num bits
    false,  // invert enable
    true,   // invert direction
    0.00823   // kt
  );

  bool mot_y_ok = maxon_motor_init(
    &y_mot,
    18,     // pwm
    21,     // enable
    47,     // direction
    0,      // channel
    3000,   // pwm freq
    10,     // num bits
    false,  // invert enable
    false,  // invert direction
    0.00823);

  // Initialize motors, but set enable to false; motors are enabled at end of setup
  maxon_motor_enable(&x_mot, false);
  maxon_motor_enable(&y_mot, false);
  maxon_motor_enable(&z_mot, false);

  // send 10% pwm to esc while esp finishes booting to avoid invalid pwm input
  maxon_motor_set_current(&x_mot, 0.0f, 2.8f);
  maxon_motor_set_current(&y_mot, 0.0f, 2.8f);
  maxon_motor_set_current(&z_mot, 0.0f, 2.8f);

  // Normalize desired quaternion
  for (int i = 0; i < 4; i++) {
    q_des[i] = q_des[i] / mag_des;
  }

// OTA Updates
#if OTA
  ota_setup();
#endif

  // ESPNOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, trying again...");
    while (1) delay(10);
  }

  // Add peer esp32 (i.e. the receiver connected to debugging laptop)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RCVR_MAC_ADDR, 6);
  peer.channel = 0;  // 0 = use current WiFi channel
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ESP-NOW add peer failed");
    while (1) delay(10);
  }
  Serial.println("ESP-NOW ready");

  // Initialize IMU. ** If it fails, it blocks and the system will not work **
  if (!imu_init(SDA1_PIN, SCL1_PIN, IMU_ADDR, I2C_HZ)) {
    Serial.println("BNO085/IMU init FAILED");
    while (1) delay(10);
  }
  Serial.println("BNO085 init OK");

  // Enable motors
  maxon_motor_enable(&x_mot, true);
  maxon_motor_enable(&y_mot, true);
  maxon_motor_enable(&z_mot, true);
}

void loop() {
#if OTA
  ArduinoOTA.handle();
#endif

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
  float q_rot[4] = {};
  float q_inv[4] = {};

  // Update persistent state
  if (q_ok) {
    q_curr[0] = q.r;
    q_curr[1] = q.i;
    q_curr[2] = q.j;
    q_curr[3] = q.k;

    if (!q0_set) {
      q_0[0] = q_curr[0];
      q_0[1] = q_curr[1];
      q_0[2] = q_curr[2];
      q_0[3] = q_curr[3];
      q0_set = true;
    }
    quatINV(q_0, q_inv);
    quatMult(q_inv, q_curr, q_rot);
    q_BW[0] = q_rot[0];
    q_BW[1] = q_rot[1];
    q_BW[2] = q_rot[2];
    q_BW[3] = q_rot[3];
    have_q = true;
  }

  if (v_ok) {
    omega[0] = v.x;
    omega[1] = v.y;
    omega[2] = v.z;
    have_w = true;
  }

  if(g_ok)
  {
    have_g = true;
  }

  // Fixed-step timing
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;

  if (have_q && have_w && have_g && dt > 0.0f) {
    last_us = now_us;

    errorQuaternion(q_BW, q_des, q_e);
    Attitude_PD(q_BW, q_e, omega, Kp, Kd, tense_COM, wheel_tau, FgW, r_COMB);

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
    if (theta >= 0.06) {
      ix_cmd = 0.0f;
      iy_cmd = 0.0f;
      iz_cmd = 0.0f;
    }
    maxon_motor_set_current(&x_mot, ix_cmd, 2.8f);
    maxon_motor_set_current(&y_mot, iy_cmd, 2.8f);
    maxon_motor_set_current(&z_mot, iz_cmd, 2.8f);

    // Transmit data packet at 20Hz
    uint32_t now_ms = millis();
    if (now_ms - last_send_ms >= SEND_PERIOD_MS) {
      last_send_ms = now_ms;

      tx_pkt.t_ms = now_ms;
      tx_pkt.r = q_BW[0];
      tx_pkt.i = q_BW[1];
      tx_pkt.j = q_BW[2];
      tx_pkt.k = q_BW[3];
      tx_pkt.ix = ix_cmd;
      tx_pkt.iy = iy_cmd;
      tx_pkt.iz = iz_cmd;
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

    Serial.print("Ang Vel = [ ");
    Serial.print(v.x, 4);
    Serial.print(", ");
    Serial.print(v.y, 4);
    Serial.print(", ");
    Serial.print(v.z, 4);
    Serial.print(" ] ");

    Serial.print("  q mag: ");
    Serial.print(q_rot[0], 4);
    Serial.print("  q x: ");
    Serial.print(q_rot[1], 4);
    Serial.print("  q y: ");
    Serial.print(q_rot[2], 4);
    Serial.print("  q z: ");
    Serial.print(q_rot[3], 4);

    Serial.print("  err: ");
    Serial.print(q_e[0], 4);
    Serial.print(q_e[1], 4);
    Serial.print(q_e[2], 4);
    Serial.println(q_e[3], 4);
  }
}