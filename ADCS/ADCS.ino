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
float Kp[3][3] = {{20, 0, 0}, {0, 20, 0}, {0, 0, 20}};
float Kd[3][3] = {{7, 0, 0}, {0, 7, 0}, {0, 0, 7}};

float q_e[4] = {};
float q_BW[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float omega[3] = {0.0f, 0.0f, 0.0f};
float wheel_tau[3] = {0.0f, 0.0f, 0.0f};

float r_COMB[3] = {0.04721447, 0.04700558, -0.05316587};
float r_COMmag = sqrt(r_COMB[0]*r_COMB[0] + r_COMB[1]*r_COMB[1] + r_COMB[2]*r_COMB[2]);
float r_BalW[3] = {0, 0, -r_COMmag};
float q_des[4] = {9.010, -0.3060, 0.3074, 0};
float mag_des = sqrt(q_des[0]*q_des[0] + q_des[1]*q_des[1] + q_des[2]*q_des[2] + q_des[3]*q_des[3]);

//Initializing gravity vector in world frame
float g = 9.81;
float mass = 0.612; //Kg
float FgW[3] = {0, 0, mass * g};

float wheel_I = 5.59e-6; //kg/m^2
float tense_COM[3][3] = {{0.00437103, 0.00135069, -0.00152319}, {0.00135069, 0.00445678, -0.00151239}, {-0.00152319, -0.00151239, 0.00376271}};
////** END GLOBAL PD CONTROLLER VARS **////

////** ESP-NOW SETUP **////
static const uint8_t RCVR_MAC_ADDR[6] = { 0x1C, 0xDB, 0xD4, 0x9C, 0x35, 0x30 };

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float r, i, j, k;
} quat_pkt_t; // quaternion packet

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float x, y, z;
} vel_pkt_t; // velocity vector packet
////** END ESP-NOW SETUP **////

////** OTA SOFTWARE UPDATE SETUP **////
#if OTA
static void ota_setup() {
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

  bool mot_x_ok = maxon_motor_init(
    &x_mot,
    4,      // pwm
    5,      // enable
    6,      // direction
    0,      // channel
    4000,   // pwm freq
    10,     // num bits
    false,  // invert enable
    false,   // invert direction
    0.141
  );

  bool mot_y_ok = maxon_motor_init(
    &y_mot,
    8,      // pwm
    9,      // enable
    10,     // direction
    0,      // channel
    4000,   // pwm freq
    10,     // num bits
    false,  // invert enable
    false,   // invert direction
    0.141    // kt
  );

  bool mot_z_ok = maxon_motor_init(
    &z_mot,
    18,     // pwm
    21,     // enable
    47,     // direction
    0,      // channel
    4000,   // pwm freq
    10,     // num bits
    false,  // invert enable
    false,   // invert direction
    0.141
  );

  maxon_motor_enable(&x_mot, true);
  maxon_motor_enable(&y_mot, true);
  maxon_motor_enable(&z_mot, true);
  
  // Normalize desired quaternion
  for (int i = 0; i < 4; i++){
  q_des[i] = q_des[i] / mag_des;
}

// OTA Updates
#if OTA
  ota_setup();
#endif

  // ESPNOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RCVR_MAC_ADDR, 6);
  peer.channel = 0;  // 0 = use current WiFi channel
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ESP-NOW add peer failed");
    while (1) delay(10);
  }
  Serial.println("ESP-NOW ready");

  // Initialize IMU
  if (!imu_init(SDA1_PIN, SCL1_PIN, IMU_ADDR, I2C_HZ)) {
    Serial.println("BNO085/IMU init FAILED");
    while (1) delay(10);
  }
  Serial.println("BNO085 init OK");
}

void loop() {
#if OTA
  ArduinoOTA.handle();
#endif

  static bool have_q = false;
  static bool have_w = false;
  static uint32_t last_us = micros();

  quaternion q;
  velocity_vec v;

  bool q_ok = get_quaternion(&q);
  bool v_ok = get_angular_velocity(&v);

  // Update persistent state
  if (q_ok) {
    q_BW[0] = q.r;
    q_BW[1] = q.j;
    q_BW[2] = -1.0f * q.i;
    q_BW[3] = q.k;
    have_q = true;
  }

  if (v_ok) {
    omega[0] = v.y;
    omega[1] = -1.0f * v.x;
    omega[2] = v.z;
    have_w = true;
  }

  // Fixed-step timing
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;

  if (have_q && have_w && dt > 0.0f) {
    last_us = now_us;

    errorQuaternion(q_BW, q_des, q_e);
    Attitude_PD(q_BW, q_e, omega, Kp, Kd, tense_COM, wheel_tau, FgW, r_COMB);

    float ix_cmd = wheel_tau[0] / x_mot.kt;
    float iy_cmd = wheel_tau[1] / y_mot.kt;
    float iz_cmd = wheel_tau[2] / z_mot.kt;

    //controller saturation
    if (ix_cmd >  2.8f) ix_cmd =  2.8f;
    if (ix_cmd < -2.8f) ix_cmd = -2.8f;
    if (iy_cmd >  2.8f) iy_cmd =  2.8f;
    if (iy_cmd < -2.8f) iy_cmd = -2.8f;
    if (iz_cmd >  2.8f) iz_cmd =  2.8f;
    if (iz_cmd < -2.8f) iz_cmd = -2.8f;

    maxon_motor_set_current(&x_mot, ix_cmd, 2.8f);
    maxon_motor_set_current(&y_mot, iy_cmd, 2.8f);
    maxon_motor_set_current(&z_mot, iz_cmd, 2.8f);
  }
}