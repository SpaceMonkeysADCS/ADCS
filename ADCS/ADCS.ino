#define OTA 1

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>
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

// ESP-NOW
static const uint8_t RCVR_MAC_ADDR[6] = { 0x1C, 0xDB, 0xD4, 0x9C, 0x35, 0x30 };

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float r, i, j, k;
} quat_pkt_t;  // msg struct for sending quaternions

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

void setup() {
  Serial.begin(115200);
  delay(2000);

bool mot_x_ok = maxon_motor_init(
                &x_mot,
                4, // pwm
                5, // enable
                6, // direction
                0, // channel
                4000, // pwm freq
                10, // num bits
                false, // invert enable
                false // invert direction
);

bool mot_y_ok = maxon_motor_init(
                &y_mot,
                8, // pwm
                9, // enable
                10, // direction
                0, // channel
                4000, // pwm freq
                10, // num bits
                false, // invert enable
                false // invert direction
);

bool mot_z_ok = maxon_motor_init(
                &z_mot,
                18, // pwm
                21, // enable
                47, // direction
                0, // channel
                4000, // pwm freq
                10, // num bits
                false, // invert enable
                false // invert direction
);

// TEST
maxon_motor_enable(&x_mot, true);
maxon_motor_set(&x_mot, true, 0.3f); // 30% speed test CCW

maxon_motor_enable(&y_mot, true);
maxon_motor_set(&y_mot, true, 0.3f); // 30% speed test CCW

maxon_motor_enable(&z_mot, true);
maxon_motor_set(&z_mot, true, 0.3f); // 30% speed test CCW
// END TEST

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

  // Init IMU
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

  quaternion q;
  //velocity_vec v;

  if (get_quaternion(&q)) {

    quat_pkt_t pkt;
    pkt.t_ms = q.t_ms;
    pkt.r = q.r;
    pkt.i = q.i;
    pkt.j = q.j;
    pkt.k = q.k;

    // Send quaternion to receiver esp32
    esp_err_t err = esp_now_send(RCVR_MAC_ADDR, (uint8_t*)&pkt, sizeof(pkt));
    if (err != ESP_OK) {
       Serial.printf("Error sending quaternion: ESP-NOW send err=%d\n", (int)err);
    }    
  }
  delay(10);
}