#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const uint8_t ESPNOW_CHANNEL = 1;

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float r, i, j, k;
  float ix, iy, iz;
  float gx, gy, gz;
  float theta;
} pkt_t;

volatile bool newPkt = false;
pkt_t lastPkt;

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(pkt_t)) return;

  memcpy((void*)&lastPkt, data, sizeof(lastPkt));
  newPkt = true;
}

void setup() {
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }

  esp_now_register_recv_cb(onRecv);

  Serial.println("ESP-NOW receiver ready.");
  Serial.println("t_ms,r,i,j,k,ix,iy,iz,gx,gy,gz");
}

void loop() {
  static uint32_t lastPrint = 0;

  if ((uint32_t)(millis() - lastPrint) < 50) return;
  lastPrint = millis();

  if (!newPkt) return;

  pkt_t p;
  noInterrupts();
  p = lastPkt;
  newPkt = false;
  interrupts();

  // Serial.print("t_ms: ");
  // Serial.print((unsigned long)p.t_ms);
  // Serial.print("  q = [");
  // Serial.print(p.r, 6);
  // Serial.print(", ");
  // Serial.print(p.i, 6);
  // Serial.print(", ");
  // Serial.print(p.j, 6);
  // Serial.print(", ");
  // Serial.print(p.k, 6);
  // Serial.print("]");

  Serial.print(" theta = ");
  Serial.print(p.theta);

  Serial.print("  i_cmd = [");
  Serial.print(p.ix, 6);
  Serial.print(", ");
  Serial.print(p.iy, 6);
  Serial.print(", ");
  Serial.print(p.iz, 6);
  Serial.print("]");

  // Serial.print("  grav_vec = [");
  // Serial.print(p.gx, 4);
  // Serial.print(",");
  // Serial.print(p.gy, 4);
  // Serial.print(",");
  // Serial.print(p.gz, 4);
  // Serial.print("]");
  Serial.println();
}