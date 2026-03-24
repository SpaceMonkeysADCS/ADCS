#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

const char* wifi_ssid = "SteinPhone";
const char* wifi_password = "stein2003";

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float r, i, j, k;
} quat_pkt_t;

volatile bool newPkt = false;
quat_pkt_t lastPkt;

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(quat_pkt_t)) return;

  // Copy quickly, no Serial here 
  memcpy((void*)&lastPkt, data, sizeof(lastPkt));
  newPkt = true;
}

void setup() {
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) delay(100);
  Serial.println(WiFi.localIP());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }
  esp_now_register_recv_cb(onRecv);

  Serial.println("ESP-NOW receiver ready.");
  Serial.println("t_ms,   r,    i,    j,    k");
}

void loop() {
  static uint32_t lastPrint = 0;
  if ((uint32_t)(millis() - lastPrint) < 30) return;
  lastPrint += 30;   // keeps a steady 30ms cadence

  // Only print if something new arrived since last print
  if (!newPkt) return;

  quat_pkt_t p;
  noInterrupts();
  p = lastPkt;  
  newPkt = false;
  interrupts();

  Serial.printf("%lu,%.6f,%.6f,%.6f,%.6f\n",
                (unsigned long)p.t_ms, p.r, p.i, p.j, p.k);
}