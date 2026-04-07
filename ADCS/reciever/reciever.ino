#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

const char* wifi_ssid = "SteinPhone";
const char* wifi_password = "stein2003";

// Must exactly match sender packet
typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float r, i, j, k;
  float ix, iy, iz;
} pkt_t;

volatile bool newPkt = false;
pkt_t lastPkt;

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(pkt_t)) {
    return;
  }

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
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }

  Serial.print("Receiver IP: ");
  Serial.println(WiFi.localIP());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }

  esp_now_register_recv_cb(onRecv);

  Serial.println("ESP-NOW receiver ready.");
  Serial.println("t_ms,r,i,j,k,ix,iy,iz");
}

void loop() {
  static uint32_t lastPrint = 0;

  // Print at about 20 Hz max to match sender
  if ((uint32_t)(millis() - lastPrint) < 50) return;
  lastPrint = millis();

  if (!newPkt) return;

  pkt_t p;
  noInterrupts();
  p = lastPkt;
  newPkt = false;
  interrupts();

  Serial.print("t_ms: ");
  Serial.print((unsigned long)p.t_ms);
  Serial.print("  q = [");
  Serial.print(p.r, 6);
  Serial.print(", ");
  Serial.print(p.i, 6);
  Serial.print(", ");
  Serial.print(p.j, 6);
  Serial.print(", ");
  Serial.print(p.k, 6);
  Serial.print("]");
  Serial.print("  i_cmd = [");
  Serial.print(p.ix, 6);
  Serial.print(", ");
  Serial.print(p.iy, 6);
  Serial.print(", ");
  Serial.print(p.iz, 6);
  Serial.println("]");
}