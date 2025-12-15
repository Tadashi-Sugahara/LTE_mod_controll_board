
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include "esp_partition.h"

static const uint32_t BAUD = 115200;

void dumpPartitions() {
  Serial.println("\n[Partitions dump]");
  auto it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it) {
    const esp_partition_t* p = esp_partition_get(it);
    Serial.printf("label=%s type=0x%02x sub=0x%02x offset=0x%08x size=0x%08x\n",
                  p->label, p->type, p->subtype, p->address, p->size);
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);
}

void setup() {
  // 1) シリアル開始＆初期化待ち
  Serial.begin(BAUD);
  // 最大2秒だけ待つ（USB CDCが立ち上がるのを待つ）
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { /* wait for USB CDC */ }
  delay(100);  // 少し落ち着かせる

  Serial.println("\n[BOOT] ESP32-C6 LittleFS read test");

  // 2) 実機のパーティション（Offset/Size）を確認
  dumpPartitions();  // ← spiffs/littlefs相当の offset/size が 0x290000 / 0x160000 になっているか確認

  // 3) LittleFSマウント（必要ならフォーマット許可）
  //    初回は true にするとフォーマットしてからマウント（中身は消えます）
  const bool FORMAT_ON_FAIL = false; // 初回だけ true にする運用も可
  if (!LittleFS.begin(FORMAT_ON_FAIL)) {
    Serial.println("[ERROR] LittleFS mount failed");
    return;
  }
  Serial.println("[INFO] LittleFS mounted");

  // 4) ファイル読込み
  const char* path = "/commands1.txt";
  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.printf("[ERROR] Open failed: %s\n", path);
    return;
  }
  Serial.printf("[INFO] Reading: %s\n", path);
  while (f.available()) {
    String line = f.readStringUntil('\n');
    // 末尾のCR/LFを整理
    while (line.endsWith("\r") || line.endsWith("\n")) {
      line.remove(line.length() - 1);
    }
    Serial.println(line);
  }
  f.close();
  Serial.println("[DONE] File read complete");
}

void loop() {}
