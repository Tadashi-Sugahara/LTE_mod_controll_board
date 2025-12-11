
/*
 * ESP32-C6-LCD-1.47 (Waveshare) 行単位送信版（重複排除済み）
 * PC(USB CDC, 115200)からのUTF-8テキストをLCDへ表示。
 * 新しい行の受信開始時にLCDをクリアして先頭から描画。
 * 改行(LF)確定時に外部UART1(9600bps, 8N1, TX=GPIO18, RX=GPIO19)へ
 * UTF-8の生バイト + LF("\n") を送信。最低1200ms応答待ち→デコード→PCへ表示。
 *
 * LCDピン（固定）:
 *   MOSI=GPIO6, SCLK=GPIO7, CS=GPIO14, DC=GPIO15, RST=GPIO21, BL=GPIO22
 */

#include <LovyanGFX.hpp>
#include <vector>
#include <stdlib.h>  // strtoul
#include <stdio.h>   // snprintf
#include <ctype.h>   // isspace

//==================== UART設定 ====================
static const uint32_t BAUD_PC        = 115200;          // PC(USB CDC)
static const uint32_t BAUD_UART      = 9600;            // 外部UART
static const uint32_t RX_TIMEOUT_MS  = 1200;            // 応答待ち(最低1秒以上)
static const uint32_t UART_FMT       = SERIAL_8N1;      // 8N1
static const int      RX1_PIN        = 19;              // 機器TX -> ESP32-C6 RX
static const int      TX1_PIN        = 18;              // 機器RX <- ESP32-C6 TX
static const char     UART_EOL       = '\n';            // 送信終端: LFのみ
static const uint32_t UART1_GAP_MS   = 50;              // 改行が無い機器用の受信ギャップ閾値

//==================== ボード固有設定（LovyanGFX） ====================
class LGFX_ESP32C6_WS : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
public:
  LGFX_ESP32C6_WS(void) {
    { // SPIバス設定
      auto cfg = _bus.config();
      cfg.spi_host   = SPI2_HOST;       // ESP32-C6はSPI2/SPI3
      cfg.spi_mode   = 0;
      cfg.freq_write = 20000000;        // まず20MHz
      cfg.freq_read  = 10000000;
      cfg.spi_3wire  = false;           // MISO無しのため4-wire想定
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk   = 7;               // GPIO7 SCLK
      cfg.pin_mosi   = 6;               // GPIO6 MOSI
      cfg.pin_miso   = -1;              // なし
      cfg.pin_dc     = 15;              // GPIO15 DC
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { // パネル設定
      auto cfg = _panel.config();
      cfg.pin_cs     = 14;              // GPIO14 CS
      cfg.pin_rst    = 21;              // GPIO21 RST
      cfg.pin_busy   = -1;
      cfg.memory_width  = 172;
      cfg.memory_height = 320;
      cfg.panel_width   = 172;
      cfg.panel_height  = 320;
      cfg.offset_x  = 0;
      cfg.offset_y  = 0;
      cfg.offset_rotation = 0;
      cfg.invert    = false;            // 色反転に見えるなら true
      cfg.rgb_order = false;            // BGRなら true
      cfg.readable  = false;            // 読み出し不可
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    { // バックライトPWM（GPIO22）
      auto cfg = _light.config();
      cfg.pin_bl = 22;
      cfg.freq   = 5000;                // 5kHz
      cfg.pwm_channel = 0;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

LGFX_ESP32C6_WS lcd;

//==================== 表示ロジック（単一定義） ====================
static const int margin = 8;   // 上下左右の余白
int cursorX = margin;
int cursorY = margin;
int lineHeight = 0;
int textSize = 1; // 文字倍率：大きくしたいときは 2 や 3
bool newInputLine = true;      // 新しい行の先頭かどうか

String lineBuf;                // LCD用の現在行バッファ（例と同じ命名）
String pcLineAccum;            // UART送信用の行バッファ（UTF-8）
bool   awaitingResponse = false;

// 折り返し判定
bool needWrap(const String& s) {
  int w = lcd.textWidth(s);
  return (cursorX + w > lcd.width() - margin);
}
// 改行処理
void newLine() {
  cursorX = margin;
  cursorY += lineHeight;
  if (cursorY > lcd.height() - margin - lineHeight) {
    lcd.fillScreen(TFT_BLACK); // 簡易スクロール
    cursorY = margin;
  }
}
void clearAndHome() {
  lcd.fillScreen(TFT_BLACK);
  cursorX = margin;
  cursorY = margin;
}
// バッファを描画
void flushLine() {
  if (lineBuf.length() == 0) return;
  lcd.setCursor(cursorX, cursorY);
  lcd.print(lineBuf);
  cursorX += lcd.textWidth(lineBuf);
  lineBuf = "";
}

//==================== UART1 行アセンブラ（受信） ====================
static std::vector<uint8_t> uart1Buf;
static uint32_t uart1LastByteMs = 0;
static String   lastDisplayedLine;

bool uart1GetLine(std::vector<uint8_t> &out) {
  bool got = false;
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    uart1LastByteMs = millis();
    if (b == '\r') { continue; }          // CRは無視（LFで確定）
    if (b == '\n') {                       // LFで行確定
      if (!uart1Buf.empty()) {
        out = uart1Buf; uart1Buf.clear(); return true;
      } else {
        continue; // 空行スキップ
      }
    }
    uart1Buf.push_back(b);
    if (uart1Buf.size() > 4096) {         // 安全のため最大長
      out = uart1Buf; uart1Buf.clear(); return true;
    }
  }
  // 改行がない機器：受信ギャップで確定
  if (!uart1Buf.empty()) {
    uint32_t now = millis();
    if (now - uart1LastByteMs >= UART1_GAP_MS) {
      out = uart1Buf; uart1Buf.clear(); got = true;
    }
  }
  return got;
}
bool uart1WaitLine(std::vector<uint8_t> &out, uint32_t totalTimeoutMs) {
  uint32_t start = millis();
  while (millis() - start < totalTimeoutMs) {
    if (uart1GetLine(out)) return true;
    delay(5);
  }
  return false;
}

//==================== 受信デコードユーティリティ ====================
bool isHexChar(char c) {
  return (c >= '0' && c <= '9')
      || (c >= 'A' && c <= 'F')
      || (c >= 'a' && c <= 'f');
}
bool looksLikeAsciiHex(const uint8_t* d, size_t n) {
  if (n < 2) return false;
  size_t hexCount = 0;
  for (size_t i = 0; i < n; ++i) {
    char c = (char)d[i];
    if (isHexChar(c)) hexCount++;
    else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') { /* 許容 */ }
    else return false;
  }
  return ((double)hexCount / (double)n) >= 0.8;
}
size_t decodeAsciiHexToBytes(const uint8_t* d, size_t n, std::vector<uint8_t>& out) {
  out.clear();
  String s; s.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    char c = (char)d[i];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
    s += c;
  }
  if (s.length() < 2) return 0;
  if (s.length() % 2) s.remove(s.length() - 1);
  for (size_t i = 0; i < s.length(); i += 2) {
    char bstr[3] = { s[i], s[i+1], 0 };
    uint8_t val = (uint8_t)strtoul(bstr, nullptr, 16);
    out.push_back(val);
  }
  return out.size();
}
bool isValidUtf8(const uint8_t* d, size_t n) {
  size_t i = 0;
  while (i < n) {
    uint8_t c = d[i];
    if (c <= 0x7F) { i++; }
    else if ((c & 0xE0) == 0xC0) {
      if (i + 1 >= n) return false;
      if ((d[i+1] & 0xC0) != 0x80) return false;
      if (c < 0xC2) return false;
      i += 2;
    }
    else if ((c & 0xF0) == 0xE0) {
      if (i + 2 >= n) return false;
      uint8_t c1 = d[i+1], c2 = d[i+2];
      if ((c1 & 0xC0) != 0x80 || (c2 & 0xC0) != 0x80) return false;
      if (c == 0xE0 && c1 < 0xA0) return false;
      if (c == 0xED && c1 >= 0xA0) return false;
      i += 3;
    }
    else if ((c & 0xF8) == 0xF0) {
      if (i + 3 >= n) return false;
      uint8_t c1 = d[i+1], c2 = d[i+2], c3 = d[i+3];
      if ((c1 & 0xC0) != 0x80 || (c2 & 0xC0) != 0x80 || (c3 & 0xC0) != 0x80) return false;
      if (c == 0xF0 && c1 < 0x90) return false;
      if (c > 0xF4 || (c == 0xF4 && c1 > 0x8F)) return false;
      i += 4;
    }
    else { return false; }
  }
  return true;
}
void pcPrintHexWithAscii(const char* label, const uint8_t* d, size_t n) {
  Serial.print(label); Serial.print(" HEX: ");
  for (size_t i = 0; i < n; ++i) {
    char buf[4]; snprintf(buf, sizeof(buf), "%02X", d[i]);
    Serial.print(buf); Serial.print(' ');
  }
  Serial.print("\n");
  Serial.print(label); Serial.print(" ASCII: ");
  for (size_t i = 0; i < n; ++i) {
    char c = (char)d[i];
    Serial.print((c >= 0x20 && c <= 0x7E) ? c : '.');
  }
  Serial.print("\n");
}
void pcSmartDisplayBytes(const std::vector<uint8_t>& bytes) {
  const uint8_t* d = bytes.data();
  size_t n = bytes.size();
  if (looksLikeAsciiHex(d, n)) {
    std::vector<uint8_t> dec;
    size_t m = decodeAsciiHexToBytes(d, n, dec);
    if (m > 0 && isValidUtf8(dec.data(), m)) {
      String utf8; utf8.reserve(m);
      for (size_t i = 0; i < m; ++i) utf8 += (char)dec[i];
      Serial.print("[RX<-UART1 decoded] "); Serial.println(utf8);
      return;
    }
    pcPrintHexWithAscii("[RX<-UART1]", d, n);
    return;
  }
  if (isValidUtf8(d, n)) {
    String utf8; utf8.reserve(n);
    for (size_t i = 0; i < n; ++i) utf8 += (char)d[i];
    Serial.print("[RX<-UART1] "); Serial.println(utf8);
    return;
  }
  pcPrintHexWithAscii("[RX<-UART1]", d, n);
}

//==================== 行送信＆待受（LF確定時に呼ぶ） ====================
void sendLineToUART1AndWait(const String& utf8Line) {
  Serial.print("[TX->UART1] "); Serial.println(utf8Line);     // PCへエコー
  Serial1.write((const uint8_t*)utf8Line.c_str(), utf8Line.length()); // UTF-8生バイト
  Serial1.write((uint8_t)UART_EOL);                                       // LFのみ追加

  awaitingResponse = true;
  std::vector<uint8_t> resp;
  if (uart1WaitLine(resp, RX_TIMEOUT_MS)) {
    pcSmartDisplayBytes(resp);
    // デュープ抑止用（任意）
    String last; for (auto b : resp) last += (char)b;
    lastDisplayedLine = last;
  } else {
    Serial.println("[RX<-UART1] (timeout/no data)");
  }
  awaitingResponse = false;
}

//==================== ★LCD表示＋行バッファ蓄積（例準拠） ====================
void putCharLCD(char c) {
  if (c == '\r') return; // CRは無視（Windows端末対策）

  if (c == '\n') {
    // --- 行確定：LCDの残りを出力して改行 ---
    flushLine();
    newLine();

    // --- UART送信（行単位、UTF-8 + LF） ---
    if (pcLineAccum.length() > 0) {
      sendLineToUART1AndWait(pcLineAccum);
      pcLineAccum = "";  // 次行へ
    }

    // 次の入力は新しい行の開始として扱う
    newInputLine = true;
    return;
  }

  // --- 新規行の先頭文字なら、LCDをクリアして先頭へ ---
  if (newInputLine) {
    clearAndHome();
    lineBuf = "";
    newInputLine = false;
  }

  // --- 折り返し判定：追加後に溢れるならいったん描画して改行 ---
  String candidate = lineBuf + c;
  if (needWrap(candidate)) {
    flushLine();
    newLine();
  }

  // --- バッファへ追加（例のスタイル） ---
  lineBuf += c;

  // --- 送信用行バッファへも蓄積（LFでまとめて送る） ---
  pcLineAccum += c;
}

//==================== Arduino標準関数 ====================
void setup() {
  Serial.begin(BAUD_PC);
  delay(300); // CDC初期化待ち

  Serial1.begin(BAUD_UART, UART_FMT, RX1_PIN, TX1_PIN);
  Serial1.setRxBufferSize(2048);

  // LCD初期化
  lcd.begin();
  lcd.setRotation(1);           // 横表示（論理座標: 320x172）
  lcd.setBrightness(220);
  lcd.fillScreen(TFT_BLACK);
  lcd.setFont(&fonts::Font4);
  lcd.setTextSize(textSize);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextDatum(textdatum_t::top_left);

  // 行高さ
  lineHeight = lcd.fontHeight() * textSize;

  // 新しい行の開始待ち
  newInputLine = true;

  Serial.println("[READY] Line-based TX: UTF-8 + LF. Type a line and press Enter.");
}

void loop() {
  // PC(USB CDC)からの文字受信 → LCD表示＆送信用バッファ格納
  while (Serial.available()) {
    char c = (char)Serial.read();
    putCharLCD(c);
  }

  // 自発的なUART1受信（応答以外も取りこぼさない）
  if (!awaitingResponse) {
    std::vector<uint8_t> line;
    if (uart1GetLine(line)) {
      String now; for (auto b : line) now += (char)b;
      if (now == lastDisplayedLine) {
        // 連続同一ならスキップ（必要なら無効化）
      } else {
        pcSmartDisplayBytes(line); // PCへ出力
        lastDisplayedLine = now;
      }
    }
  }
}
