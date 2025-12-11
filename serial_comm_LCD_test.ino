
/*
 * ESP32-C6-LCD-1.47 (Waveshare) 用
 * PCのUSBシリアル(Serial)で受信した文字列をLCD(ST7789/172x320)に表示
 * フォントは Font4（LovyanGFXの組み込み）を使用。
 * 折り返しは textWidth() により、最下段到達時は全画面クリアの簡易スクロール。
 *
 * 固定ピン（ボード仕様）:
 *   MOSI=GPIO6, SCLK=GPIO7, CS=GPIO14, DC=GPIO15, RST=GPIO21, BL=GPIO22
 */

#include <LovyanGFX.hpp>

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
      cfg.freq_write = 20000000;        // まず20MHz（安定確認後40MHzへ）
      cfg.freq_read  = 10000000;
      cfg.spi_3wire  = false;           // MISO無しのため4-wire想定
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO; // 問題があれば 0 にして切り分け
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
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.invert   = false;             // 色反転に見えるなら true
      cfg.rgb_order = false;            // BGRなら true
      cfg.readable = false;             // 読み出し不可
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

//==================== 表示ロジック ====================
static const int margin = 8;   // 上下左右の余白
int cursorX = margin;
int cursorY = margin;
int lineHeight = 0;

// 文字サイズ（整数倍率）。大きくしたいときは 2 や 3 に変更
int textSize = 1; // 例：2ならFont4の2倍

// 折り返し判定：lineBufを描画せずに幅確認
bool needWrap(const String& s) {
  int w = lcd.textWidth(s);
  return (cursorX + w > lcd.width() - margin);
}

// 改行処理：行送りと最下段チェック
void newLine() {
  cursorX = margin;
  cursorY += lineHeight;
  if (cursorY > lcd.height() - margin - lineHeight) {
    // 一旦全画面クリア（簡易スクロール）
    lcd.fillScreen(TFT_BLACK);
    cursorY = margin;
  }
}

//==================== 受信 → 描画 ====================
String lineBuf;

void flushLine() {
  if (lineBuf.length() == 0) return;
  lcd.setCursor(cursorX, cursorY);
  lcd.print(lineBuf);
  cursorX += lcd.textWidth(lineBuf);
  lineBuf = "";
}

void putCharLCD(char c) {
  if (c == '\r') return;     // CRは無視（Windows端末対策）
  if (c == '\n') {
    flushLine();
    newLine();
    return;
  }

  // 追加後に折り返しが必要なら、いったん出力 → 改行
  String candidate = lineBuf + c;
  if (needWrap(candidate)) {
    flushLine();
    newLine();
  }
  lineBuf += c;
}

//==================== Arduino標準関数 ====================
void setup() {
  // USB CDC 初期化
  Serial.begin(115200);
  delay(300); // CDC初期化待ち（IDEのシリアルモニタ接続タイミング対策）

  // LCD初期化
  lcd.begin();
  lcd.setRotation(1);           // 横表示（論理座標: 320x172）
  lcd.setBrightness(220);
  lcd.fillScreen(TFT_BLACK);

  // フォント設定（Font4）
  lcd.setFont(&fonts::Font4);
  lcd.setTextSize(textSize);    // 倍率指定（1/2/3…）
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextDatum(textdatum_t::top_left);

  // 行高さはフォント＋倍率で計算（getTextSizeは無いのでtextSizeを使用）
  lineHeight = lcd.fontHeight() * textSize;

  // タイトル行
  lcd.setCursor(margin, margin);
  lcd.println("UART->LCD Echo  (Font4)");
  cursorY = margin + lineHeight;
  cursorX = margin;
  lcd.drawLine(margin, cursorY, lcd.width() - margin, cursorY, TFT_DARKGREY);
  cursorY += (lineHeight / 4);  // 少し余白
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    putCharLCD(c);
  }
}
