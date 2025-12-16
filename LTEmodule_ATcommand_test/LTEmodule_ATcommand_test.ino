
/*
 * ESP32-C6: PCから受信した文字列を行単位にまとめてUART1(TX=GP18, RX=GP19, 9600bps)へ送信
 *           LCDは表示前にクリアして先頭から描画
 *           UART1_RXで受信したレスポンス行をPCのシリアル(USB-CDC)へ表示
 *
 * - PC(USB CDC, Serial 115200)からLF終端の1行を受信
 * - 表示直前に lcdClearBeforeNewLine() を実行して画面クリア＆ホーム
 * - 受信行をLCDに1行表示
 * - 受信行を UTF-8 のまま LF('\n') 終端で UART1 に送信
 * - UART1のレスポンスは LF で行確定（LFが無い機器は受信ギャップで確定）→ PCへ表示
 *
 * UART1: TX=GPIO18, RX=GPIO19, 9600bps, 8N1
 */

#include <LovyanGFX.hpp>

//==================== LCD（Waveshare ESP32-C6-LCD-1.47, ST7789/172x320） ====================
class LGFX_ESP32C6_WS : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
public:
  LGFX_ESP32C6_WS(void) {
    { // SPI
      auto cfg = _bus.config();
      cfg.spi_host   = SPI2_HOST;
      cfg.spi_mode   = 0;
      cfg.freq_write = 20000000;
      cfg.freq_read  = 10000000;
      cfg.spi_3wire  = false;
      cfg.use_lock   = true;
      cfg.dma_channel= SPI_DMA_CH_AUTO;
      cfg.pin_sclk   = 7;   // GPIO7
      cfg.pin_mosi   = 6;   // GPIO6
      cfg.pin_miso   = -1;
      cfg.pin_dc     = 15;  // GPIO15
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { // Panel
      auto cfg = _panel.config();
      cfg.pin_cs     = 14;  // GPIO14
      cfg.pin_rst    = 21;  // GPIO21
      cfg.pin_busy   = -1;
      cfg.memory_width  = 172;
      cfg.memory_height = 320;
      cfg.panel_width   = 172;
      cfg.panel_height  = 320;
      cfg.offset_x  = 0;
      cfg.offset_y  = 0;
      cfg.invert    = false;
      cfg.rgb_order = false;
      cfg.readable  = false;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    { // Backlight
      auto cfg = _light.config();
      cfg.pin_bl = 22;   // GPIO22
      cfg.freq   = 5000;
      cfg.pwm_channel = 0;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};
LGFX_ESP32C6_WS lcd;

//==================== 表示ロジック ====================
static const int margin = 8;
int cursorX = margin;
int cursorY = margin;
int lineHeight = 0;
int textSize   = 1;
int responseLineCount = 0;  // レスポンス行数をカウント

// タイムアウト管理用変数
unsigned long commandSentTime = 0;
unsigned long gpio20LowTime = 0;
bool waitingForResponse = false;
bool gpio20IsLow = false;
const unsigned long RESPONSE_TIMEOUT_MS = 10000;  // 10秒
const unsigned long GPIO20_LOW_DURATION_MS = 1000; // 1秒

// 画面クリア＆ホーム（タイトル再描画オプション付き）
void lcdClearBeforeNewLine(bool drawHeader = true) {
  lcd.fillScreen(TFT_BLACK);
  cursorX = margin;
  cursorY = margin;

  if (drawHeader) {
    lcd.setCursor(margin, margin);
    //lcd.println("PC->LCD(clear) / UART1 TX(LF) / UART1 RX->PC");
    cursorY = margin + lineHeight;
    cursorX = margin;
    lcd.drawLine(margin, cursorY, lcd.width() - margin, cursorY, TFT_DARKGREY);
    cursorY += (lineHeight/4);
  }
}

void lcdNewLine() {
  cursorX = margin;
  cursorY += lineHeight;
  if (cursorY > lcd.height() - margin - lineHeight) {
    lcdClearBeforeNewLine(); // 最下段到達時は全画面クリア
  }
}

void lcdPrintLine(const String& line) {
  lcd.setCursor(cursorX, cursorY);
  lcd.print(line);
  lcdNewLine();
}

//==================== PCから行単位で受信 ====================
/*
 * Serial(USB CDC, 115200)から LF 終端の1行を取得。
 * - CRは無視（CRLF対応）
 * - 受信がない場合は空行("")を返す
 */
String pcReadLine() {
  if (!Serial.available()) return String("");

  String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;     // CRは破棄
    if (c == '\n') break;        // LFで行確定
    line += c;
  }
  return line;
}

//==================== UART設定 ====================
static const uint32_t BAUD_PC   = 115200;
static const uint32_t BAUD_UART = 115200;
static const int UART1_TX_PIN   = 18;  // GP18 = UART1 TX
static const int UART1_RX_PIN   = 19;  // GP19 = UART1 RX

//==================== UART1 受信（LFまたはギャップで行確定） ====================
static std::vector<uint8_t> uart1Buf;
static uint32_t uart1LastByteMs = 0;
static const uint32_t UART1_GAP_MS = 50;  // 受信ギャップ閾値（LFが無い機器用）

/*
 * 受信バイトを取り込み、行が確定したら true と out に行データを返す。
 * 行確定条件：
 *   - LF('\n') を受信して、バッファが非空
 *   - LFが無い機器では、最後の受信から UART1_GAP_MS 経過
 */
bool uart1GetLine(String &out) {
  bool got = false;

  // バイト取り込み
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    uart1LastByteMs = millis();

    if (b == '\r') {           // CRは無視（LFで確定）
      continue;
    }
    if (b == '\n') {           // LFで確定
      if (!uart1Buf.empty()) {
        out.reserve(uart1Buf.size());
        for (auto v : uart1Buf) out += (char)v;
        uart1Buf.clear();
        return true;
      } else {
        // 空行はスキップ
        continue;
      }
    }

    // 通常文字
    uart1Buf.push_back(b);

    // 安全のため最大長（過剰な連続データ対策）
    if (uart1Buf.size() > 4096) {
      out.reserve(uart1Buf.size());
      for (auto v : uart1Buf) out += (char)v;
      uart1Buf.clear();
      return true;
    }
  }

  // LFが無い機器：受信ギャップで確定
  if (!uart1Buf.empty()) {
    uint32_t now = millis();
    if (now - uart1LastByteMs >= UART1_GAP_MS) {
      out.reserve(uart1Buf.size());
      for (auto v : uart1Buf) out += (char)v;
      uart1Buf.clear();
      got = true;
    }
  }
  return got;
}

//==================== Arduino標準 ====================
void setup() {
  // PCシリアル
  Serial.begin(BAUD_PC);
  delay(300);

  // GPIO20を出力モードに設定してHIGHにする
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);
  Serial.println("[GPIO20] Set to HIGH");

  // UART1 (9600, 8N1, TX=18, RX=19)
  Serial1.begin(BAUD_UART, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  Serial1.setRxBufferSize(2048);

  // LCD初期化
  lcd.begin();
  lcd.setRotation(1);
  lcd.setBrightness(220);
  lcd.setFont(&fonts::Font4);
  lcd.setTextSize(textSize);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextDatum(textdatum_t::top_left);

  lineHeight = lcd.fontHeight() * textSize;

  // 初期画面
  lcdClearBeforeNewLine(); // 起動時もヘッダ付きでクリア

  Serial.println("[READY] Type a line (UTF-8). Press Enter (LF).");
}

void loop() {
  // GPIO20の状態管理
  if (gpio20IsLow && (millis() - gpio20LowTime >= GPIO20_LOW_DURATION_MS)) {
    // 1秒経過したらGPIO20をHIGHに戻す
    digitalWrite(20, HIGH);
    gpio20IsLow = false;
    Serial.println("[GPIO20] Set to HIGH (timeout recovery)");
  }
  
  // レスポンスタイムアウト監視
  if (waitingForResponse && (millis() - commandSentTime >= RESPONSE_TIMEOUT_MS)) {
    // 10秒以上レスポンスがない場合
    Serial.println("[TIMEOUT] No response for 10 seconds - Resetting GPIO20");
    digitalWrite(20, LOW);
    gpio20LowTime = millis();
    gpio20IsLow = true;
    waitingForResponse = false;
    Serial.println("[GPIO20] Set to LOW (timeout)");
  }

  // 1) PCから1行受信（LF終端）
  String line = pcReadLine();

  if (line.length() > 0) {
    // 2) 表示前にクリア関数を実行（先頭から描画）
    lcdClearBeforeNewLine(/*drawHeader=*/true);
    responseLineCount = 0;  // レスポンス行数をリセット

    // 3) LCDへ行表示
    lcdPrintLine(line);

    // 4) UART1へ UTF-8 + LF で送信
    Serial.print("[TX->UART1] "); Serial.println(line);
    Serial1.write((const uint8_t*)line.c_str(), line.length());
    Serial1.write('\r'); // LF終端
    
    // タイムアウト監視開始
    commandSentTime = millis();
    waitingForResponse = true;
  }

  // 5) UART1_RX のレスポンス行をPCとLCDに表示（UTF-8としてそのまま）
  String resp;
  if (uart1GetLine(resp)) {
    Serial.print("[RX<-UART1] ");
    Serial.println(resp);
    responseLineCount++;
    
    // レスポンスを受信したのでタイムアウト監視を停止
    if (responseLineCount >= 2) {
      waitingForResponse = false;
    }
    
    if (responseLineCount == 2) {
      // 2回目のレスポンスのみLCDに表示
      lcdPrintLine(">" + resp);
    }
    // 1回目のレスポンスはLCDに表示しない
  }
}