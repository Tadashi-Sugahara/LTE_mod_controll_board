
/*
 * ESP32-C6: LittleFSのcommands1.txtからATコマンドを順次読み込み、UART1経由で送信
 *           "#"で始まる行はスキップ、LCDに実行コマンドを表示、レスポンスをPCに表示
 *           レスポンス受信完了後にLCDをクリアして次のコマンドを実行
 *
 * - LittleFSのcommands1.txtから1行ずつATコマンドを読み込み
 * - "#"で始まる行（コメント行）はスキップ
 * - LCDに実行するATコマンドを表示
 * - ATコマンドをUART1(TX=GPIO18, RX=GPIO19)に送信
 * - UART1のレスポンスをPCのシリアル(USB-CDC)に表示
 * - レスポンス受信完了後、LCDをクリアして次のコマンドを実行
 *
 * UART1: TX=GPIO18, RX=GPIO19, 115200bps, 8N1
 */

#include <LovyanGFX.hpp>
#include <LittleFS.h>

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
unsigned long gpio20LowTime = 0;
bool gpio20IsLow = false;
const unsigned long GPIO20_LOW_DURATION_MS = 1000; // 1秒

// 画面クリア＆ホーム（タイトル再描画オプション付き）
void lcdClearBeforeNewLine(bool drawHeader = true) {
  lcd.fillScreen(TFT_BLACK);
  cursorX = margin;
  cursorY = margin;

  if (drawHeader) {
    lcd.setCursor(margin, margin);
    lcd.println("AT Command Executor");
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

//==================== ファイル読み込み & ATコマンド実行 ====================
static int currentLineIndex = 0;
static std::vector<String> commandLines;
bool commandsLoaded = false;
bool waitingForCommandResponse = false;
unsigned long commandSentTime = 0;
const unsigned long COMMAND_RESPONSE_TIMEOUT_MS = 10000;  // 10秒

// SOCKET文字列比較用変数
String sendDataString = "";
String receiveDataString = "";
bool hasSendData = false;
bool hasReceiveData = false;

// RSRP情報保存用変数
String rsrpInfo = "";
bool hasRsrpInfo = false;

// コマンドファイルを読み込み
void loadCommandFile() {
  if (!LittleFS.begin(true)) {
    Serial.println("[ERROR] Failed to mount LittleFS");
    return;
  }

  File file = LittleFS.open("/commands1.txt", "r");
  if (!file) {
    Serial.println("[ERROR] Failed to open commands1.txt");
    return;
  }

  commandLines.clear();
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();  // 改行コードや空白を除去
    if (line.length() > 0) {
      commandLines.push_back(line);
    }
  }
  file.close();
  
  Serial.printf("[INFO] Loaded %d lines from commands1.txt\n", commandLines.size());
  commandsLoaded = true;
  currentLineIndex = 0;
}

// 次のATコマンドを送信
void sendNextCommand() {
  if (!commandsLoaded || currentLineIndex >= commandLines.size()) {
    Serial.println("[INFO] All commands completed");
    lcdClearBeforeNewLine();
    lcdPrintLine("All commands completed!");
    
    // 文字列比較結果表示
    if (hasSendData && hasReceiveData) {
      if (sendDataString.equals(receiveDataString)) {
        lcdPrintLine("String MATCH!");
        Serial.println("[INFO] SEND/RECEIVE strings MATCH! (26 chars)");
      } else {
        lcdPrintLine("String MISMATCH");
        Serial.printf("[INFO] SEND/RECEIVE strings MISMATCH (26 chars) - SEND: %s, RECEIVE: %s\n", sendDataString.c_str(), receiveDataString.c_str());
      }
    } else {
      lcdPrintLine("No data to compare");
    }
    
    // RSRP情報表示
    if (hasRsrpInfo) {
      lcdPrintLine(rsrpInfo);
    }
    return;
  }

  String command = commandLines[currentLineIndex];
  
  // "#"で始まる行はスキップ
  if (command.startsWith("#")) {
    Serial.printf("[SKIP] Comment line: %s\n", command.c_str());
    currentLineIndex++;
    sendNextCommand();  // 再帰的に次のコマンドをチェック
    return;
  }

  // SOCKETDATA SENDコマンドの文字列を抽出
  if (command.indexOf("AT%SOCKETDATA=\"SEND\",1,13,") >= 0) {
    int startPos = command.indexOf(",13,") + 4;
    if (startPos > 3 && startPos < command.length()) {
      sendDataString = command.substring(startPos);
      sendDataString.replace("\"", ""); // クォーテーションを削除
      // 先頭26文字に制限
      if (sendDataString.length() > 26) {
        sendDataString = sendDataString.substring(0, 26);
      }
      hasSendData = true;
      Serial.printf("[INFO] SEND data captured (26 chars): %s\n", sendDataString.c_str());
    }
  }

  // LCDクリア & コマンド表示
  lcdClearBeforeNewLine();
  lcdPrintLine("CMD: " + command);

  // UART1へ送信
  Serial.printf("[TX->UART1] %s\n", command.c_str());
  Serial1.print(command);
  Serial1.print("\r\n");

  // レスポンス待ち状態へ
  waitingForCommandResponse = true;
  commandSentTime = millis();
  responseLineCount = 0;
  currentLineIndex++;
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

  // UART1 (115200, 8N1, TX=18, RX=19)
  Serial1.begin(BAUD_UART, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  Serial1.setRxBufferSize(2048);

  // LCD初期化前の待機時間（6秒）
  delay(6000);

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
  lcdClearBeforeNewLine(); 
  lcdPrintLine("Loading commands...");

  // コマンドファイル読み込み
  loadCommandFile();
  
  Serial.println("[READY] AT Command execution started.");
  
  // 最初のコマンド送信
  delay(1000);  // 1秒待機してから開始
  sendNextCommand();
}

void loop() {
  // GPIO20の状態管理
  if (gpio20IsLow && (millis() - gpio20LowTime >= GPIO20_LOW_DURATION_MS)) {
    // 1秒経過したらGPIO20をHIGHに戻す
    digitalWrite(20, HIGH);
    gpio20IsLow = false;
    Serial.println("[GPIO20] Set to HIGH (timeout recovery)");
  }
  
  // コマンドレスポンスタイムアウト監視
  if (waitingForCommandResponse && (millis() - commandSentTime >= COMMAND_RESPONSE_TIMEOUT_MS)) {
    // 10秒以上レスポンスがない場合
    Serial.println("[TIMEOUT] No response for 10 seconds - Moving to next command");
    waitingForCommandResponse = false;
    
    // 次のコマンドを送信
    delay(1000);
    sendNextCommand();
  }

  // UART1_RX のレスポンス行をPCとLCDに表示
  String resp;
  if (uart1GetLine(resp)) {
    Serial.print("[RX<-UART1] ");
    Serial.println(resp);
    responseLineCount++;
    
    // SOCKETDATA RECEIVEレスポンスの文字列を抽出
    if (resp.indexOf("%SOCKETDATA:1,13,0,") >= 0) {
      int startPos = resp.indexOf(",0,") + 3;
      if (startPos > 2 && startPos < resp.length()) {
        String tempReceive = resp.substring(startPos);
        // ""で囲まれた部分を抽出
        int firstQuote = tempReceive.indexOf('"');
        int lastQuote = tempReceive.lastIndexOf('"');
        if (firstQuote >= 0 && lastQuote > firstQuote) {
          receiveDataString = tempReceive.substring(firstQuote + 1, lastQuote);
          // 先頭26文字に制限
          if (receiveDataString.length() > 26) {
            receiveDataString = receiveDataString.substring(0, 26);
          }
          hasReceiveData = true;
          Serial.printf("[INFO] RECEIVE data captured (26 chars): %s\n", receiveDataString.c_str());
        }
      }
    }
    
    // RSRP情報を検出
    if (resp.indexOf("RSRP: Reported") >= 0) {
      int rsrpStart = resp.indexOf("RSRP: Reported");
      int commaPos = resp.indexOf(',', rsrpStart);
      if (commaPos > rsrpStart) {
        rsrpInfo = resp.substring(rsrpStart, commaPos);
      } else {
        rsrpInfo = resp.substring(rsrpStart);
      }
      // "Reported "を削除してRSRPの数値のみにする
      rsrpInfo.replace("Reported ", "");
      // 手動で"dBm"を追加
      rsrpInfo += " dBm";
      hasRsrpInfo = true;
      Serial.printf("[INFO] RSRP info captured: %s\n", rsrpInfo.c_str());
    }
    
    // LCDにレスポンス表示
    if (resp.length() > 0) {
      String displayResp = resp;
      if (displayResp.length() > 20) {  // 長すぎる場合は短縮
        displayResp = displayResp.substring(0, 17) + "...";
      }
      lcdPrintLine("RSP: " + displayResp);
    }
    
    // "OK"または"ERROR"が来たらコマンド完了と判断
    if (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0) {
      waitingForCommandResponse = false;
      
      // 2秒待機してから次のコマンドを送信
      delay(2000);
      sendNextCommand();
    }
  }
}