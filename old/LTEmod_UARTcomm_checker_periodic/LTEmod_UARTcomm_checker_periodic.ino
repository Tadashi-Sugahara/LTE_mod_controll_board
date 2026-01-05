
/*
 * ESP32-C6: LittleFSのcommands1.txtからATコマンドを順次読み込み、UART1経由で送信
 * 1時間おき自動実行、NG/Passカウント表示、オンボードRGB LEDで結果表示
 * - MATCH → 青点灯（Pass加算）
 * - MISMATCH/ERROR → 赤点灯（NG加算）
 * - NG発生時：GPIO20をLOW→1秒後HIGHに復帰、その直後に commands1.txt の先頭から再実行
 */

#include <LovyanGFX.hpp>
#include <LittleFS.h>
#include <vector>

// ==== オンボードRGB LED（WS2812）制御 ====
#include <Adafruit_NeoPixel.h>
#define RGB_PIN   8
#define RGB_COUNT 1
Adafruit_NeoPixel rgb(RGB_COUNT, RGB_PIN, NEO_RGB + NEO_KHZ800);  // 実機に合わせて変更
inline void rgbSet(uint8_t r, uint8_t g, uint8_t b) { rgb.setPixelColor(0, rgb.Color(r,g,b)); rgb.show(); }
inline void showMatch()    { rgbSet(0,   0, 255); }   // 青
inline void showMismatch() { rgbSet(255, 0,   0); }   // 赤
inline void showIdle()     { rgbSet(0,   0,   0); }   // 消灯

/* ==================== LCD設定 ==================== */
class LGFX_ESP32C6_WS : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel; lgfx::Bus_SPI _bus; lgfx::Light_PWM _light;
public:
  LGFX_ESP32C6_WS(void) {
    { auto cfg=_bus.config(); cfg.spi_host=SPI2_HOST; cfg.spi_mode=0; cfg.freq_write=20000000; cfg.freq_read=10000000;
      cfg.spi_3wire=false; cfg.use_lock=true; cfg.dma_channel=SPI_DMA_CH_AUTO; cfg.pin_sclk=7; cfg.pin_mosi=6; cfg.pin_miso=-1; cfg.pin_dc=15;
      _bus.config(cfg); _panel.setBus(&_bus); }
    { auto cfg=_panel.config(); cfg.pin_cs=14; cfg.pin_rst=21; cfg.pin_busy=-1; cfg.memory_width=172; cfg.memory_height=320;
      cfg.panel_width=172; cfg.panel_height=320; cfg.offset_x=0; cfg.offset_y=0; cfg.invert=false; cfg.rgb_order=false; cfg.readable=false; cfg.bus_shared=false;
      _panel.config(cfg); }
    { auto cfg=_light.config(); cfg.pin_bl=22; cfg.freq=5000; cfg.pwm_channel=0; _light.config(cfg); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};
LGFX_ESP32C6_WS lcd;

/* ==================== 表示ロジック ==================== */
static const int margin=8; int cursorX=margin; int cursorY=margin; int lineHeight=0; int textSize=1; int responseLineCount=0;

// ==== 周期/NG/Pass/実行回数管理 ====
bool isRunning=false; unsigned long lastRunMillis=0; const unsigned long RUN_INTERVAL_MS=3600000UL;
uint32_t ngCountRun=0;        // このサイクルのNG
uint32_t passCountRun=0;      // このサイクルのPass（MATCH）
uint32_t passCountTotal=0;    // 通算Pass
uint32_t runCounter=0;        // 実行回数

// GPIO20タイムアウト（既存）
unsigned long gpio20LowTime=0; bool gpio20IsLow=false; const unsigned long GPIO20_LOW_DURATION_MS=1000;

// ==== 追加: NG後の再スタート要求フラグ ====
bool pendingRestart = false;

// ==== 追加: GPIO20 リセットシーケンスのトリガ ====
void triggerGpio20Reset() {
  if (!gpio20IsLow) {                 // 二重起動防止
    digitalWrite(20, LOW);            // 即LOW
    gpio20IsLow   = true;
    gpio20LowTime = millis();         // 復帰タイマー開始（loopで1秒後にHIGH）
    Serial.println("[GPIO20] LOW (NG reset sequence started)");
  } else {
    Serial.println("[GPIO20] Already LOW (reset in progress)");
  }
}

// ヘッダー描画（NG/Pass/Run）
void drawHeaderWithStatus() {
  lcd.setCursor(margin, margin); lcd.println("AT Command Executor");
  cursorY = margin + lineHeight; cursorX = margin;
  lcd.setCursor(cursorX, cursorY);
  lcd.printf("NG:%lu  Pass:%lu  Run:%lu", (unsigned long)ngCountRun, (unsigned long)passCountRun, (unsigned long)runCounter);
  cursorY += lineHeight;
  lcd.drawLine(margin, cursorY, lcd.width()-margin, cursorY, TFT_DARKGREY);
  cursorY += (lineHeight/4);
}
void lcdClearBeforeNewLine(bool drawHeader=true){ lcd.fillScreen(TFT_BLACK); cursorX=margin; cursorY=margin; if(drawHeader) drawHeaderWithStatus(); }
void lcdNewLine(){ cursorX=margin; cursorY+=lineHeight; if(cursorY>lcd.height()-margin-lineHeight){ lcdClearBeforeNewLine(); } }
void lcdPrintLine(const String& line){ lcd.setCursor(cursorX,cursorY); lcd.print(line); lcdNewLine(); }

/* ==================== ATコマンド実行 ==================== */
static int currentLineIndex=0; static std::vector<String> commandLines;
bool commandsLoaded=false; bool waitingForCommandResponse=false; unsigned long commandSentTime=0;
const unsigned long COMMAND_RESPONSE_TIMEOUT_MS=10000;

// 送受一致比較用
String sendDataString=""; String receiveDataString=""; bool hasSendData=false; bool hasReceiveData=false;

// RSRP情報
String rsrpInfo=""; bool hasRsrpInfo=false;

// コマンドファイル読み込み
void loadCommandFile(){
  if(!LittleFS.begin(true)){ Serial.println("[ERROR] Failed to mount LittleFS"); return; }
  File file=LittleFS.open("/commands2.txt","r"); if(!file){ Serial.println("[ERROR] Failed to open commands1.txt"); return; }
  commandLines.clear();
  while(file.available()){ String line=file.readStringUntil('\n'); line.trim(); if(line.length()>0) commandLines.push_back(line); }
  file.close();
  Serial.printf("[INFO] Loaded %d lines from commands1.txt\n", commandLines.size());
  commandsLoaded=true; currentLineIndex=0;
}

// 実行サイクル開始
void startRun(){
  ngCountRun=0; passCountRun=0;
  hasSendData=false; hasReceiveData=false; hasRsrpInfo=false;
  waitingForCommandResponse=false; currentLineIndex=0; runCounter++;
  showIdle(); // RGB消灯
  lcdClearBeforeNewLine(); lcdPrintLine("Loading commands...");
  loadCommandFile(); isRunning=true;
  delay(1000); sendNextCommand();
}

// ==== 追加: commands1.txt の先頭から再スタート ====
void restartFromTop() {
  Serial.println("[RESTART] Restarting from line 1 after NG reset");

  isRunning = true;
  waitingForCommandResponse = false;

  hasSendData    = false;
  hasReceiveData = false;
  hasRsrpInfo    = false;
  sendDataString = "";
  receiveDataString = "";
  rsrpInfo = "";

  currentLineIndex = 0;

  lcdClearBeforeNewLine(true);
  lcdPrintLine("Restarting from line 1...");
  loadCommandFile();
  delay(500);
  sendNextCommand();
}

// 次のATコマンド送信
void sendNextCommand(){
  if(!commandsLoaded || currentLineIndex >= (int)commandLines.size()){
    Serial.println("[INFO] All commands completed");
    lcdClearBeforeNewLine(); lcdPrintLine("All commands completed!");

    // 比較結果 & RGB LED & カウント
    if(hasSendData && hasReceiveData){
      if(sendDataString.equals(receiveDataString)){
        lcdPrintLine("String MATCH!");
        Serial.println("[INFO] SEND/RECEIVE strings MATCH! (26 chars)");
        showMatch();       // 青
        passCountRun++;    // 今サイクルのPass
        passCountTotal++;  // 通算Pass
      }else{
        lcdPrintLine("String MISMATCH");
        Serial.printf("[INFO] SEND/RECEIVE strings MISMATCH (26 chars) - SEND: %s, RECEIVE: %s\n",
                      sendDataString.c_str(), receiveDataString.c_str());
        ngCountRun++;      // NG
        showMismatch();    // 赤
        triggerGpio20Reset();
        pendingRestart = true;   // 復帰後に先頭から再スタート
      }
    }else{
      lcdPrintLine("No data to compare");
      showIdle();
    }

    if(hasRsrpInfo){ lcdPrintLine(rsrpInfo); }

    // 通常の1時間待機は、NG再スタートがなければ適用
    isRunning=false; lastRunMillis=millis();
    lcdPrintLine("Next run in 60 minutes...");
    Serial.println("[INFO] Waiting for next hourly run");
    return;
  }

  String command = commandLines[currentLineIndex];

  // コメント行スキップ
  if(command.startsWith("#")){
    Serial.printf("[SKIP] Comment line: %s\n", command.c_str());
    currentLineIndex++; sendNextCommand(); return;
  }

  // SENDの文字列抽出
  if(command.indexOf("AT%SOCKETDATA=\"SEND\",1,13,") >= 0){
    int startPos = command.indexOf(",13,") + 4;
    if(startPos > 3 && startPos < command.length()){
      sendDataString = command.substring(startPos);
      sendDataString.replace("\"",""); if(sendDataString.length()>26) sendDataString = sendDataString.substring(0,26);
      hasSendData=true; Serial.printf("[INFO] SEND data captured (26 chars): %s\n", sendDataString.c_str());
    }
  }

  // 表示＆送信
  lcdClearBeforeNewLine(); lcdPrintLine("CMD: " + command);
  Serial.printf("[TX->UART1] %s\n", command.c_str());
  Serial1.print(command); Serial1.print("\r\n");

  waitingForCommandResponse=true; commandSentTime=millis(); responseLineCount=0; currentLineIndex++;
}

/* ==================== UART設定 ==================== */
static const uint32_t BAUD_PC=115200, BAUD_UART=115200; static const int UART1_TX_PIN=18, UART1_RX_PIN=19;

/* ==================== UART受信（行確定） ==================== */
static std::vector<uint8_t> uart1Buf; static uint32_t uart1LastByteMs=0; static const uint32_t UART1_GAP_MS=50;

bool uart1GetLine(String &out){
  bool got=false;
  while(Serial1.available()){
    uint8_t b=(uint8_t)Serial1.read(); uart1LastByteMs=millis();
    if(b=='\r') continue;
    if(b=='\n'){
      if(!uart1Buf.empty()){
        out.reserve(uart1Buf.size()); for(auto v:uart1Buf) out += (char)v; uart1Buf.clear(); return true;
      }else continue;
    }
    uart1Buf.push_back(b);
    if(uart1Buf.size()>4096){
      out.reserve(uart1Buf.size()); for(auto v:uart1Buf) out += (char)v; uart1Buf.clear(); return true;
    }
  }
  if(!uart1Buf.empty()){
    uint32_t now=millis();
    if(now - uart1LastByteMs >= UART1_GAP_MS){
      out.reserve(uart1Buf.size()); for(auto v:uart1Buf) out += (char)v; uart1Buf.clear(); got=true;
    }
  }
  return got;
}

/* ==================== setup/loop ==================== */
void setup(){
  Serial.begin(BAUD_PC); delay(300);
  pinMode(20, OUTPUT); digitalWrite(20, HIGH); Serial.println("[GPIO20] Set to HIGH");
  Serial1.begin(BAUD_UART, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN); Serial1.setRxBufferSize(2048);
  delay(6000);
  lcd.begin(); lcd.setRotation(1); lcd.setBrightness(220); lcd.setFont(&fonts::Font4);
  lcd.setTextSize(1); lcd.setTextColor(TFT_WHITE, TFT_BLACK); lcd.setTextDatum(textdatum_t::top_left);
  lineHeight = lcd.fontHeight() * 1;
  lcdClearBeforeNewLine(); lcdPrintLine("READY - Hourly executor");
  rgb.begin(); rgb.show(); showIdle();
  lastRunMillis = millis();
  startRun();
}

void loop(){
  // ==== 1秒経過でGPIO20を自動復帰（HIGH） ====
  if(gpio20IsLow && (millis()-gpio20LowTime >= GPIO20_LOW_DURATION_MS)){
    digitalWrite(20, HIGH);
    gpio20IsLow=false;
    Serial.println("[GPIO20] Set to HIGH (timeout recovery)");

    // 復帰直後に先頭から再スタート（要求がある場合のみ）
    if (pendingRestart) {
      pendingRestart = false;
      restartFromTop();
      return;
    }
  }

  // 周期実行トリガ（1時間おき）
  if(!isRunning && (millis()-lastRunMillis >= RUN_INTERVAL_MS)){ startRun(); }

  // レスポンスタイムアウト監視
  if(isRunning && waitingForCommandResponse &&
     (millis()-commandSentTime >= COMMAND_RESPONSE_TIMEOUT_MS)){
    Serial.println("[TIMEOUT] No response for 10 seconds - Moving to next command");
    waitingForCommandResponse=false; delay(1000); sendNextCommand();
  }

  String resp;
  if(uart1GetLine(resp)){
    Serial.print("[RX<-UART1] "); Serial.println(resp); responseLineCount++;

    // 受信データ抽出
    if(resp.indexOf("%SOCKETDATA:1,13,0,") >= 0){
      int startPos = resp.indexOf(",0,") + 3;
      if(startPos > 2 && startPos < resp.length()){
        String tmp = resp.substring(startPos);
        int q1 = tmp.indexOf('"'); int q2 = tmp.lastIndexOf('"');
        if(q1>=0 && q2>q1){
          receiveDataString = tmp.substring(q1+1, q2);
          if(receiveDataString.length()>26) receiveDataString = receiveDataString.substring(0,26);
          hasReceiveData = true;
          Serial.printf("[INFO] RECEIVE data captured (26 chars): %s\n", receiveDataString.c_str());
        }
      }
    }

    // RSRP
    if(resp.indexOf("RSRP: Reported") >= 0){
      int s = resp.indexOf("RSRP: Reported"); int c = resp.indexOf(',', s);
      rsrpInfo = (c > s) ? resp.substring(s, c) : resp.substring(s);
      rsrpInfo.replace("Reported ", ""); rsrpInfo += " dBm"; hasRsrpInfo = true;
      Serial.printf("[INFO] RSRP info captured: %s\n", rsrpInfo.c_str());
    }

    // 表示
    if(resp.length()>0){
      String d = resp; if(d.length()>20) d = d.substring(0,17) + "...";
      lcdPrintLine("RSP: " + d);
    }

    // ERROR検出 → NG加算＆LED赤＆GPIO20リセット → 復帰後に先頭から再実行
    if(resp.indexOf("ERROR") >= 0){
      ngCountRun++; showMismatch();
      triggerGpio20Reset();
      pendingRestart = true;         // ★ 復帰後に再スタート
      lcdClearBeforeNewLine(); lcdPrintLine("RSP: ERROR detected");
    }

    // 完了判定
    if(resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0){
      waitingForCommandResponse=false; delay(2000); sendNextCommand();
    }
  }
}
