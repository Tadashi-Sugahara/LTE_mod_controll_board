
/*
 * ESP32-C6: LittleFSのcommands1.txtからATコマンドを順次読み込み、UART1経由で送信
 * 30分おき自動実行、NG/Passカウント表示（3桁）、オンボードRGB LEDで結果表示
 * - MATCH → 青点灯（Pass加算）
 * - MISMATCH → 赤点灯（NG加算）※再スタートはしない（ERROR時のみ再スタート）
 * - ERROR発生時：GPIO20をLOW→1秒後HIGHに復帰、その後さらに6秒待機してから commands1.txt の先頭から再実行
 * - 完了画面：「Next run in 30 minutes...」の直前に直近RSRPを必ず表示（このサイクル未取得でも直近履歴を表示）
 * - LCDヘッダー：Runは削除、NG/Passを3桁固定幅表示（例：NG:005  Pass:123）
 */

#include <LovyanGFX.hpp>
#include <LittleFS.h>
#include <vector>

// ==== オンボードRGB LED（WS2812）制御 ====
#include <Adafruit_NeoPixel.h>
#define RGB_PIN   8
#define RGB_COUNT 1
Adafruit_NeoPixel rgb(RGB_COUNT, RGB_PIN, NEO_RGB + NEO_KHZ800);  // 実機に合わせて NEO_GRB/NEO_BRG などに変更可
inline void rgbSet(uint8_t r, uint8_t g, uint8_t b) { rgb.setPixelColor(0, rgb.Color(r,g,b)); rgb.show(); }
inline void showMatch()    { rgbSet(0,   0, 255); }   // 青
inline void showMismatch() { rgbSet(255, 0,   0); }   // 赤
inline void showIdle()     { rgbSet(0,   0,   0); }   // 消灯

/* ==================== LCD設定（ST7789/172x320） ==================== */
class LGFX_ESP32C6_WS : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;
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
static const int margin=8;
int cursorX=margin;
int cursorY=margin;
int lineHeight=0;
int textSize=1;
int responseLineCount=0;

// ==== 周期/NG/Pass 管理 ====
bool isRunning=false;
unsigned long lastRunMillis=0;
//const unsigned long RUN_INTERVAL_MS=1800000UL; // ★ 30分（= 30*60*1000）
const unsigned long RUN_INTERVAL_MS=60000UL;
uint32_t ngCountRun=0;        // このサイクルのNG
uint32_t passCountRun=0;      // このサイクルのPass（MATCH）
uint32_t passCountTotal=0;    // 通算Pass

// GPIO20 リセット（LOW→1秒でHIGH戻し）
unsigned long gpio20LowTime=0;
bool gpio20IsLow=false;
const unsigned long GPIO20_LOW_DURATION_MS=1000;

// ERROR後の再始動要求（HIGH復帰後にさらに6秒待機→先頭から再実行）
bool pendingRestartAfterError=false;

// RSRP情報（サイクル内用）＋ 直近RSRPの永続保持
String rsrpInfo="";        // このサイクルで受信した最新のRSRP
bool   hasRsrpInfo=false;
String lastRsrpInfo="";    // 直近サイクルも含めて最終のRSRP履歴
bool   hasLastRsrp=false;

// ヘッダー描画（Runは削除、NG/Passのみ3桁表示）
void drawHeaderWithStatus() {
  lcd.setCursor(margin, margin);
  lcd.println("AT Command Executor");
  cursorY = margin + lineHeight;
  cursorX = margin;

  lcd.setCursor(cursorX, cursorY);
  lcd.printf("NG:%03lu  Pass:%03lu",
             (unsigned long)ngCountRun,
             (unsigned long)passCountRun);
  cursorY += lineHeight;

  lcd.drawLine(margin, cursorY, lcd.width()-margin, cursorY, TFT_DARKGREY);
  cursorY += (lineHeight/4);
}
void lcdClearBeforeNewLine(bool drawHeader=true){
  lcd.fillScreen(TFT_BLACK);
  cursorX=margin; cursorY=margin;
  if(drawHeader) drawHeaderWithStatus();
}
void lcdNewLine(){
  cursorX=margin; cursorY+=lineHeight;
  if(cursorY > lcd.height()-margin-lineHeight) lcdClearBeforeNewLine();
}
void lcdPrintLine(const String& line){
  lcd.setCursor(cursorX,cursorY); lcd.print(line); lcdNewLine();
}

/* ==================== ATコマンド実行 ==================== */
static int currentLineIndex=0;
static std::vector<String> commandLines;
bool commandsLoaded=false;
bool waitingForCommandResponse=false;
unsigned long commandSentTime=0;
const unsigned long COMMAND_RESPONSE_TIMEOUT_MS=10000;

// 送受一致比較用
String sendDataString="";
String receiveDataString="";
bool hasSendData=false;
bool hasReceiveData=false;

// コマンドファイル読み込み
void loadCommandFile(){
  if(!LittleFS.begin(true)){ Serial.println("[ERROR] Failed to mount LittleFS"); return; }
  File file=LittleFS.open("/commands1.txt","r"); if(!file){ Serial.println("[ERROR] Failed to open commands1.txt"); return; }
  commandLines.clear();
  while(file.available()){
    String line=file.readStringUntil('\n');
    line.trim();
    if(line.length()>0) commandLines.push_back(line);
  }
  file.close();
  Serial.printf("[INFO] Loaded %d lines from commands1.txt\n", commandLines.size());
  commandsLoaded=true; currentLineIndex=0;
}

// 実行サイクル開始
void startRun(){
  ngCountRun=0; passCountRun=0;

  // サイクル内の一時変数はリセットするが、直近RSRP履歴(lastRsrpInfo)は消さない
  hasSendData=false; hasReceiveData=false;
  hasRsrpInfo=false; rsrpInfo="";

  waitingForCommandResponse=false; currentLineIndex=0;
  showIdle(); // RGB消灯

  lcdClearBeforeNewLine();
  lcdPrintLine("Loading commands...");
  loadCommandFile(); isRunning=true;

  delay(1000);
  sendNextCommand();
}

// 先頭から再スタート（ERROR復帰後の6秒待機後に呼ぶ）
void restartFromTop() {
  Serial.println("[RESTART] Restarting from line 1");
  isRunning = true;
  waitingForCommandResponse = false;

  // サイクル内の一時変数のみリセット（直近RSRP履歴は維持）
  hasSendData=false; hasReceiveData=false;
  hasRsrpInfo=false; rsrpInfo="";
  sendDataString=""; receiveDataString="";

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
    // 完了画面は余裕のある表示順に固定：タイトル → 比較 → RSRP → Next run
    lcdClearBeforeNewLine();
    lcdPrintLine("All commands completed!");

    // 比較結果 & LED & カウント（MISMATCHでは再スタートしない）
    if(hasSendData && hasReceiveData){
      if(sendDataString.equals(receiveDataString)){
        lcdPrintLine("String MATCH!");
        Serial.println("[INFO] SEND/RECEIVE strings MATCH! (26 chars)");
        showMatch();     // 青
        passCountRun++;  // 今サイクルのPass
        passCountTotal++;// 通算Pass
      }else{
        lcdPrintLine("String MISMATCH");
        Serial.printf("[INFO] SEND/RECEIVE strings MISMATCH (26 chars) - SEND: %s, RECEIVE: %s\n",
                      sendDataString.c_str(), receiveDataString.c_str());
        ngCountRun++;    // NG
        showMismatch();  // 赤
        // MISMATCHでは再スタートしない（仕様）
      }
    }else{
      lcdPrintLine("No data to compare");
      showIdle();
    }

    // ==== 直近RSRPを必ず表示（このサイクル未取得なら履歴を使う） ====
    if (hasRsrpInfo && rsrpInfo.length() > 0) {
      lcdPrintLine(rsrpInfo);           // 例: "RSRP: -98 dBm"
      // 履歴にも反映（最新を保持）
      lastRsrpInfo = rsrpInfo; hasLastRsrp = true;
    } else if (hasLastRsrp && lastRsrpInfo.length() > 0) {
      lcdPrintLine(lastRsrpInfo);       // 直近履歴を表示
    } else {
      lcdPrintLine("RSRP: N/A");         // 一度も取得なし
    }

    // 次回までの待機案内（30分）
    isRunning=false; lastRunMillis=millis();
    lcdPrintLine("Next run in 1 minutes...");
    //lcdPrintLine("Next run in 30 minutes...");
    Serial.println("[INFO] Waiting for next 30-minute run");
    return;
  }

  String command = commandLines[currentLineIndex];

  // コメント行スキップ
  if(command.startsWith("#")){
    Serial.printf("[SKIP] Comment line: %s\n", command.c_str());
    currentLineIndex++;
    sendNextCommand();
    return;
  }

  // SENDの文字列抽出
  if(command.indexOf("AT%SOCKETDATA=\"SEND\",1,13,") >= 0){
    int startPos = command.indexOf(",13,") + 4;
    if(startPos > 3 && startPos < command.length()){
      sendDataString = command.substring(startPos);
      sendDataString.replace("\"","");
      if(sendDataString.length()>26) sendDataString = sendDataString.substring(0,26);
      hasSendData=true;
      Serial.printf("[INFO] SEND data captured (26 chars): %s\n", sendDataString.c_str());
    }
  }

  // 表示＆送信
  lcdClearBeforeNewLine();
  lcdPrintLine("CMD: " + command);

  Serial.printf("[TX->UART1] %s\n", command.c_str());
  Serial1.print(command);
  Serial1.print("\r\n");

  waitingForCommandResponse=true;
  commandSentTime=millis();
  responseLineCount=0;
  currentLineIndex++;
}

/* ==================== UART設定 ==================== */
static const uint32_t BAUD_PC=115200, BAUD_UART=115200;
static const int UART1_TX_PIN=18, UART1_RX_PIN=19;

/* ==================== UART受信（LFまたはギャップで行確定） ==================== */
static std::vector<uint8_t> uart1Buf;
static uint32_t uart1LastByteMs=0;
static const uint32_t UART1_GAP_MS=50;

bool uart1GetLine(String &out){
  bool got=false;
  while(Serial1.available()){
    uint8_t b=(uint8_t)Serial1.read();
    uart1LastByteMs=millis();
    if(b=='\r') continue;
    if(b=='\n'){
      if(!uart1Buf.empty()){
        out.reserve(uart1Buf.size());
        for(auto v:uart1Buf) out += (char)v;
        uart1Buf.clear();
        return true;
      }else continue;
    }
    uart1Buf.push_back(b);
    if(uart1Buf.size()>4096){
      out.reserve(uart1Buf.size());
      for(auto v:uart1Buf) out += (char)v;
      uart1Buf.clear();
      return true;
    }
  }
  if(!uart1Buf.empty()){
    uint32_t now=millis();
    if(now - uart1LastByteMs >= UART1_GAP_MS){
      out.reserve(uart1Buf.size());
      for(auto v:uart1Buf) out += (char)v;
      uart1Buf.clear();
      got=true;
    }
  }
  return got;
}

/* ==================== setup/loop ==================== */
void setup(){
  // PCシリアル
  Serial.begin(BAUD_PC);
  delay(300);

  // GPIO20 初期化（HIGH）
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);
  Serial.println("[GPIO20] Set to HIGH");

  // UART1
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
  lcdPrintLine("READY - 30min executor");

  // RGB LED 初期化
  rgb.begin();
  rgb.show();
  showIdle();

  lastRunMillis = millis();
  startRun();
}

void loop(){
  // ==== 1秒経過でGPIO20を自動復帰（HIGH） ====
  if(gpio20IsLow && (millis()-gpio20LowTime >= GPIO20_LOW_DURATION_MS)){
    digitalWrite(20, HIGH);
    gpio20IsLow=false;
    Serial.println("[GPIO20] Set to HIGH (timeout recovery)");

    // ERROR時の再実行：HIGH復帰後にさらに6秒待って先頭再スタート
    if (pendingRestartAfterError) {
      pendingRestartAfterError = false;
      lcdPrintLine("Wait 6s before restart...");
      delay(6000);            // 6秒待機
      restartFromTop();       // 先頭から再実行
      return;
    }
  }

  // 周期実行トリガ（30分おき）
  if(!isRunning && (millis()-lastRunMillis >= RUN_INTERVAL_MS)){
    startRun();
  }

  // レスポンスタイムアウト監視（10秒）
  if(isRunning && waitingForCommandResponse &&
     (millis()-commandSentTime >= COMMAND_RESPONSE_TIMEOUT_MS)){
    Serial.println("[TIMEOUT] No response for 10 seconds - Moving to next command");
    waitingForCommandResponse=false;
    delay(1000);
    sendNextCommand();
  }

  // UART受信・解析
  String resp;
  if(uart1GetLine(resp)){
    Serial.print("[RX<-UART1] ");
    Serial.println(resp);
    responseLineCount++;

    // %SOCKETDATAの受信データ抽出
    if(resp.indexOf("%SOCKETDATA:1,13,0,") >= 0){
      int startPos = resp.indexOf(",0,") + 3;
      if(startPos > 2 && startPos < resp.length()){
        String tmp = resp.substring(startPos);
        int q1 = tmp.indexOf('"');
        int q2 = tmp.lastIndexOf('"');
        if(q1>=0 && q2>q1){
          receiveDataString = tmp.substring(q1+1, q2);
          if(receiveDataString.length()>26) receiveDataString = receiveDataString.substring(0,26);
          hasReceiveData = true;
          Serial.printf("[INFO] RECEIVE data captured (26 chars): %s\n", receiveDataString.c_str());
        }
      }
    }

    // RSRP（サイクル内最新＋直近履歴の更新）
    if(resp.indexOf("RSRP: Reported") >= 0){
      int s = resp.indexOf("RSRP: Reported");
      int c = resp.indexOf(',', s);
      rsrpInfo = (c > s) ? resp.substring(s, c) : resp.substring(s);
      rsrpInfo.replace("Reported ", "");
      rsrpInfo += " dBm";
      hasRsrpInfo = true;

      // 直近履歴にも反映
      lastRsrpInfo = rsrpInfo;
      hasLastRsrp  = true;

      Serial.printf("[INFO] RSRP info captured: %s\n", rsrpInfo.c_str());
    }

    // LCDにレスポンス表示（長文は省略）
    if(resp.length()>0){
      String d = resp;
      if(d.length()>20) d = d.substring(0,17) + "...";
      lcdPrintLine("RSP: " + d);
    }

    // ERROR検出 → NG加算／LED赤／GPIO20 LOW → 1秒後HIGH → さらに6秒待機 → 先頭から再実行
    if(resp.indexOf("ERROR") >= 0){
      ngCountRun++;
      showMismatch();

      // リセット開始（LOW化）
      if (!gpio20IsLow) {
        digitalWrite(20, LOW);
        gpio20IsLow   = true;
        gpio20LowTime = millis();   // 1秒後に HIGH 復帰（上の復帰処理で）
        Serial.println("[GPIO20] LOW (ERROR reset sequence started)");
      }

      pendingRestartAfterError = true;  // 復帰後に再スタート

      // LCDログ
      lcdClearBeforeNewLine();
      lcdPrintLine("RSP: ERROR detected");
      lcdPrintLine("Reset GPIO20 -> wait 6s");
    }

    // 完了判定（OK/ERROR）
    if(resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0){
      waitingForCommandResponse=false;
      delay(2000);
      sendNextCommand();
    }
  }
}
