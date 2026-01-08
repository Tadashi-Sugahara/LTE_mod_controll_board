
/*
 * ESP32-C6: LittleFS の /commands1.txt から AT コマンドを順次実行（UART1）
 * - LCD（LovyanGFX/ST7789）へ CMD/RSP 表示、WS2812で結果表示
 * - ログ: /logs/atlog_YYYYMMDD_HHMMSS.txt（+CCLK時刻／なければ millis）
 * - PC→ESP32: #LIST / #GET <path> / #DELLOG（FILEBEGIN size + 本体 + FILEEND）
 * - 既定タイムアウト 30 秒（COMMAND_RESPONSE_TIMEOUT_MS）
 * - AT+COPS=1,2,"44020" 特例:
 *   - 各試行で最大10秒待機（タイムアウトでリトライ、最大10回）
 *   - 16進数トークン（<XX>）受信を検知（ログのみ／成功はOKで判定）
 * - すべてのコマンドで ERROR 受信時は最大10回までリトライ（共通）
 * - 起動直後に1回開始、その後は 1 分おき（60,000ms）に自動実行
 * - ★ インターバル中は UART1 を停止（Serial1.end）→受信完全停止
 * - ★ 非ASCII（バイナリ）をHEX化して表示・ログ（makePrintable）
 * - 転送中は PC向けログ出力を抑止（pcTransferActive）
 * - GP23（押すとLow）を GPIO割り込みで「処理の一時中断⇔再開」をトグル（30msデバウンス＋5秒ロックアウト）
 */

#include <Arduino.h>
#include <FS.h>         // fs::File
#include <LittleFS.h>   // LittleFS
#include <LovyanGFX.hpp>
#include <Adafruit_NeoPixel.h>
#include <vector>

#if defined(ESP32)
  // 競合回避用クリティカルセクション（ISRフラグの取り扱い）
  #include "freertos/FreeRTOS.h"
  #include "freertos/portmacro.h"
  portMUX_TYPE gp23Mux = portMUX_INITIALIZER_UNLOCKED;
  #ifndef IRAM_ATTR
    #define IRAM_ATTR
  #endif
#endif

/* ==================== グローバル設定・ユーティリティ ==================== */
bool pcTransferActive = false;
#define PC_PRINT(s)    do{ if(!pcTransferActive) Serial.print(s); } while(0)
#define PC_PRINTLN(s)  do{ if(!pcTransferActive) Serial.println(s); } while(0)
#define PC_PRINTF(...) do{ if(!pcTransferActive) Serial.printf(__VA_ARGS__); } while(0)

#define RGB_PIN   8
#define RGB_COUNT 1
Adafruit_NeoPixel rgb(RGB_COUNT, RGB_PIN, NEO_RGB + NEO_KHZ800);

inline void rgbSet(uint8_t r, uint8_t g, uint8_t b){ rgb.setPixelColor(0, rgb.Color(r,g,b)); rgb.show(); }
inline void showMatch()    { rgbSet(0, 0, 255); }
inline void showMismatch() { rgbSet(255, 0, 0); }
inline void showIdle()     { rgbSet(0, 0, 0); }

class LGFX_ESP32C6_WS : public lgfx::LGFX_Device{
  lgfx::Panel_ST7789 _panel; lgfx::Bus_SPI _bus; lgfx::Light_PWM _light;
public:
  LGFX_ESP32C6_WS(void){
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

static const int margin=8;
int cursorX=margin, cursorY=margin;
int lineHeight=0, textSize=1;
int responseLineCount=0;
bool isRunning=false;
bool idleHoldDisplay=false;

unsigned long lastRunMillis=0;
const unsigned long COMMAND_RESPONSE_TIMEOUT_MS = 30000;   // 既定 30s
const unsigned long RUN_INTERVAL_MS              = 10000UL; // 10sec
uint32_t ngCountRun=0, passCountRun=0;

unsigned long gpio20LowTime=0; bool gpio20IsLow=false; const unsigned long GPIO20_LOW_DURATION_MS=1000;
bool pendingRestartAfterError=false;

String rsrpInfo=""; bool hasRsrpInfo=false; String lastRsrpInfo=""; bool hasLastRsrp=false;

/* ==================== GP23: プッシュスイッチ（押すとLow）・割り込み ==================== */
#define GP23_SWITCH_PIN 23
bool userPauseEnabled = false;                    // 一時中断フラグ

// ISR→メインループへ伝えるイベントフラグ（最小の仕事のみ）
volatile bool gp23IsrEvent = false;

// デバウンス＋ロックアウトの制御（メインループ側で処理）
static bool           swDebouncing        = false;
static unsigned long  swDebounceStartMs   = 0;
static unsigned long  lastSwitchHandledMs = 0;

const unsigned long   SWITCH_DEBOUNCE_MS  = 30;     // 30ms デバウンス
const unsigned long   SWITCH_LOCK_MS      = 5000;   // 5秒ロックアウト

// ISR（FALLING）: 極力短く・IRAMに配置
void IRAM_ATTR gp23_isr(){
  #if defined(ESP32)
    portENTER_CRITICAL_ISR(&gp23Mux);
  #endif
  gp23IsrEvent = true;         // ループ側で処理
  #if defined(ESP32)
    portEXIT_CRITICAL_ISR(&gp23Mux);
  #endif
}

/* ==================== ログ（LittleFS） ==================== */
String logFilePath=""; fs::File logFile; bool logReady=false;
bool hasClockFromModem=false; String currentClockStr="";

bool ensureFS(){ if(!LittleFS.begin(true)){ PC_PRINTLN("[ERROR] Failed to mount LittleFS"); return false; } return true; }
static String pad2(int v){ String s=String(v); if(s.length()<2) s="0"+s; return s; }

String cclkToFilenameStamp(const String &cclk){
  int q1=cclk.indexOf('"'), q2=cclk.lastIndexOf('"');
  String core=(q1>=0&&q2>q1)? cclk.substring(q1+1,q2):cclk;
  int s1=core.indexOf('/'), s2=core.indexOf('/',s1+1), comma=core.indexOf(','), c1=core.indexOf(':',comma+1), c2=core.indexOf(':',c1+1);
  if(s1<0 || s2<0 || comma<0 || c1<0 || c2<0) return String(millis()/1000)+"s";
  String yy=core.substring(0,s1), MM=core.substring(s1+1,s2), dd=core.substring(s2+1,comma);
  String hh=core.substring(comma+1,c1), mm=core.substring(c1+1,c2), ss=core.substring(c2+1,c2+3);
  return String("20")+yy+pad2(MM.toInt())+pad2(dd.toInt())+"_"+pad2(hh.toInt())+pad2(mm.toInt())+pad2(ss.toInt());
}
String makeLogFilePath(){ String stamp=(hasClockFromModem && currentClockStr.length()>0)? cclkToFilenameStamp(currentClockStr) : String(millis()/1000)+"s"; return String("/logs/atlog_")+stamp+String(".txt"); }
void openNewLog(){
  if(!ensureFS()){ logReady=false; return; } LittleFS.mkdir("/logs"); logFilePath=makeLogFilePath();
  logFile=LittleFS.open(logFilePath,"a");
  if(!logFile){ PC_PRINTF("[ERROR] Failed to open log file: %s\n", logFilePath.c_str()); logReady=false; return; }
  logReady=true; PC_PRINTF("[LOG] Opened: %s\n", logFilePath.c_str()); logFile.printf("=== AT Log start ===\nfile: %s\n", logFilePath.c_str()); logFile.flush();
}
void closeLog(){ if(logReady){ logFile.printf("=== AT Log end ===\n"); logFile.flush(); logFile.close(); logReady=false; } }
String nowStamp(){ if(hasClockFromModem && currentClockStr.length()>0){ int q1=currentClockStr.indexOf('"'), q2=currentClockStr.lastIndexOf('"'); String core=(q1>=0&&q2>q1)? currentClockStr.substring(q1+1,q2):currentClockStr; return String("[")+String("20")+core+String("]"); } return String("[")+String(millis()/1000)+String("s]"); }
void logWrite(const String &line){ if(!logReady) return; logFile.printf("%s %s\n", nowStamp().c_str(), line.c_str()); logFile.flush(); }

/* ==================== 表示ユーティリティ ==================== */
void drawHeaderWithStatus(){
  lcd.setCursor(margin, margin); lcd.println("AT Command Executor");
  cursorY=margin+lineHeight; cursorX=margin;
  lcd.setCursor(cursorX, cursorY); lcd.printf("NG:%03lu Pass:%03lu",(unsigned long)ngCountRun,(unsigned long)passCountRun);
  cursorY+=lineHeight; lcd.drawLine(margin,cursorY,lcd.width()-margin,cursorY,TFT_DARKGREY); cursorY+=(lineHeight/4);
}
void lcdClearBeforeNewLine(bool drawHeader=true){ lcd.fillScreen(TFT_BLACK); cursorX=margin; cursorY=margin; if(drawHeader) drawHeaderWithStatus(); }
void lcdNewLine(){ cursorX=margin; cursorY+=lineHeight; if(cursorY>lcd.height()-margin-lineHeight) lcdClearBeforeNewLine(); }
void lcdPrintLine(const String& line){ lcd.setCursor(cursorX,cursorY); lcd.print(line); lcdNewLine(); }

/* ==================== 受信フィルタ（ASCII/HEX化） ==================== */
String makePrintable(const String& s){
  String out; out.reserve(s.length()*2);
  for(size_t i=0;i<s.length();++i){
    uint8_t b=(uint8_t)s[i];
    if(b=='\r' || b=='\n' || (b>=0x20 && b<=0x7E)) out+=(char)b;
    else { char hx[5]; snprintf(hx,sizeof(hx),"<%02X>",b); out+=hx; }
  }
  return out;
}
float asciiRatio(const String& s){
  if(s.length()==0) return 1.0f;
  size_t ascii=0;
  for(size_t i=0;i<s.length();++i){
    uint8_t b=(uint8_t)s[i];
    if(b=='\r' || b=='\n' || (b>=0x20 && b<=0x7E)) ascii++;
  }
  return (float)ascii/(float)s.length();
}

// 16進数トークン（<XX>）が含まれる行か判定
inline bool isHexChar(char c){
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F');
}
inline bool containsHexToken(const String& printable){
  int pos = printable.indexOf('<');
  while (pos >= 0) {
    if (pos + 3 < printable.length()) {
      char c1 = printable[pos + 1];
      char c2 = printable[pos + 2];
      char c3 = printable[pos + 3];
      if (isHexChar(c1) && isHexChar(c2) && c3 == '>') {
        return true;
      }
    }
    pos = printable.indexOf('<', pos + 1);
  }
  return false;
}

/* ==================== ATコマンド実行（COPS特別処理＋共通ERRORリトライ） ==================== */
static int currentLineIndex=0;
static std::vector<String> commandLines;
bool commandsLoaded=false, waitingForCommandResponse=false;
unsigned long commandSentTime=0;

// 直近コマンド情報（再送用）
String currentCommandStr="";
unsigned long currentCommandTimeoutMs=COMMAND_RESPONSE_TIMEOUT_MS;

// COPS 特別モード（タイムアウト用）
bool isCopsCommandActive=false;
int  copsAttempts=0;
bool copsSawHex=false;
const int COPS_MAX_ATTEMPTS = 10;         // COPSタイムアウト再送上限

// ★ すべてのコマンドに共通の「ERROR時リトライ」制御
int  errorRetryAttempts = 0;
const int MAX_ERROR_RETRY = 10;

String sendDataString="", receiveDataString="";
bool hasSendData=false, hasReceiveData=false;

// ★ UART1 をサイクル時のみ利用
static const uint32_t BAUD_PC=115200, BAUD_UART=115200;
static const int UART1_TX_PIN=18, UART1_RX_PIN=19;
void uart1Enable(){ Serial1.begin(BAUD_UART, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN); Serial1.setRxBufferSize(4096); PC_PRINTLN("[UART1] Enabled"); }
void uart1Disable(){ Serial1.end(); PC_PRINTLN("[UART1] Disabled"); }

/* ==================== GP23: 中断／再開処理 ==================== */
void pauseProcessing(){
  if (userPauseEnabled) return;
  userPauseEnabled = true;

  if (isRunning){
    waitingForCommandResponse = false;
    uart1Disable();
    isRunning = false;
    logWrite("USER PAUSE (GP23 IRQ): processing paused");
    lcdClearBeforeNewLine(); lcdPrintLine("PAUSED by GP23"); showIdle();
  }else{
    logWrite("USER PAUSE (GP23 IRQ): idle");
    lcdClearBeforeNewLine(); lcdPrintLine("PAUSED (idle) by GP23"); showIdle();
  }
}

void resumeProcessing(){
  if (!userPauseEnabled) return;
  userPauseEnabled = false;

  logWrite("USER RESUME (GP23 IRQ): processing resumed");
  // 即再開（次周期で再開したい場合は、この行をコメントアウト）
  startRun();
}

// メインループ側で ISRイベントを処理（デバウンス＋ロックアウト）
void handlePauseSwitchIrqDriven(){
  // ISRで立ったフラグを安全に取り出してクリア
  bool event = false;
  #if defined(ESP32)
    portENTER_CRITICAL(&gp23Mux);
  #endif
  if (gp23IsrEvent){ event = true; gp23IsrEvent = false; }
  #if defined(ESP32)
    portEXIT_CRITICAL(&gp23Mux);
  #endif

  unsigned long now = millis();

  // ISRイベントが来たらデバウンス開始
  if (event){
    swDebouncing      = true;
    swDebounceStartMs = now;
  }

  // デバウンス完了判定（30ms後に Low が維持されているか）
  if (swDebouncing && (now - swDebounceStartMs >= SWITCH_DEBOUNCE_MS)){
    if (digitalRead(GP23_SWITCH_PIN) == LOW){
      // 5秒ロックアウト
      if (now - lastSwitchHandledMs >= SWITCH_LOCK_MS){
        if (userPauseEnabled) resumeProcessing();
        else                  pauseProcessing();
        lastSwitchHandledMs = now;
      }
    }
    swDebouncing = false;  // デバウンス終了
  }
}

/* ==================== ファイルロード ==================== */
void loadCommandFile(){
  if(!ensureFS()) return;
  fs::File file=LittleFS.open("/commands1.txt","r"); if(!file){ PC_PRINTLN("[ERROR] Failed to open commands1.txt"); return; }
  commandLines.clear();
  while(file.available()){ String line=file.readStringUntil('\n'); line.trim(); if(line.length()>0) commandLines.push_back(line); }
  file.close();
  PC_PRINTF("[INFO] Loaded %d lines from commands1.txt\n",(int)commandLines.size());
  commandsLoaded=true; currentLineIndex=0;
}

/* ==================== 画面（アイドル） ==================== */
void showIdleNextRunScreen(){
  lcdClearBeforeNewLine(); String rsrpLine;
  if(hasRsrpInfo && rsrpInfo.length()>0){ rsrpLine=rsrpInfo; lastRsrpInfo=rsrpInfo; hasLastRsrp=true; }
  else if(hasLastRsrp && lastRsrpInfo.length()>0){ rsrpLine=lastRsrpInfo; }
  else { rsrpLine="RSRP: N/A"; }
  lcdPrintLine(rsrpLine); lcdPrintLine("Next run in 10 seconds...");
  idleHoldDisplay=true;
}

/* ==================== 実行サイクル ==================== */
void startRun(){
  idleHoldDisplay=false; hasSendData=false; hasReceiveData=false; hasRsrpInfo=false; rsrpInfo="";
  waitingForCommandResponse=false; showIdle();
  lcdClearBeforeNewLine(); lcdPrintLine("Loading commands...");
  loadCommandFile();
  // インターバル終了 → UART1 を有効化
  uart1Enable();
  isRunning=true; delay(300);
  openNewLog(); logWrite("RUN START");
  sendNextCommand();
}

void finishRun(){
  uart1Disable();
  isRunning=false; lastRunMillis=millis();
  showIdleNextRunScreen();
  PC_PRINTLN("[INFO] Waiting for next 10 seconds run");
  logWrite("RUN END"); closeLog();
}

void restartFromTop(){
  PC_PRINTLN("[RESTART] Restarting from line 1");
  idleHoldDisplay=false; isRunning=true; waitingForCommandResponse=false;
  hasSendData=false; hasReceiveData=false; hasRsrpInfo=false; rsrpInfo="";
  sendDataString=""; receiveDataString=""; currentLineIndex=0;
  lcdClearBeforeNewLine(true); lcdPrintLine("Restarting from line 1..."); loadCommandFile(); delay(300);
  logWrite("RESTART FROM TOP");
  uart1Enable();
  sendNextCommand();
}

/* ==================== コマンド送出 ==================== */
void sendNextCommand(){
  if(!commandsLoaded || currentLineIndex >= (int)commandLines.size()){
    PC_PRINTLN("[INFO] All commands completed");
    if(hasSendData && hasReceiveData){
      if(sendDataString.equals(receiveDataString)){ showMatch(); passCountRun++; logWrite("RESULT: MATCH"); }
      else { showMismatch(); ngCountRun++; logWrite("RESULT: MISMATCH"); }
    }
    finishRun(); return;
  }

  String command=commandLines[currentLineIndex];
  if(command.startsWith("#")){ PC_PRINTF("[SKIP] Comment line: %s\n", command.c_str()); currentLineIndex++; sendNextCommand(); return; }

  // 送受比較対象抽出（例: AT%SOCKETDATA="SEND",1,13,"...）
  if(command.indexOf("AT%SOCKETDATA=\"SEND\",1,13,")>=0){
    int startPos=command.indexOf(",13,")+5;
    if(startPos>3 && startPos<(int)command.length()){
      sendDataString=command.substring(startPos); sendDataString.replace("\"","");
      if(sendDataString.length()>26) sendDataString=sendDataString.substring(0,26);
      hasSendData=true; PC_PRINTF("[INFO] SEND data captured (26 chars): %s\n", sendDataString.c_str());
    }
  }

  // 送信表示＆UART1送信
  lcdClearBeforeNewLine(); lcdPrintLine("CMD: "+command);
  PC_PRINTF("[TX->UART1] %s\n", command.c_str());
  Serial1.print(command); Serial1.print("\r\n");

  currentCommandStr = command;

  waitingForCommandResponse=true;
  commandSentTime=millis();
  responseLineCount=0;

  // ★ ERROR再送回数 初期化（全コマンド共通）
  errorRetryAttempts = 0;

  // COPS特別処理のセットアップ
  if(command.equals("AT+COPS=1,2,\"44020\"")){
    isCopsCommandActive = true;
    copsAttempts = 1;
    copsSawHex   = false;
    currentCommandTimeoutMs = 10000UL;      // COPSは10秒
    logWrite("COPS: special handling started (timeout=10s, max attempts=10)");
  }else{
    isCopsCommandActive = false;
    copsAttempts = 0;
    copsSawHex   = false;
    currentCommandTimeoutMs = COMMAND_RESPONSE_TIMEOUT_MS; // 既定30秒
  }

  // ここでは行を進めない（成功/失敗/タイムアウトで進める）
  logWrite(String("CMD: ")+command);
}

// 前進処理（短いウェイトのみ）
void advanceWhenReady(){ delay(200); sendNextCommand(); }

/* ==================== UART1受信（行確定） ==================== */
static std::vector<uint8_t> uart1Buf;
static uint32_t uart1LastByteMs=0;
static const uint32_t UART1_GAP_MS=15;

bool uart1GetLine(String &out){
  bool got=false;
  if(!isRunning) return false;
  while(Serial1.available()){
    uint8_t b=(uint8_t)Serial1.read(); uart1LastByteMs=millis();
    if(b=='\r') continue;
    if(b=='\n'){
      if(!uart1Buf.empty()){
        out.reserve(uart1Buf.size());
        for(auto v:uart1Buf) out+=(char)v;
        uart1Buf.clear();
        return true;
      }else continue;
    }
    uart1Buf.push_back(b);
    if(uart1Buf.size()>4096){
      out.reserve(uart1Buf.size());
      for(auto v:uart1Buf) out+=(char)v;
      uart1Buf.clear();
      return true;
    }
  }
  if(!uart1Buf.empty()){
    uint32_t now=millis();
    if(now - uart1LastByteMs >= UART1_GAP_MS){
      out.reserve(uart1Buf.size());
      for(auto v:uart1Buf) out+=(char)v;
      uart1Buf.clear();
      got=true;
    }
  }
  return got;
}

/* ==================== PC→ESP32：#LIST / #GET / #DELLOG ==================== */
void handlePcSerialCommands(){
  static String line;
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\n' || c=='\r'){
      if(line.length()==0) continue;
      String cmd=line; line=""; cmd.trim();
      if(cmd=="#LIST"){
        fs::File root=LittleFS.open("/logs");
        if(!root || !root.isDirectory()){ Serial.print("ERR:/logs open failed\r\n"); return; }
        fs::File f=root.openNextFile();
        while(f){ Serial.printf("LIST:%s:%u\r\n", f.path(), (unsigned)f.size()); f=root.openNextFile(); }
        root.close(); Serial.print("LISTEND\r\n"); return;
      }
      if(cmd.startsWith("#GET ")){
        String path=cmd.substring(5); path.trim();
        fs::File f=LittleFS.open(path,"r");
        if(!f){ Serial.printf("ERR:open %s failed\r\n", path.c_str()); return; }
        pcTransferActive=true;
        size_t sz=f.size();
        Serial.printf("FILEBEGIN %s %u\r\n", path.c_str(), (unsigned)sz);
        uint8_t buf[1024];
        while(f.available()){ size_t n=f.read(buf,sizeof(buf)); Serial.write(buf,n); yield(); }
        f.close();
        Serial.print("FILEEND\r\n");
        pcTransferActive=false; return;
      }
      if(cmd=="#DELLOG"){
        fs::File root=LittleFS.open("/logs");
        if(!root || !root.isDirectory()){ Serial.print("DELLOG:EMPTY\r\n"); return; }
        size_t files=0, bytes=0;
        fs::File f=root.openNextFile();
        while(f){ String path=f.path(); size_t sz=f.size(); f.close(); if(LittleFS.remove(path)){ files++; bytes+=sz; } f=root.openNextFile(); }
        root.close(); LittleFS.mkdir("/logs");
        Serial.printf("DELLOG:OK %u %u\r\n",(unsigned)files,(unsigned)bytes); return;
      }
      Serial.print("ERR:unknown cmd\r\n");
    }else{
      line+=c;
    }
  }
}

/* ==================== setup / loop ==================== */
void setup(){
  Serial.begin(BAUD_PC); delay(300);
  ensureFS(); LittleFS.mkdir("/logs");

  // 既存GPIO
  pinMode(20, OUTPUT); digitalWrite(20, HIGH); PC_PRINTLN("[GPIO20] Set to HIGH");

  // ==== GP23 スイッチ入力（押すと Low） & 割り込み（FALLING） ====
  pinMode(GP23_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(GP23_SWITCH_PIN, gp23_isr, FALLING);

  // 起動時は UART1 未開始（サイクル開始時に enable）
  delay(6000); // LCD初期化前待機
  lcd.begin(); lcd.setRotation(1); lcd.setBrightness(220);
  lcd.setFont(&fonts::Font4); lcd.setTextSize(textSize);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextDatum(textdatum_t::top_left);
  lineHeight=lcd.fontHeight()*textSize;
  lcdClearBeforeNewLine(); lcdPrintLine("READY - Log + Transfer (10sec, UART1 off in idle)");
  rgb.begin(); rgb.show(); showIdle();
  lastRunMillis=millis();

  // 起動直後に自動開始
  startRun();
}

void loop(){
  // PCファイル転送要求（最優先）
  handlePcSerialCommands();
  if(pcTransferActive) return;

  // ==== GP23 割り込みイベントの処理（デバウンス＋ロックアウト） ====
  handlePauseSwitchIrqDriven();

  // 中断中は以降の処理を最小化（ログ転送は継続）
  if(userPauseEnabled){
    return;
  }

  // GPIO20 自動復帰
  if(gpio20IsLow && (millis()-gpio20LowTime >= GPIO20_LOW_DURATION_MS)){
    digitalWrite(20, HIGH); gpio20IsLow=false; PC_PRINTLN("[GPIO20] Set to HIGH (timeout recovery)");
    if(pendingRestartAfterError){ pendingRestartAfterError=false; lcdPrintLine("Wait 6s before restart..."); delay(6000); restartFromTop(); return; }
  }

  // 1分おきに自動実行（中断中は開始しない）
  if(!isRunning && (millis()-lastRunMillis >= RUN_INTERVAL_MS)){
    startRun();
  }

  // レスポンスタイムアウト監視（個別タイムアウト）
  if(isRunning && waitingForCommandResponse &&
     (millis()-commandSentTime >= currentCommandTimeoutMs)){
    // ==== COPS 特別モード：10秒で再送 ====
    if(isCopsCommandActive && copsAttempts < COPS_MAX_ATTEMPTS){
      PC_PRINTF("[COPS] Timeout after %lu ms -> retry %d/%d (sawHex=%s)\n",
                (unsigned long)currentCommandTimeoutMs, copsAttempts+1, COPS_MAX_ATTEMPTS,
                copsSawHex ? "true":"false");
      logWrite(String("TIMEOUT: COPS 10s, retry ")+(copsAttempts+1)+"/"+String(COPS_MAX_ATTEMPTS)+
               (copsSawHex? " (hex seen)":" (hex not seen)"));

      Serial1.print(currentCommandStr); Serial1.print("\r\n");
      copsAttempts++;
      copsSawHex = false;
      commandSentTime = millis();
      responseLineCount=0;
      waitingForCommandResponse = true;
    }else{
      // 通常コマンドのタイムアウトは次へ
      PC_PRINTLN("[TIMEOUT] No response - Moving to next command");
      logWrite(String("TIMEOUT: ")+String(currentCommandTimeoutMs/1000)+"s");
      waitingForCommandResponse=false;
      currentLineIndex++;
      if(isCopsCommandActive){ isCopsCommandActive=false; }
      advanceWhenReady();
    }
  }

  // UART1受信・解析
  String resp;
  if(uart1GetLine(resp)){
    String printable=makePrintable(resp);
    PC_PRINT("[RX<-UART1] "); PC_PRINTLN(printable);
    responseLineCount++;
    if(!isRunning && idleHoldDisplay) return;

    // COPS: HEXストリーム検知
    if(isCopsCommandActive && !copsSawHex && containsHexToken(printable)){
      copsSawHex = true;
      logWrite("COPS: hex stream detected");
    }

    // %SOCKETDATA の受信データ抽出
    if(printable.indexOf("%SOCKETDATA:1,13,0,")>=0){
      int startPos=printable.indexOf(",0,")+3;
      if(startPos>2 && startPos<(int)printable.length()){
        String tmp=printable.substring(startPos);
        int q1=tmp.indexOf('"'), q2=tmp.lastIndexOf('"');
        if(q1>=0 && q2>q1){
          receiveDataString=tmp.substring(q1+1,q2);
          if(receiveDataString.length()>26) receiveDataString=receiveDataString.substring(0,26);
          hasReceiveData=true; PC_PRINTF("[INFO] RECEIVE data captured (26 chars): %s\n", receiveDataString.c_str());
        }
      }
    }

    // +CCLK
    if(printable.indexOf("+CCLK:")>=0){ currentClockStr=printable; hasClockFromModem=true; logWrite(String("CLOCK UPDATE: ")+printable); }

    // RSRP
    if(printable.indexOf("RSRP: Reported")>=0){
      int s=printable.indexOf("RSRP: Reported"), c=printable.indexOf(',',s);
      rsrpInfo=(c>s)? printable.substring(s,c): printable.substring(s);
      rsrpInfo.replace("Reported ",""); rsrpInfo+=" dBm";
      hasRsrpInfo=true; lastRsrpInfo=rsrpInfo; hasLastRsrp=true;
      PC_PRINTF("[INFO] RSRP info captured: %s\n", rsrpInfo.c_str());
      logWrite(String("RSRP: ")+rsrpInfo);
    }

    // LCD 短縮表示
    if(printable.length()>0){
      String d=(printable.length()>20)? printable.substring(0,17)+"..." : printable;
      lcdPrintLine("RSP: "+d);
    }

    // 完了判定（OK／ERROR）
    bool isOk    = (printable.indexOf("OK")    >= 0);
    bool isError = (printable.indexOf("ERROR") >= 0);

    // OK：成功→次へ
    if(isOk){
      waitingForCommandResponse=false;
      if(isCopsCommandActive){ isCopsCommandActive=false; }
      currentLineIndex++;
      logWrite("CMD: OK received");
      advanceWhenReady();
      logWrite(String("RSP: ")+printable);
      yield();
      return;
    }

    // ERROR：全コマンド共通で最大10回までリトライ
    if(isError){
      logWrite(String("RSP: ")+printable);
      if(errorRetryAttempts < MAX_ERROR_RETRY){
        PC_PRINTF("[RETRY] ERROR received -> retry %d/%d (cmd=%s)\n",
                  errorRetryAttempts+1, MAX_ERROR_RETRY, currentCommandStr.c_str());
        logWrite(String("ERROR: retry ")+(errorRetryAttempts+1)+"/"+String(MAX_ERROR_RETRY));

        Serial1.print(currentCommandStr); Serial1.print("\r\n");
        errorRetryAttempts++;
        if(isCopsCommandActive) copsSawHex = false;
        commandSentTime = millis();
        responseLineCount=0;
        waitingForCommandResponse = true;
        yield();
        return;
      }else{
        // 上限到達：NG処理
        waitingForCommandResponse=false;
        if(isCopsCommandActive){ isCopsCommandActive=false; }

        ngCountRun++; showMismatch();
        if(!gpio20IsLow){
          digitalWrite(20,LOW); gpio20IsLow=true; gpio20LowTime=millis();
          PC_PRINTLN("[GPIO20] LOW (ERROR reset sequence started)");
        }
        pendingRestartAfterError=true; lcdClearBeforeNewLine();
        lcdPrintLine("RSP: ERROR detected (failed after max retries)"); lcdPrintLine("Reset GPIO20 -> wait 6s");
        logWrite("EVENT: ERROR after max retries; GPIO20 LOW; pending restart");

        currentLineIndex++;
        advanceWhenReady();
        yield();
        return;
      }
    }

    // 受信ログ（OK/ERROR以外）
    logWrite(String("RSP: ")+printable);
    yield();
  }
}
