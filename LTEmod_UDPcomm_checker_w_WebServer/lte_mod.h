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

/* ===== HEXユーティリティ ===== */
inline bool isHexNibble(char c){ return (c>='0'&&c<='9')||(c>='A'&&c<='F')||(c>='a'&&c<='f'); }
inline int hexVal(char c){
  if(c>='0'&&c<='9') return c-'0';
  if(c>='A'&&c<='F') return 10+(c-'A');
  if(c>='a'&&c<='f') return 10+(c-'a');
  return -1;
}
bool isPureHexString(const String& s){
  if(s.length()==0 || (s.length()%2)!=0) return false;
  for(size_t i=0;i<s.length();++i){ if(!isHexNibble(s[i])) return false; }
  return true;
}
String hexToBytesString(const String& hex){
  String out; out.reserve(hex.length()/2);
  for(size_t i=0;i<hex.length(); i+=2){
    int hi=hexVal(hex[i]), lo=hexVal(hex[i+1]);
    if(hi<0||lo<0){ break; }
    char b=(char)((hi<<4)|lo);
    out+=b;
  }
  return out;
}

/* ==================== %SOCKETDATA パーサ ==================== */
bool parseSocketdataRx(const String& line, int& cid, int& len, int& flags, String& payloadRaw){
  int p=line.indexOf("%SOCKETDATA:");
  if(p<0) return false;
  int q1=line.indexOf('"', p);
  int q2=line.lastIndexOf('"');
  String prefix=line.substring(p+12, q1);
  prefix.trim();
  int c1=prefix.indexOf(','); if(c1<0) return false;
  cid = prefix.substring(0,c1).toInt();
  int c2=prefix.indexOf(',', c1+1); if(c2<0) return false;
  len = prefix.substring(c1+1,c2).toInt();
  int c3=prefix.indexOf(',', c2+1); if(c3<0) return false;
  flags = prefix.substring(c2+1,c3).toInt();
  if(q1<0 || q2<=q1) return false;
  payloadRaw = line.substring(q1+1,q2);
  return true;
}
bool parseSocketdataSendCmd(const String& cmd, int& cid, int& len, String& payloadRaw){
  int p=cmd.indexOf("AT%SOCKETDATA=\"SEND\"");
  if(p<0) return false;
  int q2=cmd.lastIndexOf('"'); // payload closing
  int q1=cmd.lastIndexOf('"', q2-1); // payload opening
  if(q1<0 || q2<=q1) return false;
  payloadRaw = cmd.substring(q1+1, q2);
  int c2=cmd.lastIndexOf(',', q1-1);
  int c1=cmd.lastIndexOf(',', c2-1);
  if(c1<0 || c2<0) return false;
  len = cmd.substring(c1+1, c2).toInt();
  int c0=cmd.lastIndexOf(',', c1-1); if(c0<0) return false;
  cid = cmd.substring(c0+1, c1).toInt();
  return true;
}

/* ==================== ATコマンド実行関連 ==================== */
static int currentLineIndex=0;
static std::vector<String> commandLines;
bool commandsLoaded=false, waitingForCommandResponse=false;
unsigned long commandSentTime=0;
String currentCommandStr="";
unsigned long currentCommandTimeoutMs=10000UL;

/* ==== COPS 特別処理 ==== */
bool isCopsCommandActive=false;
int copsAttempts=0;
bool copsSawHex=false;
const int COPS_MAX_ATTEMPTS = 10;
const unsigned long COPS_TIMEOUT_PER_TRY_MS = 10000UL;

/* ==== ERROR リトライ ==== */
int errorRetryAttempts = 0;
const int MAX_ERROR_RETRY = 10;

/* ==== 送受比較用 ==== */
String sendDataString="", receiveDataString="";
bool hasSendData=false, hasReceiveData=false;
