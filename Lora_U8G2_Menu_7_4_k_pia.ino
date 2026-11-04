#include "Arduino.h"
#include <math.h>
#define FREQUENCY_868

// =============================================================================
// BUILD SELECT — pred flashom nastav ROLE_SERVER alebo ROLE_CLIENT
// =============================================================================
#define ROLE_SERVER 1
#define ROLE_CLIENT 2
#define DEVICE_ROLE ROLE_CLIENT

#include <U8g2lib.h>
#include <Wire.h>
#include <STM32RTC.h>
#include <IWatchdog.h>
#include "LoRa_E220.h"
#include <INA219B.h>
#include "include/states.h"
#include "include/protocol.h"

#if DEVICE_ROLE == ROLE_SERVER
#include <SPI.h>
#include "SdFat.h"
#endif


// -----------------------------------------------------------------------------
// LoRa addresses (must differ!)
// -----------------------------------------------------------------------------
#define SERVER_ADDH    0x00
#define SERVER_ADDL    0x01
#define CLIENT_ADDH    0x00
#define CLIENT_ADDL    0x02
#define EXTERNAL_ADDH  0x00   // tretia strana (PLC / gateway)
#define EXTERNAL_ADDL  0x10
#define LORA_CHANNEL   16

#define AUX_PIN        PB12
#define IGNORE_PIN     PA8
#define RELAY          PA9

#if DEVICE_ROLE == ROLE_SERVER
#define FLOW           PB0
#define PRESS          PA1
#define OCHRANA        PB13
#define SD_CS_PIN      PA4
static SPIClass mySPI(PA7, PA6, PA5, SD_CS_PIN);
#define SD2_CONFIG     SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(18), &mySPI)
#endif

#if DEVICE_ROLE == ROLE_CLIENT
#define TANK_MAX       PB13
#define TANK_MIN       PB14
#define TANK_HEIGHT_CM 178.0f
#define TANK_OVERFLOW_CM 17.0f
#define TANK_MIN_CM    88.0f
#endif

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------
INA219B ina219b;
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
STM32RTC& rtc = STM32RTC::getInstance();

// STM32 core 3.x: konkrétna trieda je Uart (HardwareSerial je abstraktná)
Uart SerialLoRa(PB7, PB6);   // RX, TX
Uart DebugSe(PA12, PA11);    // RX, TX
LoRa_E220 e220ttl(&SerialLoRa, PB12, PA10, PB10, UART_BPS_RATE_9600);

#if DEVICE_ROLE == ROLE_CLIENT
Uart LevelMeter(PA3, PA2);   // RX, TX
#endif

#if DEVICE_ROLE == ROLE_SERVER
SdFat SD;
File dataFile;
File settingsFile;
#endif

// -----------------------------------------------------------------------------
// Debug
// -----------------------------------------------------------------------------
#define TANK_DEBUG
#ifdef TANK_DEBUG
  #define DEBUG_PRINT(...)   { DebugSe.print(__VA_ARGS__); }
  #define DEBUG_PRINTF(...)  { DebugSe.printf(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { DebugSe.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINTLN(...)
#endif

// -----------------------------------------------------------------------------
// Timing
// -----------------------------------------------------------------------------
static const unsigned long INTERVAL_IDLE_MS   = 900000UL;  // 15 min
static const unsigned long INTERVAL_PUMP_MS   = 60000UL;   // 60 s
static const unsigned long INTERVAL_PING_MS   = 10000UL;   // client ping
static const unsigned long ACK_TIMEOUT_MS     = 800UL;
static const unsigned long MAX_NO_MSG_PUMP_MS = 900000UL;  // 15 min
static const unsigned long MAX_NO_MSG_PING_MS = 120000UL;
static const unsigned long MAX_NO_MSG_REMOTE_MS = 3600000UL; // 1 h (remote)
static const unsigned long REMOTE_MAX_RUN_MS    = 3600000UL; // 1 h max len pre remote START (nie FILL)
static const unsigned long DISP_INTERVAL_MS   = 1000UL;
static const unsigned long LOG_INTERVAL_MS    = 33000UL;
static const uint8_t MAX_RETRIES = 3;
static const size_t UART_LINE_MAX = 80;

// -----------------------------------------------------------------------------
// Shared state
// -----------------------------------------------------------------------------
float batteryLocalV = 0.0f;
float batteryRemoteV = 0.0f;
float pressure = 0.0f;
float filledper = 0.0f;
float filled_today = 0.0f;
bool maxLevel = false;
bool minLevel = false;
bool pumpIsActive = false;
bool ignoreFilledToday = false;
bool clientDisconnected = true;
bool needFtRestart = false;
bool press_low = true;
bool stavochrany = false;
uint8_t txSeq = 1;
uint8_t lastRxSeq = 0;
uint8_t pendingAckSeq = 0;
bool waitingAck = false;
uint8_t retryCount = 0;
unsigned long lastSendMs = 0;
unsigned long lastRxMs = 0;
unsigned long lastPollMs = 0;
unsigned long lastDispMs = 0;
unsigned long lastLogMs = 0;
unsigned long ignoreBtnDownMs = 0;
bool ignoreBtnWasHigh = false;

char zvol_rezim[10] = "AUTO";
char och_dis[8] = "OK";
char lastmess[20] = "--";
char errorMessage[40] = "";

OPERATION_MODE operationalSelected = OPERATION_NORMAL;

// UART config line buffer
char uartLine[UART_LINE_MAX];
uint8_t uartLineLen = 0;

#if DEVICE_ROLE == ROLE_SERVER
volatile unsigned long pulseCount = 0;
float maxvol = 0.0f;
int fillinterval = 0;
float filled_log = 0.0f;
unsigned long pumpStartTime = 0;
unsigned long pumpStopTime = 0;
bool actReceived = false;
bool clientSensorFault = false; // klient: zamrznutý / chybný levelmeter

// Remote / third-party control
bool remotePumpActive = false;
bool remoteFillMode = false;
float remoteTargetQuantity = 0.0f;
float remoteFilledStart = 0.0f;
unsigned long remoteStartMs = 0;
uint8_t lastRemoteStatus = REMOTE_ST_OK;
#endif

#if DEVICE_ROLE == ROLE_CLIENT
float levelInCM = 0.0f;
bool pumpCmdOn = false;
bool changedLevels = false;
bool needAckFromServer = false;
unsigned long ackStartMs = 0;
float lastLevelInCM = -1.0f;
unsigned long lastLevelChangeMs = 0;
bool fallbackMode = false;
bool sensorFault = false;
const unsigned long LEVEL_CHANGE_TIMEOUT = 10800000UL; // 3 h bez zmeny hladiny -> plaváky
bool inWorSleep = false;
#endif

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
void configureLoRa();
void printParameters(Configuration configuration);
float getBatteryVoltage();
void drawFillBar(int x, int y, int width, int height, int progress);
void formatRtcTime(char* buf, size_t buflen);
void handleIgnoreButton();
void updateRezimLabels();
bool sendProtoTo(uint8_t dstH, uint8_t dstL, uint8_t type, uint8_t flags,
                 const uint8_t* payload, uint8_t payloadLen, bool useWorWake);
bool sendProto(uint8_t type, uint8_t flags, const uint8_t* payload, uint8_t payloadLen, bool useWorWake);
bool sendProtoRetry(uint8_t type, uint8_t flags, const uint8_t* payload, uint8_t payloadLen, bool useWorWake);
void processIncoming();
bool tryParseBuffer(const uint8_t* data, size_t len);
void onPacket(const ProtoPacket& pkt);
void setModeNormal();
void setModeWorTx();
void setModeWorRx();
void drawScreen();
void processUartMenu();
void uartPrintHelp();
void uartPrintTime();
bool uartSetDateTime(const char* args);
void uartPrintStatus();

#if DEVICE_ROLE == ROLE_SERVER
void pulseFlowCounter();
void logWrite();
void logRead();
void loadSettings();
void readPress();
void startPump();
void stopPump();
void managePump();
void sendPumpCmd(bool wakeClient, bool ackReq);
void sendAck(uint8_t seq);
void sendTimeSync();
uint8_t handleRemoteCommand(uint8_t cmd, float volumeM3);
void clearRemoteMode();
void sendRemoteAck(uint8_t status);
float remoteSessionFilled();
#endif

#if DEVICE_ROLE == ROLE_CLIENT
void waterLevel();
void sendTankStatus(bool ackReq);
void sendManual(bool ignore);
void sendAck(uint8_t seq);
void enterWorSleepIfIdle();
void wakeToNormal();
#endif

// =============================================================================
// SETUP / LOOP
// =============================================================================
void setup() {
#ifdef TANK_DEBUG
  DebugSe.begin(115200);
#endif
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  Wire.begin();
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
#if DEVICE_ROLE == ROLE_SERVER
  u8g2.drawStr(0, 20, "SERVER BOOT");
#else
  u8g2.drawStr(0, 20, "CLIENT BOOT");
#endif
  u8g2.sendBuffer();
  delay(300);

  pinMode(IGNORE_PIN, INPUT);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);

#if DEVICE_ROLE == ROLE_SERVER
  pinMode(OCHRANA, INPUT);
  attachInterrupt(FLOW, pulseFlowCounter, FALLING);

  if (!SD.begin(SD2_CONFIG)) {
    u8g2.clearBuffer();
    u8g2.drawStr(0, 20, "SD PROBLEM!");
    u8g2.sendBuffer();
    DEBUG_PRINTLN(F("SD init failed"));
    while (1) {}
  }
  dataFile = SD.open("log.txt", FILE_WRITE);
  if (dataFile) dataFile.close();
  loadSettings();
  logRead();
  filled_today = filled_log;
  pulseCount = (unsigned long)(filled_today * 12000.0f);
#endif

#if DEVICE_ROLE == ROLE_CLIENT
  pinMode(TANK_MAX, INPUT_PULLUP);
  pinMode(TANK_MIN, INPUT_PULLUP);
  LevelMeter.begin(9600);
  lastLevelChangeMs = millis();
#endif

  if (ina219b.begin()) DEBUG_PRINTLN(F("INA219 OK"));
  ina219b.reset();
  ina219b.setPGAGain(GAIN_8_320MV);
  ina219b.setBusRange(RANGE_32V);
  ina219b.setCalibration(4096);
#if DEVICE_ROLE == ROLE_CLIENT
  ina219b.setBusADCRes(BUS_RES_2S);
  ina219b.setShuntADCRes(SHUNT_RES_2S);
#else
  ina219b.setBusADCRes(BUS_RES_8S);
  ina219b.setShuntADCRes(SHUNT_RES_8S);
#endif
  ina219b.setOperationMode(SANDBVOLT_CONTINUOUS);

  rtc.begin();
  e220ttl.begin();
  delay(100);
  configureLoRa();

  getBatteryVoltage();
  lastPollMs = millis();
  lastRxMs = millis();
  IWatchdog.begin(4000000);

#if DEVICE_ROLE == ROLE_CLIENT
  enterWorSleepIfIdle();
#endif

  DEBUG_PRINTLN(F("INIT OK"));
  uartPrintHelp();
}

void loop() {
  IWatchdog.reload();
  processUartMenu();
  handleIgnoreButton();
  updateRezimLabels();
  processIncoming();

  // Daily filled reset at midnight
  if (rtc.getHours() == 23) needFtRestart = true;
  if (rtc.getHours() == 0 && needFtRestart) {
#if DEVICE_ROLE == ROLE_SERVER
    filled_today = 0.0f;
    pulseCount = 0;
#endif
    needFtRestart = false;
  }

  if (maxLevel) ignoreFilledToday = false;

#if DEVICE_ROLE == ROLE_SERVER
  stavochrany = (digitalRead(OCHRANA) == LOW);
  strcpy(och_dis, stavochrany ? "OK" : "!!VYP!!");

  unsigned long interval = pumpIsActive ? INTERVAL_PUMP_MS : INTERVAL_IDLE_MS;
  if (millis() - lastPollMs >= interval) {
    lastPollMs = millis();
    if (!remotePumpActive) operationalSelected = OPERATION_NORMAL;
    sendPumpCmd(true, true);
    managePump();
  }

  if (waitingAck && (millis() - lastSendMs >= ACK_TIMEOUT_MS)) {
    if (retryCount < MAX_RETRIES) {
      retryCount++;
      DEBUG_PRINTLN(F("ACK timeout, retry"));
      sendPumpCmd(true, true);
    } else {
      waitingAck = false;
      retryCount = 0;
      DEBUG_PRINTLN(F("ACK failed"));
    }
  }

  if (pumpIsActive) {
    // FILL: bez časového limitu na beh; stop len cieľ m3 / max / tlak / ručný stop
    // START remote: 1h bez odpovede klienta; bežný režim: pôvodné limity
    unsigned long limit = 0;
    if (remotePumpActive && remoteFillMode) {
      limit = 0; // žiadny timeout na RX počas FILL
    } else if (remotePumpActive) {
      limit = MAX_NO_MSG_REMOTE_MS;
    } else if (operationalSelected == OPERATION_PING) {
      limit = MAX_NO_MSG_PING_MS;
    } else {
      limit = MAX_NO_MSG_PUMP_MS;
    }

    if (limit > 0 && (millis() - lastRxMs > limit)) {
      stopPump();
      clearRemoteMode();
      strncpy(errorMessage, "Nie je odpoved!", sizeof(errorMessage) - 1);
      clientDisconnected = true;
      sendRemoteAck(REMOTE_ST_STOPPED);
    }

    // Hard 1h cap len pre remote START (FILL beží do cieľového m3 / max / stop)
    if (remotePumpActive && !remoteFillMode &&
        (millis() - remoteStartMs >= REMOTE_MAX_RUN_MS)) {
      stopPump();
      clearRemoteMode();
      strncpy(errorMessage, "Remote 1h timeout", sizeof(errorMessage) - 1);
      sendPumpCmd(true, false);
      sendRemoteAck(REMOTE_ST_STOPPED);
      DEBUG_PRINTLN(F("Remote START 1h timeout"));
    }
  }

  managePump();
  logWrite();
#endif

#if DEVICE_ROLE == ROLE_CLIENT
  if (pumpCmdOn) {
    digitalWrite(RELAY, HIGH);
    pumpIsActive = true;
  } else {
    digitalWrite(RELAY, LOW);
    pumpIsActive = false;
  }

  // Rýchla kontrola plavákov aj v WOR (bez UART levelmetra)
  {
    bool fMax = (digitalRead(TANK_MAX) == LOW);
    bool fMin = (digitalRead(TANK_MIN) == LOW);
    if (fallbackMode || sensorFault) {
      // Pri chybe senzora majú plaváky plnú prioritu
      if (fMax != maxLevel || fMin != minLevel) {
        maxLevel = fMax;
        minLevel = fMin;
        changedLevels = true;
      }
    } else if (fMax && !maxLevel) {
      // Hard safety: MAX plavák vždy zastaví čerpanie
      maxLevel = true;
      changedLevels = true;
    }
  }

  bool needPing = (operationalSelected == OPERATION_PING) &&
                  (millis() - lastPollMs >= INTERVAL_PING_MS);
  if (needPing || changedLevels ||
      (needAckFromServer && millis() - ackStartMs >= INTERVAL_PING_MS)) {
    wakeToNormal();
    if (needAckFromServer) ackStartMs = millis();
    sendTankStatus(needAckFromServer || changedLevels);
    changedLevels = false;
    lastPollMs = millis();
  }

  if (waitingAck && (millis() - lastSendMs >= ACK_TIMEOUT_MS)) {
    if (retryCount < MAX_RETRIES) {
      retryCount++;
      wakeToNormal();
      sendTankStatus(true);
    } else {
      waitingAck = false;
      retryCount = 0;
    }
  }

  enterWorSleepIfIdle();
#endif

  if (millis() - lastDispMs >= DISP_INTERVAL_MS) {
    lastDispMs = millis();
    getBatteryVoltage();
#if DEVICE_ROLE == ROLE_SERVER
    readPress();
#endif
#if DEVICE_ROLE == ROLE_CLIENT
    if (!inWorSleep) waterLevel();
#endif
    drawScreen();
  }
}

// =============================================================================
// LoRa mode / config
// =============================================================================
void setModeNormal() {
  e220ttl.setMode(MODE_0_NORMAL);
}

void setModeWorTx() {
  e220ttl.setMode(MODE_1_WOR_TRANSMITTER);
}

void setModeWorRx() {
  e220ttl.setMode(MODE_2_WOR_RECEIVER);
}

void configureLoRa() {
  ResponseStructContainer c = e220ttl.getConfiguration();
  Configuration configuration = *(Configuration*)c.data;
  c.close();

#if DEVICE_ROLE == ROLE_SERVER
  configuration.ADDH = SERVER_ADDH;
  configuration.ADDL = SERVER_ADDL;
#else
  configuration.ADDH = CLIENT_ADDH;
  configuration.ADDL = CLIENT_ADDL;
#endif
  configuration.CHAN = LORA_CHANNEL;

  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.airDataRate = AIR_DATA_RATE_100_96;
  configuration.SPED.uartParity = MODE_00_8N1;

  configuration.OPTION.subPacketSetting = SPS_064_10; // max ~64 B na paket
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration.OPTION.transmissionPower = POWER_22;

  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  DEBUG_PRINTLN(rs.getResponseDescription());

  c = e220ttl.getConfiguration();
  Configuration configurationNew = *(Configuration*)c.data;
  printParameters(configurationNew);
  c.close();

  setModeNormal();
}

void printParameters(Configuration configuration) {
  DEBUG_PRINTLN(F("---- LoRa CFG ----"));
  DEBUG_PRINT(F("AddH: ")); DEBUG_PRINTLN(configuration.ADDH, HEX);
  DEBUG_PRINT(F("AddL: ")); DEBUG_PRINTLN(configuration.ADDL, HEX);
  DEBUG_PRINT(F("Chan: ")); DEBUG_PRINTLN(configuration.CHAN, DEC);
  DEBUG_PRINT(F("Air : ")); DEBUG_PRINTLN(configuration.SPED.getAirDataRateDescription());
  DEBUG_PRINTLN(F("------------------"));
}

// =============================================================================
// Binary TX / RX
// =============================================================================
bool sendProtoTo(uint8_t dstH, uint8_t dstL, uint8_t type, uint8_t flags,
                 const uint8_t* payload, uint8_t payloadLen, bool useWorWake) {
  ProtoPacket pkt;
  pkt.type = type;
  pkt.seq = txSeq++;
  if (txSeq == 0) txSeq = 1;
  pkt.flags = flags;
  pkt.payloadLen = payloadLen;
  if (payloadLen && payload) memcpy(pkt.payload, payload, payloadLen);

  uint8_t frame[PROTO_MAX_FRAME];
  size_t frameLen = protoBuildFrame(pkt, frame, sizeof(frame));
  if (frameLen == 0 || frameLen > PROTO_MAX_AIR) {
    DEBUG_PRINTLN(F("TX abort: frame > 64B or build fail"));
    return false;
  }

  if (useWorWake) setModeWorTx();
  else setModeNormal();

  ResponseStatus rs = e220ttl.sendFixedMessage(dstH, dstL, LORA_CHANNEL, frame, frameLen);
  DEBUG_PRINT(F("TX type=0x"));
  DEBUG_PRINT(type, HEX);
  DEBUG_PRINT(F(" dst="));
  DEBUG_PRINT(dstH, HEX);
  DEBUG_PRINT(F(":"));
  DEBUG_PRINT(dstL, HEX);
  DEBUG_PRINT(F(" len="));
  DEBUG_PRINT(frameLen);
  DEBUG_PRINT(F(" -> "));
  DEBUG_PRINTLN(rs.getResponseDescription());

  if (useWorWake) delay(20);
  setModeNormal();
#if DEVICE_ROLE == ROLE_CLIENT
  inWorSleep = false;
#endif

  lastSendMs = millis();
  if (flags & FLAG_ACK_REQ) {
    waitingAck = true;
    pendingAckSeq = pkt.seq;
  }
  return (rs.code == E220_SUCCESS) || (rs.code == 1);
}

bool sendProto(uint8_t type, uint8_t flags, const uint8_t* payload, uint8_t payloadLen, bool useWorWake) {
#if DEVICE_ROLE == ROLE_SERVER
  return sendProtoTo(CLIENT_ADDH, CLIENT_ADDL, type, flags, payload, payloadLen, useWorWake);
#else
  return sendProtoTo(SERVER_ADDH, SERVER_ADDL, type, flags, payload, payloadLen, useWorWake);
#endif
}

bool sendProtoRetry(uint8_t type, uint8_t flags, const uint8_t* payload, uint8_t payloadLen, bool useWorWake) {
  retryCount = 0;
  return sendProto(type, flags, payload, payloadLen, useWorWake);
}

void processIncoming() {
  if (e220ttl.available() <= 0) return;

  ResponseContainer rc = e220ttl.receiveMessage();
  if (rc.status.code != E220_SUCCESS && rc.status.code != 1) {
    DEBUG_PRINTLN(rc.status.getResponseDescription());
    return;
  }

  const uint8_t* data = reinterpret_cast<const uint8_t*>(rc.data.c_str());
  size_t len = rc.data.length();
  if (len == 0) return;

  // Scan for framed packets inside buffer
  size_t start = 0;
  while (start < len) {
    while (start < len && data[start] != PROTO_START) start++;
    if (start >= len) break;
    size_t end = start + 1;
    while (end < len && data[end] != PROTO_END) end++;
    if (end >= len) break;
    tryParseBuffer(data + start, end - start + 1);
    start = end + 1;
  }
}

bool tryParseBuffer(const uint8_t* data, size_t len) {
  ProtoPacket pkt;
  if (!protoParseFrame(data, len, pkt)) {
    DEBUG_PRINTLN(F("Bad frame/CRC"));
    return false;
  }
  // Duplicate filter (skip for third-party remote + ACK)
  if (pkt.type != MSG_ACK && pkt.type != MSG_REMOTE_CTRL && pkt.seq == lastRxSeq) {
    DEBUG_PRINTLN(F("Dup seq"));
    return false;
  }
  if (pkt.type != MSG_ACK && pkt.type != MSG_REMOTE_CTRL) lastRxSeq = pkt.seq;

  // Remote ctrl does not count as client keepalive
  if (pkt.type != MSG_REMOTE_CTRL) lastRxMs = millis();
  onPacket(pkt);
  return true;
}

void onPacket(const ProtoPacket& pkt) {
  DEBUG_PRINT(F("RX type=0x"));
  DEBUG_PRINTLN(pkt.type, HEX);

  if (pkt.type == MSG_ACK) {
    if (pkt.payloadLen >= sizeof(AckPayload)) {
      AckPayload ack;
      memcpy(&ack, pkt.payload, sizeof(ack));
      if (waitingAck && ack.ackSeq == pendingAckSeq) {
        waitingAck = false;
        retryCount = 0;
#if DEVICE_ROLE == ROLE_CLIENT
        needAckFromServer = false;
#endif
        DEBUG_PRINTLN(F("ACK OK"));
      }
    }
    return;
  }

#if DEVICE_ROLE == ROLE_SERVER
  if (pkt.type == MSG_TANK_STATUS && pkt.payloadLen >= sizeof(TankStatusPayload)) {
    TankStatusPayload p;
    memcpy(&p, pkt.payload, sizeof(p));
    maxLevel = (p.levels & 0x01) != 0;
    minLevel = (p.levels & 0x02) != 0;
    clientSensorFault = (p.levels & 0x04) != 0; // bit2: sensor fault / float fallback
    filledper = protoU16x100ToFloat(p.filledPct_x100);
    batteryRemoteV = protoMvToVolts(p.bat_mV);
    clientDisconnected = false;
    actReceived = true;
    errorMessage[0] = '\0';
    formatRtcTime(lastmess, sizeof(lastmess));

    if (pkt.flags & FLAG_ACK_REQ) sendAck(pkt.seq);
    managePump();
  } else if (pkt.type == MSG_MANUAL && pkt.payloadLen >= sizeof(ManualPayload)) {
    ManualPayload p;
    memcpy(&p, pkt.payload, sizeof(p));
    ignoreFilledToday = (p.ignoreFtd != 0);
    if (pkt.flags & FLAG_ACK_REQ) sendAck(pkt.seq);
  } else if (pkt.type == MSG_TIME_SYNC && pkt.payloadLen >= sizeof(TimeSyncPayload)) {
    TimeSyncPayload t;
    memcpy(&t, pkt.payload, sizeof(t));
    if (t.month >= 1 && t.month <= 12 && t.day >= 1 && t.day <= 31) {
      rtc.setYear(t.year);
      rtc.setMonth(t.month);
      rtc.setDay(t.day);
      rtc.setHours(t.hour);
      rtc.setMinutes(t.minute);
      rtc.setSeconds(t.second);
    }
  } else if (pkt.type == MSG_REMOTE_CTRL && pkt.payloadLen >= sizeof(RemoteCtrlPayload)) {
    RemoteCtrlPayload r;
    memcpy(&r, pkt.payload, sizeof(r));
    float vol = protoU16x100ToFloat(r.volume_x100);
    uint8_t st = handleRemoteCommand(r.cmd, vol);
    if (pkt.flags & FLAG_ACK_REQ) sendAck(pkt.seq);
    sendRemoteAck(st);
  }
#endif

#if DEVICE_ROLE == ROLE_CLIENT
  wakeToNormal();

  if (pkt.type == MSG_PUMP_CMD && pkt.payloadLen >= sizeof(PumpCmdPayload)) {
    PumpCmdPayload p;
    memcpy(&p, pkt.payload, sizeof(p));
    operationalSelected = (OPERATION_MODE)p.mode;
    filled_today = protoU16x100ToFloat(p.filledToday_x100);
    pressure = protoU16x100ToFloat(p.pressure_x100);
    batteryRemoteV = protoMvToVolts(p.bat_mV);
    pumpCmdOn = (pkt.flags & FLAG_PUMP_ON) != 0;
    stavochrany = (pkt.flags & FLAG_OCHRANA_OK) != 0;
    strcpy(och_dis, stavochrany ? "OK" : "!!VYP!!");
    if (pkt.flags & FLAG_IGNORE_FTD) ignoreFilledToday = true;
    formatRtcTime(lastmess, sizeof(lastmess));

    waterLevel();
    getBatteryVoltage();

    if (pkt.flags & FLAG_ACK_REQ) {
      sendAck(pkt.seq);
    }
    // Always reply with tank status after pump cmd
    sendTankStatus(false);
    lastPollMs = millis();
  } else if (pkt.type == MSG_TIME_SYNC && pkt.payloadLen >= sizeof(TimeSyncPayload)) {
    TimeSyncPayload t;
    memcpy(&t, pkt.payload, sizeof(t));
    if (t.month >= 1 && t.month <= 12 && t.day >= 1 && t.day <= 31) {
      rtc.setYear(t.year);
      rtc.setMonth(t.month);
      rtc.setDay(t.day);
      rtc.setHours(t.hour);
      rtc.setMinutes(t.minute);
      rtc.setSeconds(t.second);
    }
    if (pkt.flags & FLAG_ACK_REQ) sendAck(pkt.seq);
  } else if (pkt.type == MSG_MANUAL && pkt.payloadLen >= sizeof(ManualPayload)) {
    ManualPayload p;
    memcpy(&p, pkt.payload, sizeof(p));
    ignoreFilledToday = (p.ignoreFtd != 0);
    if (pkt.flags & FLAG_ACK_REQ) sendAck(pkt.seq);
  }
#endif
}

void sendAck(uint8_t seq) {
  AckPayload ack;
  ack.ackSeq = seq;
  sendProto(MSG_ACK, 0, reinterpret_cast<uint8_t*>(&ack), sizeof(ack), false);
}

// =============================================================================
// Shared helpers
// =============================================================================
float getBatteryVoltage() {
  batteryLocalV = ina219b.getBusVoltage_V();
  return batteryLocalV;
}

void formatRtcTime(char* buf, size_t buflen) {
  snprintf(buf, buflen, "%02u.%02u.%02u %02u:%02u",
           rtc.getDay(), rtc.getMonth(), rtc.getYear(),
           rtc.getHours(), rtc.getMinutes());
}

void updateRezimLabels() {
#if DEVICE_ROLE == ROLE_SERVER
  if (remotePumpActive) {
    strcpy(zvol_rezim, remoteFillMode ? "FILL" : "REMOTE");
  } else
#endif
  {
    strcpy(zvol_rezim, ignoreFilledToday ? "MANUAL" : "AUTO");
  }
}

void handleIgnoreButton() {
  bool high = digitalRead(IGNORE_PIN) == HIGH;
  if (high && !ignoreBtnWasHigh) {
    ignoreBtnDownMs = millis();
    ignoreBtnWasHigh = true;
  } else if (!high && ignoreBtnWasHigh) {
    unsigned long dur = millis() - ignoreBtnDownMs;
    ignoreBtnWasHigh = false;
    if (dur >= 100 && dur < 2000) {
      ignoreFilledToday = true;
#if DEVICE_ROLE == ROLE_CLIENT
      wakeToNormal();
      sendManual(true);
#elif DEVICE_ROLE == ROLE_SERVER
      sendPumpCmd(true, false);
#endif
    } else if (dur >= 2000) {
      ignoreFilledToday = false;
#if DEVICE_ROLE == ROLE_CLIENT
      wakeToNormal();
      sendManual(false);
#endif
    }
  }
}

void drawFillBar(int x, int y, int width, int height, int progress) {
  if (progress > 100) progress = 100;
  if (progress < 0) progress = 0;
  float bar = ((float)(height - 4) / 100.0f) * progress;
  u8g2.drawFrame(x, y, width, height);
  u8g2.drawBox(x + 2, y + height - (int)bar - 2, width - 4, (int)bar);
}

void drawScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_squeezed_b7_tr);
  u8g2.setCursor(1, 9);
  if (errorMessage[0]) {
    u8g2.print(errorMessage);
  } else {
    u8g2.print(zvol_rezim);
    u8g2.print(F(" P="));
    u8g2.print(pressure, 1);
    u8g2.print(F("b"));
  }

  u8g2.setFont(u8g2_font_squeezed_r7_tr);
  u8g2.setCursor(1, 18);
#if DEVICE_ROLE == ROLE_SERVER
  if (remotePumpActive && remoteFillMode) {
    u8g2.print(F("Fill: "));
    u8g2.setFont(u8g2_font_squeezed_b7_tr);
    u8g2.print(remoteSessionFilled(), 2);
    u8g2.print('/');
    u8g2.print(remoteTargetQuantity, 2);
  } else
#endif
  {
    u8g2.print(F("Dnes: "));
    u8g2.setFont(u8g2_font_squeezed_b7_tr);
    u8g2.print(filled_today, 2);
    u8g2.print(F(" m3"));
  }

  u8g2.setFont(u8g2_font_squeezed_r7_tr);
  u8g2.setCursor(1, 26);
  u8g2.print(F("Pos:"));
  u8g2.setFont(u8g2_font_squeezed_b7_tr);
  u8g2.print(lastmess);

  u8g2.setFont(u8g2_font_squeezed_r7_tr);
  u8g2.setCursor(1, 34);
  u8g2.print(F("Min:"));
  u8g2.setFont(u8g2_font_squeezed_b7_tr);
  u8g2.print(minLevel ? F("ANO") : F("NIE"));
  u8g2.setFont(u8g2_font_squeezed_r7_tr);
  u8g2.print(F(" Max:"));
  u8g2.setFont(u8g2_font_squeezed_b7_tr);
  u8g2.print(maxLevel ? F("ANO") : F("NIE"));

  u8g2.setFont(u8g2_font_squeezed_r7_tr);
  u8g2.setCursor(1, 42);
  u8g2.print(F("Ochr:"));
  u8g2.setFont(u8g2_font_squeezed_b7_tr);
  u8g2.print(och_dis);

  u8g2.setFont(u8g2_font_squeezed_r7_tr);
  u8g2.setCursor(1, 50);
#if DEVICE_ROLE == ROLE_SERVER
  u8g2.print(F("Bat.kl:"));
  u8g2.setFont(u8g2_font_squeezed_b7_tr);
  u8g2.print(batteryRemoteV, 1);
#else
  u8g2.print(F("Bat:"));
  u8g2.setFont(u8g2_font_squeezed_b7_tr);
  u8g2.print(batteryLocalV, 1);
#endif
  u8g2.print(F("V"));

  u8g2.setFont(u8g2_font_squeezed_r7_tr);
  u8g2.setCursor(1, 58);
#if DEVICE_ROLE == ROLE_SERVER
  if (clientSensorFault) {
    u8g2.print(F("Hladina: PLAVAKY"));
  } else {
    u8g2.print(F("Naplneny "));
    u8g2.setFont(u8g2_font_squeezed_b7_tr);
    u8g2.print(filledper, 0);
    u8g2.print(F("%"));
  }
#elif DEVICE_ROLE == ROLE_CLIENT
  if (fallbackMode || sensorFault) {
    u8g2.print(F("Hladina: PLAVAKY"));
  } else {
    u8g2.print(F("Naplneny "));
    u8g2.setFont(u8g2_font_squeezed_b7_tr);
    u8g2.print(filledper, 0);
    u8g2.print(F("%"));
  }
#endif

  drawFillBar(115, 2, 10, 62, (filledper >= 0) ? (int)filledper : 0);
  u8g2.sendBuffer();
}

// =============================================================================
// SERVER-ONLY
// =============================================================================
#if DEVICE_ROLE == ROLE_SERVER

void pulseFlowCounter() {
  pulseCount++;
  filled_today = (float)pulseCount / 12000.0f;
}

void loadSettings() {
  settingsFile = SD.open("settings.txt");
  if (!settingsFile) {
    DEBUG_PRINTLN(F("settings.txt missing"));
    maxvol = 10.0f;
    fillinterval = 24;
    return;
  }
  while (settingsFile.available()) {
    char line[64];
    size_t n = settingsFile.fgets(line, sizeof(line));
    if (n == 0) break;
    if (strncmp(line, "maxvol", 6) == 0) {
      char* eq = strchr(line, '=');
      if (eq) maxvol = atof(eq + 1);
    } else if (strncmp(line, "fillinterval", 12) == 0) {
      char* eq = strchr(line, '=');
      if (eq) fillinterval = atoi(eq + 1);
    }
  }
  settingsFile.close();
  DEBUG_PRINT(F("maxvol=")); DEBUG_PRINTLN(maxvol);
}

void logWrite() {
  if (millis() - lastLogMs < LOG_INTERVAL_MS) return;
  lastLogMs = millis();
  if (!SD.exists("log.txt")) return;
  dataFile = SD.open("log.txt", FILE_WRITE);
  if (!dataFile) return;
  char line[80];
  snprintf(line, sizeof(line), "%02u/%02u/%02u,%02u:%02u:%02u,%.3f",
           rtc.getDay(), rtc.getMonth(), rtc.getYear(),
           rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(),
           filled_today);
  dataFile.println(line);
  dataFile.close();
}

void logRead() {
  if (!SD.exists("log.txt")) return;
  dataFile = SD.open("log.txt", FILE_READ);
  if (!dataFile) return;
  int fileSize = dataFile.size();
  if (fileSize <= 0) { dataFile.close(); return; }

  int pos = fileSize - 1;
  dataFile.seek(pos);
  while (pos > 0) {
    char ch = dataFile.peek();
    if (ch == '\n') break;
    pos--;
    dataFile.seek(pos);
  }
  char line[80];
  size_t n = dataFile.fgets(line, sizeof(line));
  dataFile.close();
  if (n == 0) return;

  // date,time,filled
  char* p1 = strchr(line, ',');
  if (!p1) return;
  char* p2 = strchr(p1 + 1, ',');
  if (!p2) return;
  filled_log = atof(p2 + 1);
  DEBUG_PRINT(F("log filled=")); DEBUG_PRINTLN(filled_log);
}

void readPress() {
  float sum = 0.0f;
  for (int i = 0; i < 8; i++) {
    int raw = analogRead(PRESS);
    float vout = raw * (3.3f / 1024.0f) * (1000.0f + 1800.0f) / 1800.0f;
    sum += vout;
    delayMicroseconds(500);
  }
  float vout_avg = sum / 8.0f;
  pressure = (vout_avg - 0.47f) / (4.5f - 0.47f) * 100.0f;
  if (pressure < 1.0f) press_low = true;
  else if (pressure > 1.5f) press_low = false;
}

void startPump() {
  DEBUG_PRINTLN(F("START PUMP"));
  pumpIsActive = true;
  digitalWrite(RELAY, HIGH);
  pumpStartTime = millis();
  lastRxMs = millis();
  errorMessage[0] = '\0';
}

void stopPump() {
  DEBUG_PRINTLN(F("STOP PUMP"));
  pumpIsActive = false;
  digitalWrite(RELAY, LOW);
  pumpStopTime = millis();
}

void clearRemoteMode() {
  remotePumpActive = false;
  remoteFillMode = false;
  remoteTargetQuantity = 0.0f;
}

float remoteSessionFilled() {
  float d = filled_today - remoteFilledStart;
  return (d > 0.0f) ? d : 0.0f;
}

void sendRemoteAck(uint8_t status) {
  lastRemoteStatus = status;
  RemoteAckPayload a;
  a.status = status;
  a.filledToday_x100 = protoFloatToU16x100(filled_today);
  a.filledSession_x100 = protoFloatToU16x100(remoteSessionFilled());
  a.pumpOn = pumpIsActive ? 1 : 0;
  sendProtoTo(EXTERNAL_ADDH, EXTERNAL_ADDL, MSG_REMOTE_ACK, 0,
              reinterpret_cast<uint8_t*>(&a), sizeof(a), false);
  DEBUG_PRINTF("REMOTE_ACK st=0x%02X sess=%.2f pump=%u\n",
               status, remoteSessionFilled(), a.pumpOn);
}

uint8_t handleRemoteCommand(uint8_t cmd, float volumeM3) {
  DEBUG_PRINTF("Remote cmd=%u vol=%.2f\n", cmd, volumeM3);

  if (cmd == REMOTE_CMD_STOP) {
    stopPump();
    clearRemoteMode();
    ignoreFilledToday = false;
    sendPumpCmd(true, false);
    return REMOTE_ST_STOPPED;
  }

  if (press_low) {
    return REMOTE_ST_ERR_PRESS;
  }
  if (maxLevel) {
    return REMOTE_ST_ERR_MAX;
  }

  if (cmd == REMOTE_CMD_START) {
    remotePumpActive = true;
    remoteFillMode = false;
    remoteTargetQuantity = 0.0f;
    remoteFilledStart = filled_today;
    remoteStartMs = millis();
    ignoreFilledToday = true;
    startPump();
    sendPumpCmd(true, true);
    return REMOTE_ST_STARTED;
  }

  if (cmd == REMOTE_CMD_FILL) {
    if (volumeM3 <= 0.0f || volumeM3 > 500.0f) {
      return REMOTE_ST_ERR_PARAM;
    }
    remotePumpActive = true;
    remoteFillMode = true;
    remoteTargetQuantity = volumeM3;
    remoteFilledStart = filled_today;
    remoteStartMs = millis();
    ignoreFilledToday = true;
    startPump();
    sendPumpCmd(true, true);
    DEBUG_PRINT(F("Remote FILL target m3="));
    DEBUG_PRINTLN(remoteTargetQuantity);
    return REMOTE_ST_FILLING;
  }

  return REMOTE_ST_ERR_PARAM;
}

void managePump() {
  // -------------------------------------------------------------------------
  // 1) Bezpečnosť VŽDY (aj počas remote): tlak + max plavák / hladina
  // -------------------------------------------------------------------------
  if (pumpIsActive && press_low) {
    stopPump();
    sendPumpCmd(true, false);
    if (remotePumpActive) {
      clearRemoteMode();
      ignoreFilledToday = false;
      sendRemoteAck(REMOTE_ST_ERR_PRESS);
    }
    return;
  }

  if (pumpIsActive && maxLevel) {
    stopPump();
    sendPumpCmd(true, true);
    if (remotePumpActive) {
      clearRemoteMode();
      ignoreFilledToday = false;
      sendRemoteAck(REMOTE_ST_ERR_MAX);
    }
    DEBUG_PRINTLN(F("STOP: maxLevel (sensor/float)"));
    return;
  }

  // -------------------------------------------------------------------------
  // 2) Client neodpovedá (vybitá batéria / timeout v loop)
  //    - AUTO start potlačený
  //    - remote môže bežať do 1h RX timeoutu (loop), stále platí max/tlak vyššie
  // -------------------------------------------------------------------------
  if (clientDisconnected) {
    if (!remotePumpActive) {
      if (pumpIsActive) {
        stopPump();
        sendPumpCmd(true, false);
      }
      return;
    }
    // remote + disconnected: iba FILL podľa prietoku na serveri, bez AUTO startu
    if (remotePumpActive && remoteFillMode && pumpIsActive) {
      if (remoteSessionFilled() >= remoteTargetQuantity) {
        stopPump();
        clearRemoteMode();
        ignoreFilledToday = false;
        sendPumpCmd(true, false);
        sendRemoteAck(REMOTE_ST_DONE);
      }
    }
    return;
  }

  if (!actReceived && !remotePumpActive) return;

  if (operationalSelected == OPERATION_DISABLED) {
    if (pumpIsActive) {
      stopPump();
      sendPumpCmd(true, false);
    }
    return;
  }

  // -------------------------------------------------------------------------
  // 3) Remote FILL: cieľový objem (bezpečnosť vyššie už spracovaná)
  // -------------------------------------------------------------------------
  if (remotePumpActive && remoteFillMode && pumpIsActive) {
    if (remoteSessionFilled() >= remoteTargetQuantity) {
      stopPump();
      clearRemoteMode();
      ignoreFilledToday = false;
      sendPumpCmd(true, true);
      sendRemoteAck(REMOTE_ST_DONE);
      DEBUG_PRINTLN(F("Remote FILL done"));
      return;
    }
  }

  // -------------------------------------------------------------------------
  // 4) AUTO logika (funguje aj popri remote, ak client žije)
  //    - stop pri dennom limitoch (ak nie ignore / nie remote)
  //    - start pri min a !max
  // -------------------------------------------------------------------------
  if (pumpIsActive && !remotePumpActive && filled_today >= maxvol && !ignoreFilledToday) {
    stopPump();
    sendPumpCmd(true, false);
    return;
  }

  if (!pumpIsActive && !maxLevel && minLevel && !press_low) {
    bool allowStart = false;
    if (remotePumpActive) {
      // remote chce čerpať; reštart po výpadku tlaku atď., kým nie je hotový fill
      if (remoteFillMode) {
        allowStart = remoteSessionFilled() < remoteTargetQuantity;
      } else {
        allowStart = true; // REMOTE START do STOP / max / 1h
      }
    } else {
      allowStart = (filled_today < maxvol) || ignoreFilledToday;
    }

    if (allowStart) {
      startPump();
      sendPumpCmd(true, true);
      DEBUG_PRINTLN(clientSensorFault ? F("START (float/fallback)") : F("START (auto)"));
    }
  }
}

void sendPumpCmd(bool wakeClient, bool ackReq) {
  PumpCmdPayload p;
  p.mode = (uint8_t)operationalSelected;
  p.filledToday_x100 = protoFloatToU16x100(filled_today);
  p.pressure_x100 = protoFloatToU16x100(pressure);
  p.bat_mV = protoVoltsToMv(batteryLocalV);

  uint8_t flags = 0;
  if (ackReq) flags |= FLAG_ACK_REQ;
  if (wakeClient) flags |= FLAG_WOR_WAKE;
  if (pumpIsActive) flags |= FLAG_PUMP_ON;
  if (ignoreFilledToday) flags |= FLAG_IGNORE_FTD;
  if (stavochrany) flags |= FLAG_OCHRANA_OK;

  sendProtoRetry(MSG_PUMP_CMD, flags, reinterpret_cast<uint8_t*>(&p), sizeof(p), wakeClient);
}

void sendTimeSync() {
  TimeSyncPayload t;
  t.year = rtc.getYear();
  t.month = rtc.getMonth();
  t.day = rtc.getDay();
  t.hour = rtc.getHours();
  t.minute = rtc.getMinutes();
  t.second = rtc.getSeconds();
  sendProto(MSG_TIME_SYNC, FLAG_ACK_REQ, reinterpret_cast<uint8_t*>(&t), sizeof(t), true);
}

#endif // ROLE_SERVER

// =============================================================================
// UART configuration menu (DebugSe 115200)
// =============================================================================
void uartPrintHelp() {
  DebugSe.println(F("=== UART MENU ==="));
  DebugSe.println(F("help"));
  DebugSe.println(F("time                 - show RTC"));
  DebugSe.println(F("settime YYYY-MM-DD HH:MM:SS"));
  DebugSe.println(F("status               - system status"));
#if DEVICE_ROLE == ROLE_SERVER
  DebugSe.println(F("synctime             - push RTC to client"));
  DebugSe.println(F("start                - remote START (timeout 1h)"));
  DebugSe.println(F("stop                 - remote STOP"));
  DebugSe.println(F("fill <m3>            - naplnit objem v m3, potom stop"));
#endif
  DebugSe.println(F("=================="));
}

void uartPrintTime() {
  char buf[32];
  snprintf(buf, sizeof(buf), "RTC %02u-%02u-%02u %02u:%02u:%02u",
           2000 + rtc.getYear(), rtc.getMonth(), rtc.getDay(),
           rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  DebugSe.println(buf);
}

bool uartSetDateTime(const char* args) {
  // Expected: YYYY-MM-DD HH:MM:SS
  int Y = 0, Mo = 0, D = 0, H = 0, Mi = 0, S = 0;
  if (sscanf(args, "%d-%d-%d %d:%d:%d", &Y, &Mo, &D, &H, &Mi, &S) != 6) {
    return false;
  }
  if (Y >= 2000) Y -= 2000;
  if (Mo < 1 || Mo > 12 || D < 1 || D > 31 || H > 23 || Mi > 59 || S > 59) {
    return false;
  }
  rtc.setYear((uint8_t)Y);
  rtc.setMonth((uint8_t)Mo);
  rtc.setDay((uint8_t)D);
  rtc.setHours((uint8_t)H);
  rtc.setMinutes((uint8_t)Mi);
  rtc.setSeconds((uint8_t)S);
  return true;
}

void uartPrintStatus() {
#if DEVICE_ROLE == ROLE_SERVER
  DebugSe.println(F("--- SERVER STATUS ---"));
  DebugSe.print(F("pump=")); DebugSe.println(pumpIsActive ? F("ON") : F("OFF"));
  DebugSe.print(F("remote=")); DebugSe.println(remotePumpActive ? F("YES") : F("NO"));
  if (remotePumpActive) {
    DebugSe.print(F("fillMode=")); DebugSe.println(remoteFillMode ? F("YES") : F("NO"));
    DebugSe.print(F("target_m3=")); DebugSe.println(remoteTargetQuantity, 2);
    DebugSe.print(F("session_m3=")); DebugSe.println(remoteSessionFilled(), 2);
  }
  DebugSe.print(F("filled_today=")); DebugSe.println(filled_today, 3);
  DebugSe.print(F("maxvol=")); DebugSe.println(maxvol, 2);
  DebugSe.print(F("pressure=")); DebugSe.println(pressure, 2);
  DebugSe.print(F("press_low=")); DebugSe.println(press_low ? F("YES") : F("NO"));
  DebugSe.print(F("min/max=")); DebugSe.print(minLevel); DebugSe.print('/'); DebugSe.println(maxLevel);
  DebugSe.print(F("sensorFault=")); DebugSe.println(clientSensorFault ? F("YES") : F("NO"));
  DebugSe.print(F("clientDisc=")); DebugSe.println(clientDisconnected ? F("YES") : F("NO"));
  DebugSe.print(F("bat_srv=")); DebugSe.println(batteryLocalV, 2);
  DebugSe.print(F("bat_cli=")); DebugSe.println(batteryRemoteV, 2);
  DebugSe.print(F("lastRemoteSt=0x")); DebugSe.println(lastRemoteStatus, HEX);
#else
  DebugSe.println(F("--- CLIENT STATUS ---"));
  DebugSe.print(F("pumpCmd=")); DebugSe.println(pumpCmdOn ? F("ON") : F("OFF"));
  DebugSe.print(F("filled%=")); DebugSe.println(filledper, 1);
  DebugSe.print(F("level_cm=")); DebugSe.println(levelInCM, 1);
  DebugSe.print(F("floatMax/Min="));
  DebugSe.print(digitalRead(TANK_MAX) == LOW ? 1 : 0);
  DebugSe.print('/');
  DebugSe.println(digitalRead(TANK_MIN) == LOW ? 1 : 0);
  DebugSe.print(F("min/max used=")); DebugSe.print(minLevel); DebugSe.print('/'); DebugSe.println(maxLevel);
  DebugSe.print(F("fallback3h=")); DebugSe.println(fallbackMode ? F("YES") : F("NO"));
  DebugSe.print(F("sensorFault=")); DebugSe.println(sensorFault ? F("YES") : F("NO"));
  DebugSe.print(F("bat=")); DebugSe.println(batteryLocalV, 2);
  DebugSe.print(F("WOR=")); DebugSe.println(inWorSleep ? F("YES") : F("NO"));
#endif
  uartPrintTime();
}

void processUartMenu() {
#ifdef TANK_DEBUG
  while (DebugSe.available() > 0) {
    char c = (char)DebugSe.read();
    if (c == '\r') continue;
    if (c == '\n') {
      uartLine[uartLineLen] = '\0';
      if (uartLineLen == 0) {
        uartLineLen = 0;
        continue;
      }

      // trim leading spaces
      char* cmd = uartLine;
      while (*cmd == ' ' || *cmd == '\t') cmd++;

      if (strcasecmp(cmd, "help") == 0 || strcasecmp(cmd, "?") == 0) {
        uartPrintHelp();
      } else if (strcasecmp(cmd, "time") == 0) {
        uartPrintTime();
      } else if (strncasecmp(cmd, "settime ", 8) == 0) {
        if (uartSetDateTime(cmd + 8)) {
          DebugSe.println(F("OK time set"));
          uartPrintTime();
#if DEVICE_ROLE == ROLE_SERVER
          // optional: also push to client
#endif
        } else {
          DebugSe.println(F("ERR use: settime YYYY-MM-DD HH:MM:SS"));
        }
      } else if (strcasecmp(cmd, "status") == 0) {
        uartPrintStatus();
#if DEVICE_ROLE == ROLE_SERVER
      } else if (strcasecmp(cmd, "synctime") == 0) {
        sendTimeSync();
        DebugSe.println(F("OK synctime sent"));
      } else if (strcasecmp(cmd, "start") == 0) {
        uint8_t st = handleRemoteCommand(REMOTE_CMD_START, 0.0f);
        sendRemoteAck(st);
        DebugSe.print(F("OK start st=0x")); DebugSe.println(st, HEX);
      } else if (strcasecmp(cmd, "stop") == 0) {
        uint8_t st = handleRemoteCommand(REMOTE_CMD_STOP, 0.0f);
        sendRemoteAck(st);
        DebugSe.print(F("OK stop st=0x")); DebugSe.println(st, HEX);
      } else if (strncasecmp(cmd, "fill ", 5) == 0) {
        float vol = atof(cmd + 5);
        uint8_t st = handleRemoteCommand(REMOTE_CMD_FILL, vol);
        sendRemoteAck(st);
        DebugSe.print(F("OK fill st=0x")); DebugSe.println(st, HEX);
#endif
      } else {
        DebugSe.println(F("ERR unknown cmd, type help"));
      }

      uartLineLen = 0;
    } else if (uartLineLen + 1 < UART_LINE_MAX) {
      uartLine[uartLineLen++] = c;
    } else {
      uartLineLen = 0; // overflow -> reset
      DebugSe.println(F("ERR line too long"));
    }
  }
#endif
}

// =============================================================================
// CLIENT-ONLY
// =============================================================================
#if DEVICE_ROLE == ROLE_CLIENT

void wakeToNormal() {
  if (inWorSleep) {
    setModeNormal();
    inWorSleep = false;
    delay(5);
  } else {
    setModeNormal();
  }
}

void enterWorSleepIfIdle() {
  if (pumpCmdOn || waitingAck || needAckFromServer) return;
  if (inWorSleep) return;
  setModeWorRx();
  inWorSleep = true;
  DEBUG_PRINTLN(F("WOR RX sleep"));
}

void waterLevel() {
  // Plaváky: INPUT_PULLUP, aktívne pri spojení na GND (LOW)
  const bool floatMax = (digitalRead(TANK_MAX) == LOW);
  const bool floatMin = (digitalRead(TANK_MIN) == LOW);

  uint8_t request[] = {0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xC5, 0xCB};
  while (LevelMeter.available()) LevelMeter.read();
  LevelMeter.write(request, sizeof(request));
  LevelMeter.flush();

  uint8_t response[7];
  size_t bytesRead = 0;
  unsigned long t0 = millis();
  while (bytesRead < 7 && (millis() - t0) < 120) {
    if (LevelMeter.available()) {
      bytesRead += LevelMeter.readBytes(response + bytesRead, 7 - bytesRead);
    }
  }

  bool ok = false;
  if (bytesRead == 7) {
    uint16_t receivedCrc = ((uint16_t)response[6] << 8) | response[5];
    uint16_t calculatedCrc = protoCrc16(response, 5); // Modbus CRC16 (bez FastCRC/pgmspace)
    if (receivedCrc == calculatedCrc) {
      int16_t signedValue = (int16_t)(((uint16_t)response[3] << 8) | response[4]);
      if (signedValue < 0) {
        levelInCM = 0.0f;
      } else {
        levelInCM = (float)signedValue;
      }
      // Nezmyselná hodnota mimo fyzického rozsahu nádrže
      const float maxSane = TANK_HEIGHT_CM + TANK_OVERFLOW_CM + 30.0f;
      if (levelInCM > maxSane) {
        ok = false;
        sensorFault = true;
        DEBUG_PRINTLN(F("Level out of range -> floats"));
      } else {
        ok = true;
      }
    }
  }

  if (!ok) {
    // Výpadok / CRC / nezmysel: výhradne plaváky
    sensorFault = true;
    fallbackMode = true;
    bool newMax = floatMax;
    bool newMin = floatMin;
    if (newMax != maxLevel || newMin != minLevel) changedLevels = true;
    maxLevel = newMax;
    minLevel = newMin;
    filledper = -1.0f;
    return;
  }

  // Detekcia zamrznutia senzora (3 h bez zmeny >= 1 cm)
  if (fabsf(levelInCM - lastLevelInCM) >= 1.0f) {
    lastLevelChangeMs = millis();
    lastLevelInCM = levelInCM;
    if (fallbackMode) DEBUG_PRINTLN(F("Level changed -> exit float fallback"));
    fallbackMode = false;
    sensorFault = false;
  } else if (millis() - lastLevelChangeMs > LEVEL_CHANGE_TIMEOUT) {
    if (!fallbackMode) DEBUG_PRINTLN(F("Level frozen 3h -> float priority"));
    fallbackMode = true;
    sensorFault = true;
  }

  if (levelInCM <= TANK_HEIGHT_CM) {
    filledper = (levelInCM / TANK_HEIGHT_CM) * 100.0f;
  } else if (levelInCM <= TANK_HEIGHT_CM + TANK_OVERFLOW_CM) {
    filledper = 100.0f + ((levelInCM - TANK_HEIGHT_CM) / TANK_OVERFLOW_CM) * 10.0f;
  } else {
    filledper = 110.0f;
  }

  bool newMax, newMin;
  if (fallbackMode) {
    // Priorita plavákov pri zamrznutom senzore
    newMax = floatMax;
    newMin = floatMin;
    filledper = -1.0f;
  } else {
    // Senzor + hard safety: MAX plavák vždy OR (nikdy neignorovať)
    newMax = (levelInCM >= TANK_HEIGHT_CM) || floatMax;
    newMin = (levelInCM <= TANK_MIN_CM) || floatMin;
  }

  if (newMax != maxLevel || newMin != minLevel) changedLevels = true;
  maxLevel = newMax;
  minLevel = newMin;
}

void sendTankStatus(bool ackReq) {
  getBatteryVoltage();
  waterLevel();

  TankStatusPayload p;
  p.levels = 0;
  if (maxLevel) p.levels |= 0x01;
  if (minLevel) p.levels |= 0x02;
  if (sensorFault || fallbackMode) p.levels |= 0x04; // bit2: fault/fallback
  float pct = (filledper >= 0) ? filledper : 0.0f;
  p.filledPct_x100 = protoFloatToU16x100(pct);
  p.bat_mV = protoVoltsToMv(batteryLocalV);

  uint8_t flags = ackReq ? FLAG_ACK_REQ : 0;
  if (ackReq) {
    needAckFromServer = true;
    ackStartMs = millis();
  }
  sendProtoRetry(MSG_TANK_STATUS, flags, reinterpret_cast<uint8_t*>(&p), sizeof(p), false);
}

void sendManual(bool ignore) {
  ManualPayload p;
  p.ignoreFtd = ignore ? 1 : 0;
  ignoreFilledToday = ignore;
  sendProto(MSG_MANUAL, FLAG_ACK_REQ, reinterpret_cast<uint8_t*>(&p), sizeof(p), false);
}

#endif // ROLE_CLIENT
