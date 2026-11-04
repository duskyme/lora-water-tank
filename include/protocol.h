#ifndef INCLUDE_PROTOCOL_H_
#define INCLUDE_PROTOCOL_H_

#include <Arduino.h>
#include <string.h>

#define PROTO_START 0xED
#define PROTO_END   0xEF
#define PROTO_ESC   0xEE

// E220 sub-packet limit (SPS_064): max air payload ~64 B na jeden paket
#define PROTO_MAX_AIR     64
#define PROTO_MAX_PAYLOAD 16
#define PROTO_MAX_FRAME   PROTO_MAX_AIR

enum MsgType : uint8_t {
  MSG_PUMP_CMD    = 0x01,
  MSG_TANK_STATUS = 0x02,
  MSG_ACK         = 0x03,
  MSG_TIME_SYNC   = 0x04,
  MSG_MANUAL      = 0x05,
  MSG_REMOTE_CTRL = 0x06,  // third-party / external controller -> server
  MSG_REMOTE_ACK  = 0x07   // server -> third-party status reply
};

enum MsgFlags : uint8_t {
  FLAG_ACK_REQ    = 0x01,
  FLAG_WOR_WAKE   = 0x02,
  FLAG_PUMP_ON    = 0x04,
  FLAG_IGNORE_FTD = 0x08,
  FLAG_OCHRANA_OK = 0x10
};

// MSG_REMOTE_CTRL.cmd
enum RemoteCmd : uint8_t {
  REMOTE_CMD_STOP  = 0x00,
  REMOTE_CMD_START = 0x01,  // run until stop / max / 1h timeout
  REMOTE_CMD_FILL  = 0x02   // start and fill volume_x100 m3 then stop
};

// MSG_REMOTE_ACK.status
enum RemoteStatus : uint8_t {
  REMOTE_ST_OK       = 0x00,
  REMOTE_ST_STARTED  = 0x01,
  REMOTE_ST_STOPPED  = 0x02,
  REMOTE_ST_FILLING  = 0x03,
  REMOTE_ST_DONE     = 0x04,
  REMOTE_ST_ERR_PRESS = 0x10,
  REMOTE_ST_ERR_MAX  = 0x11,
  REMOTE_ST_ERR_PARAM = 0x12,
  REMOTE_ST_ERR_BUSY = 0x13
};

#pragma pack(push, 1)
struct PumpCmdPayload {
  uint8_t mode;
  uint16_t filledToday_x100;
  uint16_t pressure_x100;
  uint16_t bat_mV;
};

struct TankStatusPayload {
  uint8_t levels; // bit0 max, bit1 min, bit2 sensorFault/floatFallback
  uint16_t filledPct_x100;
  uint16_t bat_mV;
};

struct TimeSyncPayload {
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

struct ManualPayload {
  uint8_t ignoreFtd;
};

struct AckPayload {
  uint8_t ackSeq;
};

// Third-party command (LoRa -> server address)
struct RemoteCtrlPayload {
  uint8_t cmd;            // RemoteCmd
  uint16_t volume_x100;   // objem v m3 * 100 (napr. 2.50 m3 -> 250), len FILL
};

// Optional reply to third party (broadcast or their address if known)
struct RemoteAckPayload {
  uint8_t status;         // RemoteStatus
  uint16_t filledToday_x100;
  uint16_t filledSession_x100;
  uint8_t pumpOn;         // 0/1
};

struct ProtoPacket {
  uint8_t type;
  uint8_t seq;
  uint8_t flags;
  uint8_t payload[PROTO_MAX_PAYLOAD];
  uint8_t payloadLen;
};
#pragma pack(pop)

// CRC-16/IBM (Modbus-compatible poly 0xA001)
inline uint16_t protoCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

inline size_t protoEscape(const uint8_t* in, size_t inLen, uint8_t* out, size_t outMax) {
  size_t o = 0;
  for (size_t i = 0; i < inLen; i++) {
    uint8_t v = in[i];
    if (v == PROTO_START || v == PROTO_END || v == PROTO_ESC) {
      if (o + 2 > outMax) return 0;
      out[o++] = PROTO_ESC;
      out[o++] = v ^ 0x20;
    } else {
      if (o + 1 > outMax) return 0;
      out[o++] = v;
    }
  }
  return o;
}

inline size_t protoUnescape(const uint8_t* in, size_t inLen, uint8_t* out, size_t outMax) {
  size_t o = 0;
  for (size_t i = 0; i < inLen; i++) {
    if (in[i] == PROTO_ESC) {
      i++;
      if (i >= inLen || o >= outMax) return 0;
      out[o++] = in[i] ^ 0x20;
    } else {
      if (o >= outMax) return 0;
      out[o++] = in[i];
    }
  }
  return o;
}

inline size_t protoBuildFrame(const ProtoPacket& pkt, uint8_t* frame, size_t frameMax) {
  uint8_t raw[3 + PROTO_MAX_PAYLOAD + 2];
  if (pkt.payloadLen > PROTO_MAX_PAYLOAD) return 0;

  raw[0] = pkt.type;
  raw[1] = pkt.seq;
  raw[2] = pkt.flags;
  if (pkt.payloadLen) memcpy(raw + 3, pkt.payload, pkt.payloadLen);

  size_t bodyLen = 3 + pkt.payloadLen;
  uint16_t crc = protoCrc16(raw, bodyLen);
  raw[bodyLen] = (uint8_t)(crc & 0xFF);
  raw[bodyLen + 1] = (uint8_t)(crc >> 8);
  bodyLen += 2;

  uint8_t escaped[PROTO_MAX_FRAME - 2];
  size_t escLen = protoEscape(raw, bodyLen, escaped, sizeof(escaped));
  size_t total = escLen + 2; // START + END
  if (escLen == 0 || total > frameMax || total > PROTO_MAX_AIR) return 0;

  frame[0] = PROTO_START;
  memcpy(frame + 1, escaped, escLen);
  frame[1 + escLen] = PROTO_END;
  return total;
}

inline bool protoParseFrame(const uint8_t* data, size_t len, ProtoPacket& pkt) {
  if (len < 4 || data[0] != PROTO_START || data[len - 1] != PROTO_END) return false;

  uint8_t raw[3 + PROTO_MAX_PAYLOAD + 2];
  size_t rawLen = protoUnescape(data + 1, len - 2, raw, sizeof(raw));
  if (rawLen < 5) return false;

  size_t bodyLen = rawLen - 2;
  uint16_t rxCrc = (uint16_t)raw[bodyLen] | ((uint16_t)raw[bodyLen + 1] << 8);
  if (protoCrc16(raw, bodyLen) != rxCrc) return false;

  pkt.type = raw[0];
  pkt.seq = raw[1];
  pkt.flags = raw[2];
  pkt.payloadLen = (uint8_t)(bodyLen - 3);
  if (pkt.payloadLen > PROTO_MAX_PAYLOAD) return false;
  if (pkt.payloadLen) memcpy(pkt.payload, raw + 3, pkt.payloadLen);
  return true;
}

inline uint16_t protoFloatToU16x100(float v) {
  if (v < 0) v = 0;
  if (v > 655.35f) v = 655.35f;
  return (uint16_t)(v * 100.0f + 0.5f);
}

inline float protoU16x100ToFloat(uint16_t v) {
  return (float)v / 100.0f;
}

inline uint16_t protoVoltsToMv(float v) {
  if (v < 0) v = 0;
  if (v > 65.535f) v = 65.535f;
  return (uint16_t)(v * 1000.0f + 0.5f);
}

inline float protoMvToVolts(uint16_t mv) {
  return (float)mv / 1000.0f;
}

#endif /* INCLUDE_PROTOCOL_H_ */
