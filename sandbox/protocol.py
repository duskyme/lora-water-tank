"""Binárny LoRa protokol — port z include/protocol.h"""

from __future__ import annotations

PROTO_START = 0xED
PROTO_END = 0xEF
PROTO_ESC = 0xEE
PROTO_MAX_AIR = 64
PROTO_MAX_PAYLOAD = 16

MSG_PUMP_CMD = 0x01
MSG_TANK_STATUS = 0x02
MSG_ACK = 0x03
MSG_TIME_SYNC = 0x04
MSG_MANUAL = 0x05
MSG_REMOTE_CTRL = 0x06
MSG_REMOTE_ACK = 0x07

FLAG_ACK_REQ = 0x01
FLAG_WOR_WAKE = 0x02
FLAG_PUMP_ON = 0x04
FLAG_IGNORE_FTD = 0x08
FLAG_OCHRANA_OK = 0x10

REMOTE_CMD_STOP = 0x00
REMOTE_CMD_START = 0x01
REMOTE_CMD_FILL = 0x02

REMOTE_ST_OK = 0x00
REMOTE_ST_STARTED = 0x01
REMOTE_ST_STOPPED = 0x02
REMOTE_ST_FILLING = 0x03
REMOTE_ST_DONE = 0x04
REMOTE_ST_ERR_PRESS = 0x10
REMOTE_ST_ERR_MAX = 0x11
REMOTE_ST_ERR_PARAM = 0x12

SERVER_ADDR = (0x00, 0x01)
CLIENT_ADDR = (0x00, 0x02)
EXTERNAL_ADDR = (0x00, 0x10)
CHANNEL = 16


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def escape(data: bytes) -> bytes:
    out = bytearray()
    for b in data:
        if b in (PROTO_START, PROTO_END, PROTO_ESC):
            out.append(PROTO_ESC)
            out.append(b ^ 0x20)
        else:
            out.append(b)
    return bytes(out)


def unescape(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        if data[i] == PROTO_ESC:
            i += 1
            if i >= len(data):
                raise ValueError("truncated escape")
            out.append(data[i] ^ 0x20)
        else:
            out.append(data[i])
        i += 1
    return bytes(out)


def build_frame(msg_type: int, seq: int, flags: int, payload: bytes = b"") -> bytes:
    if len(payload) > PROTO_MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = bytes([msg_type & 0xFF, seq & 0xFF, flags & 0xFF]) + payload
    crc = crc16_modbus(body)
    raw = body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    esc = escape(raw)
    frame = bytes([PROTO_START]) + esc + bytes([PROTO_END])
    if len(frame) > PROTO_MAX_AIR:
        raise ValueError(f"frame {len(frame)} B exceeds {PROTO_MAX_AIR} B")
    return frame


def parse_frame(frame: bytes) -> dict:
    if len(frame) < 4 or frame[0] != PROTO_START or frame[-1] != PROTO_END:
        raise ValueError("bad delimiters")
    raw = unescape(frame[1:-1])
    if len(raw) < 5:
        raise ValueError("too short")
    body, crc_lo, crc_hi = raw[:-2], raw[-2], raw[-1]
    rx_crc = crc_lo | (crc_hi << 8)
    if crc16_modbus(body) != rx_crc:
        raise ValueError("CRC mismatch")
    return {
        "type": body[0],
        "seq": body[1],
        "flags": body[2],
        "payload": body[3:],
    }


def u16_le(v: int) -> bytes:
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def read_u16_le(b: bytes, i: int = 0) -> int:
    return b[i] | (b[i + 1] << 8)


def float_to_x100(v: float) -> int:
    return max(0, min(65535, int(v * 100 + 0.5)))


def x100_to_float(v: int) -> float:
    return v / 100.0


def volts_to_mv(v: float) -> int:
    return max(0, min(65535, int(v * 1000 + 0.5)))


def pack_pump_cmd(mode: int, filled_today: float, pressure: float, bat_v: float) -> bytes:
    return (
        bytes([mode & 0xFF])
        + u16_le(float_to_x100(filled_today))
        + u16_le(float_to_x100(pressure))
        + u16_le(volts_to_mv(bat_v))
    )


def unpack_pump_cmd(p: bytes) -> dict:
    return {
        "mode": p[0],
        "filled_today": x100_to_float(read_u16_le(p, 1)),
        "pressure": x100_to_float(read_u16_le(p, 3)),
        "bat_v": read_u16_le(p, 5) / 1000.0,
    }


def pack_tank_status(max_l: bool, min_l: bool, fault: bool, filled_pct: float, bat_v: float) -> bytes:
    levels = (0x01 if max_l else 0) | (0x02 if min_l else 0) | (0x04 if fault else 0)
    pct = 0.0 if filled_pct < 0 else filled_pct
    return bytes([levels]) + u16_le(float_to_x100(pct)) + u16_le(volts_to_mv(bat_v))


def unpack_tank_status(p: bytes) -> dict:
    return {
        "max": bool(p[0] & 0x01),
        "min": bool(p[0] & 0x02),
        "fault": bool(p[0] & 0x04),
        "filled_pct": x100_to_float(read_u16_le(p, 1)),
        "bat_v": read_u16_le(p, 3) / 1000.0,
    }


def pack_remote_ctrl(cmd: int, volume_m3: float = 0.0) -> bytes:
    return bytes([cmd & 0xFF]) + u16_le(float_to_x100(volume_m3))


def unpack_remote_ctrl(p: bytes) -> dict:
    return {"cmd": p[0], "volume_m3": x100_to_float(read_u16_le(p, 1))}


def pack_remote_ack(status: int, filled_today: float, session: float, pump_on: bool) -> bytes:
    return (
        bytes([status & 0xFF])
        + u16_le(float_to_x100(filled_today))
        + u16_le(float_to_x100(session))
        + bytes([1 if pump_on else 0])
    )


def unpack_remote_ack(p: bytes) -> dict:
    return {
        "status": p[0],
        "filled_today": x100_to_float(read_u16_le(p, 1)),
        "session": x100_to_float(read_u16_le(p, 3)),
        "pump_on": bool(p[5]),
    }
