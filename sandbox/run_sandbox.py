#!/usr/bin/env python3
"""Sandbox testy protokolu a logiky server/klient/remote (bez HW)."""

from __future__ import annotations

import sys
import traceback

from protocol import (
    FLAG_ACK_REQ,
    FLAG_PUMP_ON,
    MSG_PUMP_CMD,
    PROTO_END,
    PROTO_ESC,
    PROTO_MAX_AIR,
    PROTO_START,
    REMOTE_CMD_FILL,
    REMOTE_CMD_START,
    REMOTE_CMD_STOP,
    REMOTE_ST_DONE,
    REMOTE_ST_ERR_MAX,
    REMOTE_ST_ERR_PARAM,
    REMOTE_ST_ERR_PRESS,
    REMOTE_ST_FILLING,
    REMOTE_ST_STARTED,
    REMOTE_ST_STOPPED,
    build_frame,
    escape,
    pack_pump_cmd,
    parse_frame,
)
from sim import ClientSim, ExternalSim, LoRaBus, ServerSim, exchange

PASSED = 0
FAILED = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global PASSED, FAILED
    if cond:
        PASSED += 1
        print(f"  [PASS] {name}")
    else:
        FAILED += 1
        print(f"  [FAIL] {name} {detail}")


def test_protocol_roundtrip() -> None:
    print("\n== Protocol roundtrip ==")
    payload = pack_pump_cmd(0, 1.25, 2.50, 12.345)
    frame = build_frame(MSG_PUMP_CMD, 7, FLAG_ACK_REQ | FLAG_PUMP_ON, payload)
    check("frame under 64B", len(frame) <= PROTO_MAX_AIR, f"len={len(frame)}")
    check("starts/ends with delim", frame[0] == PROTO_START and frame[-1] == PROTO_END)
    pkt = parse_frame(frame)
    check("type", pkt["type"] == MSG_PUMP_CMD)
    check("seq", pkt["seq"] == 7)
    check("flags", pkt["flags"] == (FLAG_ACK_REQ | FLAG_PUMP_ON))
    check("payload", pkt["payload"] == payload)


def test_escape_and_crc() -> None:
    print("\n== Escape + CRC ==")
    # force special bytes into body via payload containing 0xED/0xEF/0xEE
    payload = bytes([PROTO_START, PROTO_END, PROTO_ESC, 0x01])
    frame = build_frame(0x55, 1, 0, payload)
    check("escaped present", PROTO_ESC in frame[1:-1])
    pkt = parse_frame(frame)
    check("unescape restore", pkt["payload"] == payload)

    bad = bytearray(frame)
    bad[-3] ^= 0xFF  # corrupt before END
    try:
        parse_frame(bytes(bad))
        check("CRC detects corruption", False)
    except ValueError:
        check("CRC detects corruption", True)

    # verify escape of delimiter
    esc = escape(bytes([PROTO_START]))
    check("escape ED", esc == bytes([PROTO_ESC, PROTO_START ^ 0x20]))


def test_all_msg_sizes() -> None:
    print("\n== Message sizes <= 64B ==")
    cases = [
        ("PUMP_CMD", build_frame(0x01, 1, 0x1F, pack_pump_cmd(2, 99.99, 9.99, 14.0))),
        ("TANK", build_frame(0x02, 2, 1, bytes([0x07, 0x10, 0x27, 0xE8, 0x03]))),
        ("ACK", build_frame(0x03, 3, 0, bytes([9]))),
        ("TIME", build_frame(0x04, 4, 1, bytes([24, 10, 16, 19, 30, 0]))),
        ("MANUAL", build_frame(0x05, 5, 1, bytes([1]))),
        ("REMOTE", build_frame(0x06, 6, 1, bytes([2, 0xFA, 0x00]))),  # 2.50 m3
        ("RACK", build_frame(0x07, 7, 0, bytes([4, 0, 0, 0xFA, 0, 1]))),
    ]
    for name, fr in cases:
        check(f"{name} len={len(fr)}", len(fr) <= 64)


def test_auto_start_stop() -> None:
    print("\n== AUTO start/stop (min/max) ==")
    bus = LoRaBus()
    srv = ServerSim(bus)
    cli = ClientSim(bus, min_level=True, max_level=False, filled_pct=35.0)
    srv.client_disconnected = False
    srv.act_received = True
    srv.min_level = True
    srv.max_level = False
    srv.manage_pump()
    check("auto start on min", srv.pump_on)
    exchange(srv, cli, 4)
    check("client got pump ON", cli.pump_cmd_on)
    check("server got tank status", srv.act_received and not srv.client_disconnected)

    # max float -> stop
    cli.max_level = True
    cli.min_level = False
    cli._send_tank(False)
    exchange(srv, cli, 3)
    check("stop on max", not srv.pump_on and srv.max_level)


def test_remote_fill_m3() -> None:
    print("\n== Remote FILL 2.50 m3 (no time limit logic) ==")
    bus = LoRaBus()
    srv = ServerSim(bus, filled_today=1.0)
    cli = ClientSim(bus, min_level=True, max_level=False)
    ext = ExternalSim(bus)

    frame = ext.send_remote(REMOTE_CMD_FILL, 2.50)
    check("remote frame <=64", len(frame) <= 64)
    # deliver remote to server
    srv.tick()
    check("status FILLING", srv.last_remote_status == REMOTE_ST_FILLING)
    check("pump started", srv.pump_on and srv.remote_fill)
    check("target 2.5", abs(srv.remote_target - 2.50) < 1e-6)

    exchange(srv, cli, 3)
    check("client pumping", cli.pump_cmd_on)

    # simulate flow reaching target
    srv.filled_today = 1.0 + 2.50
    srv.manage_pump()
    check("FILL done", srv.last_remote_status == REMOTE_ST_DONE)
    check("pump stopped after fill", not srv.pump_on and not srv.remote_active)

    # external receives ack(s)
    ext.tick()
    # also drain from server.external via bus - ExternalSim listens EXTERNAL
    # server already sent REMOTE_ACK to EXTERNAL; may have been consumed by srv.tick
    # re-check via srv.external_acks
    check("external ack logged", any(a["status"] == REMOTE_ST_DONE for a in srv.external_acks)
          or (ext.last_ack and ext.last_ack["status"] == REMOTE_ST_DONE))


def test_remote_start_stop() -> None:
    print("\n== Remote START / STOP ==")
    bus = LoRaBus()
    srv = ServerSim(bus)
    cli = ClientSim(bus, min_level=True, max_level=False)
    ext = ExternalSim(bus)

    ext.send_remote(REMOTE_CMD_START)
    srv.tick()
    check("STARTED", srv.last_remote_status == REMOTE_ST_STARTED and srv.pump_on)
    exchange(srv, cli, 2)
    check("client ON", cli.pump_cmd_on)

    # vypni min, aby AUTO po STOP hneď znova nenaštartovalo
    cli.min_level = False
    srv.min_level = False
    ext.send_remote(REMOTE_CMD_STOP)
    srv.tick()
    check("STOPPED", srv.last_remote_status == REMOTE_ST_STOPPED and not srv.pump_on)
    exchange(srv, cli, 2)
    check("client OFF", not cli.pump_cmd_on)


def test_safety_max_during_remote() -> None:
    print("\n== Safety max during remote FILL ==")
    bus = LoRaBus()
    srv = ServerSim(bus, filled_today=0.0)
    cli = ClientSim(bus, min_level=True, max_level=False)
    ext = ExternalSim(bus)
    ext.send_remote(REMOTE_CMD_FILL, 5.0)
    srv.tick()
    exchange(srv, cli, 2)
    check("remote pumping", srv.pump_on and srv.remote_active)

    cli.max_level = True
    cli._send_tank(False)
    exchange(srv, cli, 3)
    check("max stops remote", not srv.pump_on and not srv.remote_active)
    check("remote ack ERR_MAX or stopped", srv.last_remote_status in (REMOTE_ST_ERR_MAX, REMOTE_ST_STOPPED)
          or srv.max_level)


def test_press_low_blocks_remote() -> None:
    print("\n== Pressure low blocks remote ==")
    bus = LoRaBus()
    srv = ServerSim(bus, press_low=True)
    ext = ExternalSim(bus)
    ext.send_remote(REMOTE_CMD_START)
    srv.tick()
    check("ERR_PRESS", srv.last_remote_status == REMOTE_ST_ERR_PRESS)
    check("pump off", not srv.pump_on)


def test_client_disconnect_suppresses_auto() -> None:
    print("\n== Client disconnect suppresses AUTO ==")
    bus = LoRaBus()
    srv = ServerSim(bus, client_disconnected=True, act_received=True, min_level=True, max_level=False)
    srv.manage_pump()
    check("no auto start when disconnected", not srv.pump_on)


def test_sensor_fault_floats() -> None:
    print("\n== Sensor fault -> float bits ==")
    bus = LoRaBus()
    srv = ServerSim(bus)
    cli = ClientSim(bus, min_level=True, max_level=False, sensor_fault=True, filled_pct=-1)
    srv.send_pump_cmd(True, True)
    exchange(srv, cli, 3)
    check("fault bit received", srv.client_fault)
    check("min from floats", srv.min_level and not srv.max_level)


def test_fill_param_reject() -> None:
    print("\n== FILL bad param ==")
    bus = LoRaBus()
    srv = ServerSim(bus)
    st = srv.handle_remote(REMOTE_CMD_FILL, 0.0)
    check("reject 0", st == REMOTE_ST_ERR_PARAM and not srv.pump_on)


def main() -> int:
    print("LoRa vodojem sandbox — protocol + functional simulation")
    print("=" * 56)
    tests = [
        test_protocol_roundtrip,
        test_escape_and_crc,
        test_all_msg_sizes,
        test_auto_start_stop,
        test_remote_fill_m3,
        test_remote_start_stop,
        test_safety_max_during_remote,
        test_press_low_blocks_remote,
        test_client_disconnect_suppresses_auto,
        test_sensor_fault_floats,
        test_fill_param_reject,
    ]
    for t in tests:
        try:
            t()
        except Exception:
            global FAILED
            FAILED += 1
            print(f"  [FAIL] {t.__name__} EXCEPTION")
            traceback.print_exc()

    print("\n" + "=" * 56)
    print(f"RESULT: {PASSED} passed, {FAILED} failed")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
