"""Simulácia server / klient / externe cez virtuálny LoRa bus."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from protocol import (
    CLIENT_ADDR,
    EXTERNAL_ADDR,
    FLAG_ACK_REQ,
    FLAG_OCHRANA_OK,
    FLAG_PUMP_ON,
    FLAG_WOR_WAKE,
    MSG_ACK,
    MSG_PUMP_CMD,
    MSG_REMOTE_ACK,
    MSG_REMOTE_CTRL,
    MSG_TANK_STATUS,
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
    SERVER_ADDR,
    build_frame,
    pack_pump_cmd,
    pack_remote_ack,
    pack_remote_ctrl,
    pack_tank_status,
    parse_frame,
    unpack_pump_cmd,
    unpack_remote_ack,
    unpack_remote_ctrl,
    unpack_tank_status,
)

Addr = Tuple[int, int]


@dataclass
class AirPacket:
    src: Addr
    dst: Addr
    frame: bytes


class LoRaBus:
    def __init__(self) -> None:
        self.queue: List[AirPacket] = []
        self.log: List[str] = []

    def send(self, src: Addr, dst: Addr, frame: bytes) -> None:
        assert len(frame) <= 64, f"oversize {len(frame)}"
        self.queue.append(AirPacket(src, dst, frame))
        self.log.append(f"AIR {src}->{dst} len={len(frame)} hex={frame.hex()}")

    def deliver_to(self, addr: Addr) -> List[bytes]:
        kept: List[AirPacket] = []
        out: List[bytes] = []
        for p in self.queue:
            if p.dst == addr:
                out.append(p.frame)
            else:
                kept.append(p)
        self.queue = kept
        return out


@dataclass
class ClientSim:
    bus: LoRaBus
    seq: int = 1
    min_level: bool = True
    max_level: bool = False
    sensor_fault: bool = False
    filled_pct: float = 40.0
    bat_v: float = 12.4
    pump_cmd_on: bool = False
    wor_sleep: bool = True
    last_pump: Optional[dict] = None

    def wake(self) -> None:
        self.wor_sleep = False

    def enter_wor(self) -> None:
        if not self.pump_cmd_on:
            self.wor_sleep = True

    def tick(self) -> None:
        for frame in self.bus.deliver_to(CLIENT_ADDR):
            self.wake()
            pkt = parse_frame(frame)
            if pkt["type"] == MSG_PUMP_CMD:
                p = unpack_pump_cmd(pkt["payload"])
                self.last_pump = p
                self.pump_cmd_on = bool(pkt["flags"] & FLAG_PUMP_ON)
                if pkt["flags"] & FLAG_ACK_REQ:
                    self._send_ack(pkt["seq"])
                self._send_tank(ack_req=False)
            elif pkt["type"] == MSG_ACK:
                pass
            self.enter_wor()

    def _next_seq(self) -> int:
        s = self.seq
        self.seq = 1 if self.seq == 255 else self.seq + 1
        return s

    def _send_ack(self, ack_seq: int) -> None:
        frame = build_frame(MSG_ACK, self._next_seq(), 0, bytes([ack_seq & 0xFF]))
        self.bus.send(CLIENT_ADDR, SERVER_ADDR, frame)

    def _send_tank(self, ack_req: bool) -> None:
        payload = pack_tank_status(
            self.max_level, self.min_level, self.sensor_fault, self.filled_pct, self.bat_v
        )
        flags = FLAG_ACK_REQ if ack_req else 0
        frame = build_frame(MSG_TANK_STATUS, self._next_seq(), flags, payload)
        self.bus.send(CLIENT_ADDR, SERVER_ADDR, frame)


@dataclass
class ServerSim:
    bus: LoRaBus
    seq: int = 1
    pump_on: bool = False
    filled_today: float = 0.0
    maxvol: float = 10.0
    pressure: float = 2.5
    press_low: bool = False
    bat_v: float = 13.1
    stavochrany: bool = True
    max_level: bool = False
    min_level: bool = False
    client_fault: bool = False
    client_disconnected: bool = True
    act_received: bool = False
    ignore_ftd: bool = False
    remote_active: bool = False
    remote_fill: bool = False
    remote_target: float = 0.0
    remote_start_filled: float = 0.0
    last_remote_status: int = 0
    external_acks: List[dict] = field(default_factory=list)

    def session_filled(self) -> float:
        d = self.filled_today - self.remote_start_filled
        return d if d > 0 else 0.0

    def _next_seq(self) -> int:
        s = self.seq
        self.seq = 1 if self.seq == 255 else self.seq + 1
        return s

    def send_pump_cmd(self, wake: bool = True, ack: bool = True) -> bytes:
        payload = pack_pump_cmd(0, self.filled_today, self.pressure, self.bat_v)
        flags = 0
        if ack:
            flags |= FLAG_ACK_REQ
        if wake:
            flags |= FLAG_WOR_WAKE
        if self.pump_on:
            flags |= FLAG_PUMP_ON
        if self.ignore_ftd:
            flags |= 0x08
        if self.stavochrany:
            flags |= FLAG_OCHRANA_OK
        frame = build_frame(MSG_PUMP_CMD, self._next_seq(), flags, payload)
        self.bus.send(SERVER_ADDR, CLIENT_ADDR, frame)
        return frame

    def handle_remote(self, cmd: int, volume_m3: float = 0.0) -> int:
        if cmd == REMOTE_CMD_STOP:
            self.pump_on = False
            self._clear_remote()
            self.ignore_ftd = False
            self.send_pump_cmd(wake=True, ack=False)
            st = REMOTE_ST_STOPPED
            self._send_remote_ack(st)
            return st
        if self.press_low:
            st = REMOTE_ST_ERR_PRESS
            self._send_remote_ack(st)
            return st
        if self.max_level:
            st = REMOTE_ST_ERR_MAX
            self._send_remote_ack(st)
            return st
        if cmd == REMOTE_CMD_START:
            self.remote_active = True
            self.remote_fill = False
            self.remote_target = 0.0
            self.remote_start_filled = self.filled_today
            self.ignore_ftd = True
            self.pump_on = True
            self.send_pump_cmd(wake=True, ack=True)
            st = REMOTE_ST_STARTED
            self._send_remote_ack(st)
            return st
        if cmd == REMOTE_CMD_FILL:
            if volume_m3 <= 0 or volume_m3 > 500:
                st = REMOTE_ST_ERR_PARAM
                self._send_remote_ack(st)
                return st
            self.remote_active = True
            self.remote_fill = True
            self.remote_target = volume_m3
            self.remote_start_filled = self.filled_today
            self.ignore_ftd = True
            self.pump_on = True
            self.send_pump_cmd(wake=True, ack=True)
            st = REMOTE_ST_FILLING
            self._send_remote_ack(st)
            return st
        st = REMOTE_ST_ERR_PARAM
        self._send_remote_ack(st)
        return st

    def _clear_remote(self) -> None:
        self.remote_active = False
        self.remote_fill = False
        self.remote_target = 0.0

    def _send_remote_ack(self, status: int) -> None:
        self.last_remote_status = status
        payload = pack_remote_ack(status, self.filled_today, self.session_filled(), self.pump_on)
        frame = build_frame(MSG_REMOTE_ACK, self._next_seq(), 0, payload)
        self.bus.send(SERVER_ADDR, EXTERNAL_ADDR, frame)

    def manage_pump(self) -> None:
        if self.pump_on and self.press_low:
            self.pump_on = False
            self.send_pump_cmd(False, False)
            if self.remote_active:
                self._clear_remote()
                self.ignore_ftd = False
                self._send_remote_ack(REMOTE_ST_ERR_PRESS)
            return
        if self.pump_on and self.max_level:
            self.pump_on = False
            self.send_pump_cmd(True, True)
            if self.remote_active:
                self._clear_remote()
                self.ignore_ftd = False
                self._send_remote_ack(REMOTE_ST_ERR_MAX)
            return
        if self.client_disconnected:
            if not self.remote_active:
                if self.pump_on:
                    self.pump_on = False
                    self.send_pump_cmd(False, False)
                return
            if self.remote_fill and self.pump_on and self.session_filled() >= self.remote_target:
                self.pump_on = False
                self._clear_remote()
                self.ignore_ftd = False
                self.send_pump_cmd(False, False)
                self._send_remote_ack(REMOTE_ST_DONE)
            return
        if not self.act_received and not self.remote_active:
            return
        if self.remote_active and self.remote_fill and self.pump_on:
            if self.session_filled() >= self.remote_target:
                self.pump_on = False
                self._clear_remote()
                self.ignore_ftd = False
                self.send_pump_cmd(True, True)
                self._send_remote_ack(REMOTE_ST_DONE)
                return
        if self.pump_on and not self.remote_active and self.filled_today >= self.maxvol and not self.ignore_ftd:
            self.pump_on = False
            self.send_pump_cmd(False, False)
            return
        if (not self.pump_on) and (not self.max_level) and self.min_level and (not self.press_low):
            allow = False
            if self.remote_active:
                if self.remote_fill:
                    allow = self.session_filled() < self.remote_target
                else:
                    allow = True
            else:
                allow = self.filled_today < self.maxvol or self.ignore_ftd
            if allow:
                self.pump_on = True
                self.send_pump_cmd(True, True)

    def tick(self) -> None:
        for frame in self.bus.deliver_to(SERVER_ADDR):
            pkt = parse_frame(frame)
            if pkt["type"] == MSG_TANK_STATUS:
                t = unpack_tank_status(pkt["payload"])
                self.max_level = t["max"]
                self.min_level = t["min"]
                self.client_fault = t["fault"]
                self.client_disconnected = False
                self.act_received = True
                if pkt["flags"] & FLAG_ACK_REQ:
                    ack = build_frame(MSG_ACK, self._next_seq(), 0, bytes([pkt["seq"]]))
                    self.bus.send(SERVER_ADDR, CLIENT_ADDR, ack)
                self.manage_pump()
            elif pkt["type"] == MSG_REMOTE_CTRL:
                r = unpack_remote_ctrl(pkt["payload"])
                self.handle_remote(r["cmd"], r["volume_m3"])
            elif pkt["type"] == MSG_ACK:
                pass
        for frame in self.bus.deliver_to(EXTERNAL_ADDR):
            pkt = parse_frame(frame)
            if pkt["type"] == MSG_REMOTE_ACK:
                self.external_acks.append(unpack_remote_ack(pkt["payload"]))


@dataclass
class ExternalSim:
    bus: LoRaBus
    seq: int = 1
    last_ack: Optional[dict] = None

    def send_remote(self, cmd: int, volume_m3: float = 0.0) -> bytes:
        payload = pack_remote_ctrl(cmd, volume_m3)
        frame = build_frame(MSG_REMOTE_CTRL, self.seq, FLAG_ACK_REQ, payload)
        self.seq = 1 if self.seq == 255 else self.seq + 1
        self.bus.send(EXTERNAL_ADDR, SERVER_ADDR, frame)
        return frame

    def tick(self) -> None:
        for frame in self.bus.deliver_to(EXTERNAL_ADDR):
            pkt = parse_frame(frame)
            if pkt["type"] == MSG_REMOTE_ACK:
                self.last_ack = unpack_remote_ack(pkt["payload"])


def exchange(server: ServerSim, client: ClientSim, rounds: int = 3) -> None:
    """Jednoduchý round-robin delivery."""
    for _ in range(rounds):
        server.tick()
        client.tick()
        server.tick()
