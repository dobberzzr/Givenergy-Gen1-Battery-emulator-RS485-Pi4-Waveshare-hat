#!/usr/bin/env python3
"""
DATOUBOSS (JBD BMS) -> GivEnergy Gen 1 BMS gateway for the Pi 4 + Waveshare
RS485 CAN HAT (B).  Replaces the ESP32 emulator and the Pi Zero MQTT bridge.

  battery  RS485 (/dev/ttySC0)  --JBD native 0x03/0x04, read-only-->  this program
  inverter RS485 (/dev/ttySC1)  <--Modbus RTU slave 1, 9600 8N1-----  this program
  Home Assistant                <--MQTT, same topics/entities as jbd_mqtt.py

Two threads that only share a snapshot:
  * battery thread  polls the BMS, publishes MQTT, updates the snapshot.
  * inverter thread answers the Gen 1's polls from the snapshot.  It NEVER
    waits on the battery, so a slow or dead BMS cannot stall a reply.

Everything the inverter is told is derived from the snapshot and its age at the
moment of the reply.  Fail-safe, same two stages as the ESP32 firmware:
  stage 1, no good BMS read for 5 s : HR25 = 0 (stop current flow)
  stage 2, no good BMS read for 20 s: also HR20 bits 2+3 ("full" + "empty")
Both are well inside the ~30 s the inverter tolerates.  Stale values are never
presented as live: past stage 1 the current limit is zero.

Replies match the working ESP32 firmware (src/main.cpp) byte for byte wherever
the firmware used a fixed value; only fields that were fixed or faked there now
carry live data.  Needs jbd_mqtt.py in the same directory.

  Self-test, no hardware:   python3 giv_bridge.py --selftest
  Normal run (systemd):     python3 giv_bridge.py
"""
import argparse
import json
import logging
import os
import signal
import sys
import threading
import time

import serial

import jbd_mqtt
from jbd_mqtt import BMSError, Bridge, PROTECTION_BITS, load_config, read_bms

log = logging.getLogger("giv-bridge")

CELLS = 16
RESPONSE_DELAY_S = 0.05      # same as the firmware; inverter allows ~100 ms
STAGE1_S = 5.0
STAGE2_S = 20.0

# JBD protection word bit (jbd_mqtt.PROTECTION_BITS order) -> HR20 bit mask.
# HR20: 0 over-current, 1 short, 2 over-voltage ("full"), 3 under-voltage
# ("empty"), 4 discharge over-temp, 5 charge over-temp, 6 discharge
# under-temp, 7 charge under-temp.
PROT_TO_HR20 = {
    "cell_overvoltage": 1 << 2,
    "cell_undervoltage": 1 << 3,
    "pack_overvoltage": 1 << 2,
    "pack_undervoltage": 1 << 3,
    "charge_overtemp": 1 << 5,
    "charge_undertemp": 1 << 7,
    "discharge_overtemp": 1 << 4,
    "discharge_undertemp": 1 << 6,
    "charge_overcurrent": 1 << 0,
    "discharge_overcurrent": 1 << 0,
    "short_circuit": 1 << 1,
    "frontend_ic_error": (1 << 2) | (1 << 3),   # unknown fault: stop both
    "mos_software_lock": 0,                      # handled through the FET state
}


# --------------------------------------------------------------------------
# Tunables.  Every value marked ESTIMATE is mine, not from a datasheet or a
# capture: confirm or change it in the config file before relying on it.
# --------------------------------------------------------------------------
class Params:
    DEFAULTS = {
        # HR25 while everything is healthy, in amps.  The Gen 1 tops out near
        # 50 A and the pack is 157 A continuous.                    ESTIMATE
        "limit_a": 100.0,
        # Pack thresholds from the BMS's own screen (2026-10-22): max charge
        # voltage 56.8 V, max discharge voltage 44.8 V.  Recovery points are
        # 3.45 V/cell for over-voltage and the spec sheet's 46.4 V for under.
        # The BMS's hard trips (58.4 V / 43.2 V) sit beyond these.
        "pack_ov_trip_v": 56.8, "pack_ov_recover_v": 55.2,
        "pack_uv_trip_v": 44.8, "pack_uv_recover_v": 46.4,
        # Per-cell backstops, ahead of the BMS's own trip.            ESTIMATE
        "cell_ov_trip_v": 3.55, "cell_ov_recover_v": 3.45,
        "cell_uv_trip_v": 2.90, "cell_uv_recover_v": 3.05,
        # No charging below 0 C, resumes at 5 C (battery spec).
        "charge_ut_trip_c": 0.0, "charge_ut_recover_c": 5.0,
    }

    def __init__(self, cfg=None):
        for k, d in self.DEFAULTS.items():
            v = d
            if cfg is not None and k in cfg:
                v = type(d)(cfg[k])
            setattr(self, k, v)


class Latch:
    """Trips at one level, clears only at another, so it cannot chatter."""

    def __init__(self, trip, recover, high):
        self.trip, self.recover, self.high, self.on = trip, recover, high, False

    def update(self, v):
        if self.high:
            if v >= self.trip:
                self.on = True
            elif v <= self.recover:
                self.on = False
        else:
            if v <= self.trip:
                self.on = True
            elif v >= self.recover:
                self.on = False
        return self.on


class Latches:
    def __init__(self, p):
        self.cell_ov = Latch(p.cell_ov_trip_v, p.cell_ov_recover_v, True)
        self.cell_uv = Latch(p.cell_uv_trip_v, p.cell_uv_recover_v, False)
        self.pack_ov = Latch(p.pack_ov_trip_v, p.pack_ov_recover_v, True)
        self.pack_uv = Latch(p.pack_uv_trip_v, p.pack_uv_recover_v, False)
        self.charge_ut = Latch(p.charge_ut_trip_c, p.charge_ut_recover_c, False)

    def update(self, snap):
        self.cell_ov.update(snap["cell_max_v"])
        self.cell_uv.update(snap["cell_min_v"])
        self.pack_ov.update(snap["voltage"])
        self.pack_uv.update(snap["voltage"])
        self.charge_ut.update(snap["temp_min"])


# --------------------------------------------------------------------------
# Snapshot and derived registers
# --------------------------------------------------------------------------
def extract_snapshot(s):
    """Pull what the inverter side needs out of jbd_mqtt.read_bms()'s dict."""
    cells = [s.get(f"cell_{i:02d}") for i in range(1, CELLS + 1)]
    if any(c is None for c in cells) or s.get("cell_17") is not None:
        raise BMSError("expected exactly 16 cells")
    temps = [s[k] for k in sorted(s) if k.startswith("temp_") and k[5:].isdigit()]
    if not temps:
        raise BMSError("no temperature sensors reported")
    cells_mv = [int(round(c * 1000)) for c in cells]
    return {
        "voltage": s["voltage"],
        "current": s["current"],            # + = charging, same as HR23
        "soc": s["soc"],
        "nominal_ah": s["nominal_ah"],
        "remaining_ah": s["remaining_ah"],
        "cells_mv": cells_mv,
        "cell_min_v": min(cells_mv) / 1000.0,
        "cell_max_v": max(cells_mv) / 1000.0,
        "temps": temps,
        "temp_min": min(temps),
        "temp_max": max(temps),
        "temp_avg": sum(temps) / len(temps),
        "charge_fet": s["charge_fet"] == "ON",
        "discharge_fet": s["discharge_fet"] == "ON",
        "prot": s["protection_raw"],
    }


def s16(v):
    return int(v) & 0xFFFF


def derive(snap, latches, age, p):
    """Everything the inverter is told, for a snapshot that is `age` s old.
    snap may be None (nothing read yet)."""
    stage = 2 if age >= STAGE2_S else 1 if age >= STAGE1_S else 0
    if snap is None:
        return {"stage": max(stage, 1), "hr20": (0x0C if stage >= 2 else 0),
                "limit": 0, "soc": 0, "real_soc": 0, "capacity_ah": 0,
                "v_mv": 0, "i_centi": 0,
                "cells_mv": [0] * CELLS, "cell_sum": 0, "nominal_raw": 0,
                "remaining_raw": 0, "t_avg10": 0, "t_max10": 0, "t_min10": 0,
                "t_whole": 0, "reasons": ["no BMS data yet"]}

    reasons = []
    hr20 = 0
    for bit, (key, _) in enumerate(PROTECTION_BITS):
        if snap["prot"] & (1 << bit):
            hr20 |= PROT_TO_HR20[key]
            reasons.append(f"BMS {key}")
    if latches.cell_ov.on or latches.pack_ov.on:
        hr20 |= 1 << 2
        reasons.append("over-voltage latch")
    if latches.cell_uv.on or latches.pack_uv.on:
        hr20 |= 1 << 3
        reasons.append("under-voltage latch")
    if latches.charge_ut.on:
        hr20 |= 1 << 7
        reasons.append("charge under-temp latch")
    if not snap["charge_fet"]:
        hr20 |= 1 << 2
        reasons.append("charge FET off")
    if not snap["discharge_fet"]:
        hr20 |= 1 << 3
        reasons.append("discharge FET off")

    limit = int(round(p.limit_a * 100))
    if hr20 or snap["prot"]:
        limit = 0
    if stage >= 1:
        limit = 0
        reasons.append(f"BMS data {age:.0f} s old")
    if stage >= 2:
        hr20 |= (1 << 2) | (1 << 3)

    soc = max(0, min(100, int(snap["soc"])))     # the BMS's own SoC, unscaled
    nominal_raw = int(round(snap["nominal_ah"] * 100))
    return {
        "stage": stage, "hr20": hr20, "limit": min(limit, 0xFFFF),
        "soc": soc, "real_soc": snap["soc"],
        "capacity_ah": int(round(snap["nominal_ah"])),
        "v_mv": int(round(snap["voltage"] * 1000)),
        "i_centi": int(round(snap["current"] * 100)),
        "cells_mv": snap["cells_mv"], "cell_sum": sum(snap["cells_mv"]),
        "nominal_raw": nominal_raw,
        "remaining_raw": int(round(snap["remaining_ah"] * 100)),
        "t_avg10": int(round(snap["temp_avg"] * 10)),
        "t_max10": int(round(snap["temp_max"] * 10)),
        "t_min10": int(round(snap["temp_min"] * 10)),
        "t_whole": int(round(snap["temp_avg"])),
        "reasons": reasons,
    }


# --------------------------------------------------------------------------
# Modbus framing - ported from the ESP32 firmware
# --------------------------------------------------------------------------
def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(frame):
    c = crc16(frame)
    return bytes(frame) + bytes([c & 0xFF, c >> 8])


def crc_ok(frame):
    return len(frame) >= 4 and crc16(frame[:-2]) == frame[-2] | (frame[-1] << 8)


def be16(v):
    return bytes([(v >> 8) & 0xFF, v & 0xFF])


def be32(v):
    return bytes([(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


def holding_regs(d):
    hr = [0] * 28
    hr[0] = 0x0065
    hr[1] = hr[2] = hr[3] = hr[4] = 0xFFFF
    ser = b"BG1234G567"
    for i in range(5):
        hr[5 + i] = (ser[2 * i] << 8) | ser[2 * i + 1]
    hr[10] = 0xFFFF
    hr[11] = d["capacity_ah"]        # the BMS's real capacity, Ah
    hr[12] = 48
    hr[13] = 3022
    hr[17] = 0x114B          # clock hash: fixed, as in the working firmware
    hr[18] = 0x389D
    hr[19] = 0x00CE          # status: fixed, as in the working firmware
    hr[20] = d["hr20"]
    hr[21] = d["soc"]
    hr[22] = s16(d["v_mv"] // 10)
    hr[23] = s16(d["i_centi"])
    hr[24] = s16(d["t_whole"])
    hr[25] = d["limit"]
    return hr                # HR26/HR27 stay 0 (Gen 3 only, never read)


def block1(d):
    b = bytearray(42)
    b[0:20] = b"EM20260001" + b" " * 10
    for i in range(5):
        b[22 + 2 * i:24 + 2 * i] = be16(s16(d["t_avg10"]))
    b[32:34] = b"\x00\x01"
    b[34:36] = b"\x00\x08"
    return bytes(b)


def block2(d):
    b = bytearray([
        0x10, 0x00, 0x00, 0x00, 0x00, 0xCB, 0x20, 0xCB, 0x20,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x10, 0x00, 0x00,
        0x27, 0x10, 0x00, 0x00, 0x13, 0x88, 0x32, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0xCE, 0x00])
    b[5:7] = be16(min(d["v_mv"], 0xFFFF))        # pack mV
    b[7:9] = be16(min(d["cell_sum"], 0xFFFF))    # cell-sum mV
    b[13:17] = be32(d["nominal_raw"])            # full capacity, 0.01 Ah
    b[17:21] = be32(d["nominal_raw"])            # design capacity
    b[21:25] = be32(d["remaining_raw"])          # remaining capacity
    b[25] = d["soc"]
    return bytes(b)


def block3(d):
    b = bytearray(40)
    for i, mv in enumerate(d["cells_mv"]):
        b[2 * i:2 * i + 2] = be16(mv)
    b[32:34] = be16(s16(d["t_max10"]))
    b[34:36] = be16(s16(d["t_min10"]))
    b[36:38] = be16(max(d["cells_mv"]))          # highest cell
    b[38:40] = be16(min(d["cells_mv"]))          # lowest cell
    return bytes(b)


def absent_block(n):
    b = bytearray(n)
    if n == 42:
        for i in range(5):
            b[22 + 2 * i:24 + 2 * i] = b"\xF5\x56"
    elif n == 40:
        b[32:36] = b"\xF5\x56\xF5\x56"
    return bytes(b)


FC04_BLOCKS = {(0x0000, 21): (42, block1), (0x0015, 19): (38, block2),
               (0x0028, 20): (40, block3)}


def exception(dev, fc, code):
    return with_crc(bytes([dev, fc | 0x80, code]))


def build_reply(req, d, p):
    """req: a CRC-checked 8-byte request.  Returns the full reply or None."""
    dev, fc = req[0], req[1]
    start = (req[2] << 8) | req[3]
    count = (req[4] << 8) | req[5]

    if fc == 0x03:
        if dev != 1:
            return None
        if count == 0 or start >= 28 or start + count > 28:
            return exception(dev, fc, 0x02)
        hr = holding_regs(d)
        body = b"".join(be16(hr[start + i]) for i in range(count))
        return with_crc(bytes([dev, 0x03, count * 2]) + body)

    if fc == 0x04:
        if not 1 <= dev <= 5:
            return None
        blk = FC04_BLOCKS.get((start, count))
        if blk is None:
            return exception(dev, fc, 0x02)
        n, builder = blk
        payload = builder(d) if dev == 1 else absent_block(n)
        # GivEnergy quirk: the start address sits where Modbus has a byte count
        return with_crc(bytes([dev, 0x04]) + be16(start) + payload)

    if fc == 0x06:
        return bytes(req)                         # echoed unchanged, as before

    if 1 <= dev <= 5:
        return exception(dev, fc, 0x01)
    return None


# --------------------------------------------------------------------------
# Home Assistant: what the inverter is being told
# --------------------------------------------------------------------------
def inverter_discovery(dev_id, prefix, device_name, expire_after):
    """Extra HA entities, on the same device as jbd_mqtt's, fed from <prefix>/inverter."""
    common = {
        "state_topic": f"{prefix}/inverter",
        "availability_topic": f"{prefix}/availability",
        "expire_after": expire_after,
        "device": {"identifiers": [dev_id], "name": device_name,
                   "manufacturer": "DATOUBOSS / JBD", "model": "LiFePO4 48V BMS"},
    }
    # key, name, unit, device_class, state_class, icon
    sensors = [
        ("hr25_a", "Inverter current limit (HR25)", "A", "current", "measurement", None),
        ("hr21_soc", "SOC told to inverter (HR21)", "%", "battery", "measurement", None),
        ("hr20", "Inverter alarm register (HR20)", None, None, None, "mdi:alert-octagon-outline"),
        ("stage", "Fail-safe stage", None, None, None, "mdi:shield-half-full"),
        ("data_age_s", "BMS data age", "s", "duration", "measurement", None),
        ("poll_age_s", "Since inverter last polled", "s", "duration", "measurement", None),
        ("replies", "Inverter replies sent", None, None, "total_increasing", "mdi:counter"),
        ("reasons", "Why the inverter is limited", None, None, None, "mdi:text-box-outline"),
    ]
    msgs = []
    for key, name, unit, dclass, sclass, icon in sensors:
        p = dict(common, name=name, unique_id=f"{dev_id}_inv_{key}",
                 value_template=f"{{{{ value_json.{key} }}}}")
        for k, v in (("unit_of_measurement", unit), ("device_class", dclass),
                     ("state_class", sclass), ("icon", icon)):
            if v:
                p[k] = v
        msgs.append((f"homeassistant/sensor/{dev_id}/inv_{key}/config", p))
    binaries = [
        ("polling", "Inverter polling", "connectivity",
         "{{ 'ON' if value_json.poll_age_s is not none and value_json.poll_age_s < 30 else 'OFF' }}"),
        ("limited", "Inverter limited", "problem",
         "{{ 'ON' if value_json.hr25_a == 0 else 'OFF' }}"),
    ]
    for key, name, dclass, template in binaries:
        p = dict(common, name=name, unique_id=f"{dev_id}_inv_{key}", device_class=dclass,
                 value_template=template, payload_on="ON", payload_off="OFF")
        msgs.append((f"homeassistant/binary_sensor/{dev_id}/inv_{key}/config", p))
    return msgs


# --------------------------------------------------------------------------
# Gateway
# --------------------------------------------------------------------------
def opt(cfg, name, default):
    return type(default)(cfg[name]) if name in cfg else default


class Gateway(Bridge):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.p = Params(cfg)
        self.latches = Latches(self.p)
        self.lock = threading.Lock()
        self.snap = None
        self.t_ok = time.monotonic()       # start counts as "last good" so the
        self.battery_port = opt(cfg, "battery_port", "/dev/ttySC0")    # stages
        self.inverter_port = opt(cfg, "inverter_port", "/dev/ttySC1")  # time
        self.poll = opt(cfg, "gw_poll_interval", 1.0)                  # boot
        self.mqtt_every = opt(cfg, "gw_mqtt_interval", 5.0)
        self.stats = dict(fc03=0, fc04=0, fc06=0, other=0, reads=0, read_fail=0)
        self.last_told = None
        self.last_poll = None              # monotonic time of the inverter's last poll
        self.replies = 0

    def open_port(self):
        return serial.Serial(self.battery_port, self.cfg.getint("baud"),
                             bytesize=8, parity="N", stopbits=1, timeout=0.1)

    # -- battery side ---------------------------------------------------------
    def battery_loop(self):
        port, fails, last_pub = None, 0, 0.0
        while self.running:
            t0 = time.monotonic()
            try:
                if port is None:
                    port = self.open_port()
                    log.info("battery: %s @ %s", self.battery_port, self.cfg["baud"])
                s = read_bms(port)
                snap = extract_snapshot(s)
                with self.lock:
                    self.snap = snap
                    self.t_ok = time.monotonic()
                    self.latches.update(snap)
                self.last_state = s
                self.stats["reads"] += 1
                if fails:
                    log.info("BMS reads recovered after %d failure(s)", fails)
                fails = 0
                if self.connected:
                    if not self.discovery_sent:
                        self.send_discovery()
                    if t0 - last_pub >= self.mqtt_every:
                        last_pub = t0
                        self.client.publish(f"{self.prefix}/state",
                                            json.dumps(s), qos=0)
            except serial.SerialException as e:
                fails += 1
                self.stats["read_fail"] += 1
                log.warning("battery serial error: %s - reopening", e)
                try:
                    if port:
                        port.close()
                except Exception:
                    pass
                port = None
            except BMSError as e:
                fails += 1
                self.stats["read_fail"] += 1
                log.warning("bad BMS read (%d in a row): %s", fails, e)
            time.sleep(max(0.0, self.poll - (time.monotonic() - t0)))

    def current(self):
        with self.lock:
            snap, age = self.snap, time.monotonic() - self.t_ok
            return derive(snap, self.latches, age, self.p), age

    def send_discovery(self):
        super().send_discovery()
        msgs = inverter_discovery(self.cfg["device_id"], self.prefix,
                                  self.cfg["device_name"], self.cfg.getint("expire_after"))
        for topic, payload in msgs:
            self.client.publish(topic, json.dumps(payload), qos=1, retain=True)
        log.info("published %d inverter-side HA entities", len(msgs))

    def publish_inverter_view(self):
        d, age = self.current()
        lp = self.last_poll
        view = {"hr20": d["hr20"], "hr25_a": d["limit"] / 100.0, "hr21_soc": d["soc"],
                "real_soc": d["real_soc"], "stage": d["stage"],
                "data_age_s": round(age, 1),
                "poll_age_s": None if lp is None else round(time.monotonic() - lp, 1),
                "replies": self.replies,
                "reasons": ", ".join(d["reasons"]) or "none - healthy"}
        self.client.publish(f"{self.prefix}/inverter", json.dumps(view), qos=0)

    # -- inverter side --------------------------------------------------------
    def inverter_loop(self):
        port, buf, last_rx = None, bytearray(), 0.0
        while self.running:
            try:
                if port is None:
                    port = serial.Serial(self.inverter_port, self.cfg.getint("baud"),
                                         bytesize=8, parity="N", stopbits=1,
                                         timeout=0.02)
                    log.info("inverter: %s @ %s", self.inverter_port, self.cfg["baud"])
                data = port.read(port.in_waiting or 1)
                now = time.monotonic()
                if buf and now - last_rx > 0.1:
                    buf.clear()                      # stale partial frame
                if not data:
                    continue
                buf += data
                last_rx = now
                while len(buf) >= 8:
                    f = bytes(buf[:8])
                    if f[0] in (1, 2, 3, 4, 5) and f[1] in (3, 4, 6) and crc_ok(f):
                        del buf[:8]
                        self.answer(port, f, last_rx)
                        port.reset_input_buffer()    # drop any echo of our reply
                        buf.clear()
                    else:
                        del buf[0]                   # resync one byte at a time
            except serial.SerialException as e:
                log.warning("inverter serial error: %s - reopening in 2 s", e)
                try:
                    if port:
                        port.close()
                except Exception:
                    pass
                port, buf = None, bytearray()
                time.sleep(2)

    def answer(self, port, req, t_rx):
        key = {3: "fc03", 4: "fc04", 6: "fc06"}[req[1]]
        self.stats[key] += 1
        self.last_poll = time.monotonic()
        d, _ = self.current()
        reply = build_reply(req, d, self.p)
        self.note_change(d)
        if reply is None:
            return
        time.sleep(max(0.0, RESPONSE_DELAY_S - (time.monotonic() - t_rx)))
        port.write(reply)
        port.flush()
        self.replies += 1
        log.debug("rx %s tx %d bytes", req.hex(" "), len(reply))

    def note_change(self, d):
        told = (d["hr20"], d["limit"] == 0, d["stage"])
        if told != self.last_told:
            self.last_told = told
            log.info("inverter told: HR20=0x%02X HR25=%.1f A HR21=%d%% (real %s%%) "
                     "stage %d %s", d["hr20"], d["limit"] / 100.0, d["soc"],
                     d["real_soc"], d["stage"],
                     "[" + ", ".join(d["reasons"]) + "]" if d["reasons"] else "")

    # -- lifecycle ------------------------------------------------------------
    def run(self):
        self.client.connect_async(self.cfg["mqtt_host"], self.cfg.getint("mqtt_port"), 60)
        self.client.loop_start()
        threads = [threading.Thread(target=self.battery_loop, name="battery", daemon=True),
                   threading.Thread(target=self.inverter_loop, name="inverter", daemon=True)]
        for t in threads:
            t.start()
        last_stats = last_view = time.monotonic()
        while self.running:
            time.sleep(0.5)
            if self.connected and time.monotonic() - last_view >= self.mqtt_every:
                last_view = time.monotonic()
                self.publish_inverter_view()
            if time.monotonic() - last_stats >= 300:
                last_stats = time.monotonic()
                log.info("stats %s", self.stats)
        self.shutdown()


# --------------------------------------------------------------------------
# Self-test
# --------------------------------------------------------------------------
def selftest():
    p = Params()
    fake = {"voltage": 53.51, "current": 34.04, "soc": 47, "nominal_ah": 314.0, "remaining_ah": 147.7,
            "charge_fet": "ON", "discharge_fet": "ON", "protection_raw": 0}
    cells = [3.345, 3.344, 3.345, 3.345, 3.348, 3.345, 3.345, 3.345, 3.345, 3.342,
             3.346, 3.345, 3.345, 3.345, 3.345, 3.345]
    for i, c in enumerate(cells, 1):
        fake[f"cell_{i:02d}"] = c
    for i, t in enumerate([15.6, 15.8, 15.5, 15.7, 15.6, 15.4], 1):
        fake[f"temp_{i}"] = t

    n = [0]

    def check(cond, msg):
        n[0] += 1
        if not cond:
            raise AssertionError(msg)

    def run(state, age=0.0, mutate=None):
        snap = extract_snapshot(state)
        lt = Latches(p)
        lt.update(snap)
        if mutate:
            mutate(lt)
        return derive(snap, lt, age, p)

    req = lambda dev, fc, start, count: with_crc(bytes([dev, fc]) + be16(start) + be16(count))

    check(req(1, 3, 0, 1).hex() == "010300000001840a", "CRC16 against a known Modbus frame")
    d = run(fake)
    # healthy
    check(d["limit"] == 10000 and d["hr20"] == 0 and d["stage"] == 0, "healthy state")
    hr = holding_regs(d)
    check(hr[22] == 5351, "HR22 is 0.01 V")
    check(hr[23] == 3404, "HR23 + = charging")
    check(hr[21] == 47, "HR21 is the BMS's own SoC, unscaled")
    check(hr[11] == 314, "HR11 is the BMS's real capacity")
    check(hr[25] == 10000, "HR25 0.01 A")
    # frame shapes
    r = build_reply(req(1, 3, 0, 26), d, p)
    check(len(r) == 57 and crc_ok(r) and r[2] == 52, "FC03 26 regs")
    r = build_reply(req(1, 3, 17, 9), d, p)
    check(len(r) == 23 and crc_ok(r), "FC03 short poll")
    check(build_reply(req(2, 3, 0, 26), d, p) is None, "FC03 other device ignored")
    check(build_reply(req(1, 3, 27, 5), d, p)[1] == 0x83, "FC03 range exception")
    for (st, cnt), size in (((0, 21), 42), ((0x15, 19), 38), ((0x28, 20), 40)):
        r = build_reply(req(1, 4, st, cnt), d, p)
        check(len(r) == 4 + size + 2 and crc_ok(r) and r[2:4] == be16(st), f"FC04 {st:#x}")
        r = build_reply(req(3, 4, st, cnt), d, p)
        check(len(r) == 4 + size + 2 and crc_ok(r), f"FC04 absent {st:#x}")
    b2 = block2(d)
    check(len(b2) == 38 and b2[0] == 0x10, "block 2 template")
    check(int.from_bytes(b2[5:7], "big") == 53510, "block 2 pack mV")
    check(int.from_bytes(b2[13:17], "big") == 31400, "block 2 full capacity u32")
    check(int.from_bytes(b2[21:25], "big") == 14770, "block 2 remaining u32 = the BMS's real remaining Ah")
    check(b2[25] == 47 and b2[35:37] == b"\x0B\xCE", "block 2 SoC / firmware")
    b3 = block3(d)
    check(int.from_bytes(b3[36:38], "big") == 3348, "block 3 offset 36 = HIGHEST cell")
    check(int.from_bytes(b3[38:40], "big") == 3342, "block 3 offset 38 = LOWEST cell")
    check(all(2200 < int.from_bytes(b3[2 * i:2 * i + 2], "big") < 3700 for i in range(16)),
          "cells inside 2200-3700 mV")
    check(build_reply(req(1, 6, 0x10, 1), d, p) == req(1, 6, 0x10, 1), "FC06 echo")
    # fail-safe stages
    check(run(fake, 4.9)["limit"] == 10000, "no stage before 5 s")
    d1 = run(fake, 5.0)
    check(d1["limit"] == 0 and d1["hr20"] == 0, "stage 1: HR25 only")
    d2 = run(fake, 20.0)
    check(d2["limit"] == 0 and d2["hr20"] == 0x0C, "stage 2: HR20 bits 2+3")
    d0 = derive(None, Latches(p), 1.0, p)
    check(d0["limit"] == 0, "no data yet: HR25 = 0")
    check(derive(None, Latches(p), 21.0, p)["hr20"] == 0x0C, "no data for 20 s: bits 2+3")
    # protections
    check(run(dict(fake, protection_raw=1 << 8))["hr20"] == 1 and
          run(dict(fake, protection_raw=1 << 8))["limit"] == 0, "charge OC -> bit 0, HR25 0")
    check(run(dict(fake, protection_raw=1 << 10))["hr20"] == 2, "short circuit -> bit 1")
    check(run(dict(fake, protection_raw=1 << 6))["hr20"] == 0x10, "discharge OT -> bit 4")
    check(run(dict(fake, charge_fet="OFF"))["hr20"] == 0x04, "charge FET off -> bit 2")
    check(run(dict(fake, discharge_fet="OFF"))["hr20"] == 0x08, "discharge FET off -> bit 3")
    hot = dict(fake, cell_05=3.56)
    check(run(hot)["hr20"] & 4 and run(hot)["limit"] == 0, "cell > 3.55 V -> full + HR25 0")
    cold = {k: (-1.0 if k.startswith("temp_") else v) for k, v in fake.items()}
    check(run(cold)["hr20"] & 0x80, "below 0 C -> charge under-temp")
    lt = Latches(p)
    lt.update(extract_snapshot(hot))
    lt.update(extract_snapshot(dict(fake, cell_05=3.50)))
    check(lt.cell_ov.on, "latch holds between 3.45 and 3.55 V")
    lt.update(extract_snapshot(fake))
    check(not lt.cell_ov.on, "latch clears below 3.45 V")
    # bad cell counts are rejected, not guessed at
    short = {k: v for k, v in fake.items() if k != "cell_16"}
    try:
        extract_snapshot(short)
        check(False, "short cell list accepted")
    except BMSError:
        n[0] += 1
    msgs = inverter_discovery("dev", "pre", "Name", 60)
    check(len(msgs) == 10 and len({t for t, _ in msgs}) == 10, "10 distinct inverter entities")
    for t, pl in msgs:
        json.dumps(pl)
        check(pl["unique_id"].startswith("dev_inv_") and pl["state_topic"] == "pre/inverter",
              f"discovery payload {t}")
    print(f"selftest OK ({n[0]} checks)")
    print("sample FC03 reply:", build_reply(req(1, 3, 0, 26), d, p).hex(" "))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-c", "--config",
                    default=os.environ.get("GIV_BRIDGE_CONF", "/etc/giv-bridge.conf"))
    ap.add_argument("--selftest", action="store_true", help="check the logic, no hardware")
    ap.add_argument("-v", "--verbose", action="store_true", help="log every frame")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    cfg = load_config(args.config)
    log.info("config: %s", args.config)
    gw = Gateway(cfg)

    def stop(signum, frame):
        log.info("signal %d, shutting down", signum)
        gw.running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    gw.run()


if __name__ == "__main__":
    main()
