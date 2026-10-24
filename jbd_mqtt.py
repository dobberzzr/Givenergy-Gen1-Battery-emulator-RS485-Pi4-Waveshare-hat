#!/usr/bin/env python3
"""
DATOUBOSS (JBD BMS) RS485 -> MQTT bridge for Home Assistant.  READ-ONLY.

Speaks the BMS's native JBD protocol (the same thing jbdtool uses) and
publishes everything to MQTT with Home Assistant auto-discovery:
pack voltage/current/power, SOC, capacity, cycles, all cell voltages,
cell min/max/avg/spread, temperatures, FET states, balancing, and every
protection alarm as its own warning light.

Only two commands are ever sent: 0x03 (basic info) and 0x04 (cell
voltages).  Both are reads.  Nothing is ever written to the BMS.

  Test once, print and exit:   python3 jbd_mqtt.py --test
  Normal run (systemd):        python3 jbd_mqtt.py
"""
import argparse
import configparser
import json
import logging
import os
import signal
import sys
import time

import serial
import paho.mqtt.client as mqtt

log = logging.getLogger("jbd-mqtt")

# Protection status word (basic info bytes 16-17), bit order per JBD spec.
PROTECTION_BITS = [
    ("cell_overvoltage", "Cell overvoltage"),
    ("cell_undervoltage", "Cell undervoltage"),
    ("pack_overvoltage", "Pack overvoltage"),
    ("pack_undervoltage", "Pack undervoltage"),
    ("charge_overtemp", "Charge over-temperature"),
    ("charge_undertemp", "Charge under-temperature"),
    ("discharge_overtemp", "Discharge over-temperature"),
    ("discharge_undertemp", "Discharge under-temperature"),
    ("charge_overcurrent", "Charge overcurrent"),
    ("discharge_overcurrent", "Discharge overcurrent"),
    ("short_circuit", "Short circuit"),
    ("frontend_ic_error", "Front-end IC error"),
    ("mos_software_lock", "MOSFET software lock"),
]


class BMSError(Exception):
    pass


# --------------------------------------------------------------------------
# JBD protocol
# --------------------------------------------------------------------------
def build_request(cmd):
    """DD A5 <cmd> 00 <chk_hi> <chk_lo> 77  (read, no payload)."""
    chk = (0x10000 - cmd) & 0xFFFF
    return bytes([0xDD, 0xA5, cmd, 0x00, chk >> 8, chk & 0xFF, 0x77])


def transact(port, cmd, timeout=1.5):
    """Send one read command and return the validated data payload."""
    port.reset_input_buffer()
    port.write(build_request(cmd))
    port.flush()

    buf = bytearray()
    frame = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        chunk = port.read(port.in_waiting or 1)
        if chunk:
            buf += chunk
        start = buf.find(b"\xDD")
        if start < 0:
            buf.clear()
            continue
        if start:
            del buf[:start]
        if len(buf) >= 4:
            total = 4 + buf[3] + 3          # header + data + chk(2) + 0x77
            if len(buf) >= total:
                frame = bytes(buf[:total])
                break
    if frame is None:
        raise BMSError(f"timeout on cmd 0x{cmd:02X} (got {len(buf)} bytes)")

    if frame[1] != cmd:
        raise BMSError(f"reply for cmd 0x{frame[1]:02X}, expected 0x{cmd:02X}")
    if frame[2] != 0x00:
        raise BMSError(f"BMS returned error status 0x{frame[2]:02X}")
    if frame[-1] != 0x77:
        raise BMSError("bad end byte")
    want = (0x10000 - sum(frame[2:-3])) & 0xFFFF
    got = (frame[-3] << 8) | frame[-2]
    if want != got:
        raise BMSError(f"checksum mismatch {got:04X} != {want:04X}")
    return frame[4:-3]


def u16(d, i):
    return (d[i] << 8) | d[i + 1]


def s16(d, i):
    v = u16(d, i)
    return v - 0x10000 if v & 0x8000 else v


def read_bms(port):
    """Read basic info + cells and return a flat dict ready for MQTT."""
    b = transact(port, 0x03)
    if len(b) < 23:
        raise BMSError(f"basic info too short ({len(b)} bytes)")

    voltage = u16(b, 0) / 100.0
    current = s16(b, 2) / 100.0
    n_cells = b[21]
    n_temps = b[22]
    if len(b) < 23 + 2 * n_temps:
        raise BMSError("basic info truncated before temperatures")

    temps = [round((u16(b, 23 + 2 * i) - 2731) / 10.0, 1) for i in range(n_temps)]
    balance = u16(b, 12) | (u16(b, 14) << 16)
    prot = u16(b, 16)
    fet = b[20]

    time.sleep(0.3)   # JBD needs a breather between requests
    c = transact(port, 0x04)
    cells = [u16(c, 2 * i) / 1000.0 for i in range(len(c) // 2)][:n_cells]
    if len(cells) != n_cells or not cells:
        raise BMSError(f"expected {n_cells} cells, got {len(cells)}")

    on = lambda x: "ON" if x else "OFF"
    s = {
        "voltage": round(voltage, 2),
        "current": round(current, 2),
        "power": round(voltage * current),
        "soc": b[19],
        "remaining_ah": round(u16(b, 4) / 100.0, 2),
        "nominal_ah": round(u16(b, 6) / 100.0, 2),
        "cycles": u16(b, 8),
        "cell_min": round(min(cells), 3),
        "cell_max": round(max(cells), 3),
        "cell_avg": round(sum(cells) / len(cells), 3),
        "cell_diff_mv": round((max(cells) - min(cells)) * 1000),
        "cell_min_no": cells.index(min(cells)) + 1,
        "cell_max_no": cells.index(max(cells)) + 1,
        "temp_max": max(temps) if temps else None,
        "temp_min": min(temps) if temps else None,
        "charge_fet": on(fet & 0x01),
        "discharge_fet": on(fet & 0x02),
        "balancing": on(balance),
        "balance_bits": balance,
        "protection_raw": prot,
        "alarm_any": on(prot),
    }
    for i, v in enumerate(cells, 1):
        s[f"cell_{i:02d}"] = round(v, 3)
    for i, t in enumerate(temps, 1):
        s[f"temp_{i}"] = t
    for bit, (key, _) in enumerate(PROTECTION_BITS):
        s[f"alarm_{key}"] = on(prot & (1 << bit))
    return s


# --------------------------------------------------------------------------
# Home Assistant discovery
# --------------------------------------------------------------------------
def discovery_messages(cfg, state):
    dev_id = cfg["device_id"]
    prefix = cfg["topic_prefix"]
    device = {
        "identifiers": [dev_id],
        "name": cfg["device_name"],
        "manufacturer": "DATOUBOSS / JBD",
        "model": "LiFePO4 48V BMS",
    }
    common = {
        "state_topic": f"{prefix}/state",
        "availability_topic": f"{prefix}/availability",
        "expire_after": cfg.getint("expire_after"),
        "device": device,
    }

    # key, name, unit, device_class, state_class, precision, icon
    sensors = [
        ("voltage", "Pack voltage", "V", "voltage", "measurement", 2, None),
        ("current", "Pack current", "A", "current", "measurement", 2, None),
        ("power", "Pack power", "W", "power", "measurement", 0, None),
        ("soc", "State of charge", "%", "battery", "measurement", 0, None),
        ("remaining_ah", "Remaining capacity", "Ah", None, "measurement", 1, "mdi:battery-medium"),
        ("nominal_ah", "Design capacity", "Ah", None, None, 0, "mdi:battery"),
        ("cycles", "Cycle count", None, None, "total_increasing", 0, "mdi:counter"),
        ("cell_min", "Cell min", "V", "voltage", "measurement", 3, None),
        ("cell_max", "Cell max", "V", "voltage", "measurement", 3, None),
        ("cell_avg", "Cell average", "V", "voltage", "measurement", 3, None),
        ("cell_diff_mv", "Cell spread", "mV", "voltage", "measurement", 0, None),
        ("cell_min_no", "Lowest cell", None, None, None, 0, "mdi:arrow-down-bold"),
        ("cell_max_no", "Highest cell", None, None, None, 0, "mdi:arrow-up-bold"),
        ("temp_max", "Temperature max", "°C", "temperature", "measurement", 1, None),
        ("temp_min", "Temperature min", "°C", "temperature", "measurement", 1, None),
        ("protection_raw", "Protection word", None, None, None, 0, "mdi:shield-alert"),
    ]
    sensors += [(k, f"Cell {k[5:]}", "V", "voltage", "measurement", 3, None)
                for k in state if k.startswith("cell_") and k[5:].isdigit()]
    sensors += [(k, f"Temperature {k[5:]}", "°C", "temperature", "measurement", 1, None)
                for k in state if k.startswith("temp_") and k[5:].isdigit()]

    binaries = [
        ("charge_fet", "Charge FET", None, "mdi:battery-arrow-up"),
        ("discharge_fet", "Discharge FET", None, "mdi:battery-arrow-down"),
        ("balancing", "Balancing", None, "mdi:scale-balance"),
        ("alarm_any", "Any alarm", "problem", None),
    ]
    binaries += [(f"alarm_{k}", n, "problem", None) for k, n in PROTECTION_BITS]

    msgs = []
    for key, name, unit, dclass, sclass, prec, icon in sensors:
        p = dict(common, name=name, unique_id=f"{dev_id}_{key}",
                 value_template=f"{{{{ value_json.{key} }}}}")
        if unit:
            p["unit_of_measurement"] = unit
        if dclass:
            p["device_class"] = dclass
        if sclass:
            p["state_class"] = sclass
        if prec is not None:
            p["suggested_display_precision"] = prec
        if icon:
            p["icon"] = icon
        msgs.append((f"homeassistant/sensor/{dev_id}/{key}/config", p))

    for key, name, dclass, icon in binaries:
        p = dict(common, name=name, unique_id=f"{dev_id}_{key}",
                 value_template=f"{{{{ value_json.{key} }}}}",
                 payload_on="ON", payload_off="OFF")
        if dclass:
            p["device_class"] = dclass
        if icon:
            p["icon"] = icon
        msgs.append((f"homeassistant/binary_sensor/{dev_id}/{key}/config", p))
    return msgs


# --------------------------------------------------------------------------
# Bridge
# --------------------------------------------------------------------------
class Bridge:
    def __init__(self, cfg):
        self.cfg = cfg
        self.prefix = cfg["topic_prefix"]
        self.avail = f"{self.prefix}/availability"
        self.last_state = None
        self.discovery_sent = False
        self.connected = False
        self.running = True

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id=f"jbd-mqtt-{cfg['device_id']}")
        user = cfg.get("mqtt_username", "").strip()
        if user:
            self.client.username_pw_set(user, cfg.get("mqtt_password", "").strip() or None)
        self.client.will_set(self.avail, "offline", qos=1, retain=True)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(min_delay=2, max_delay=60)

    # MQTT callbacks ---------------------------------------------------------
    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            log.error("mqtt connect failed: %s", reason_code)
            return
        log.info("mqtt connected")
        self.connected = True
        client.subscribe("homeassistant/status")
        client.publish(self.avail, "online", qos=1, retain=True)
        self.discovery_sent = False
        if self.last_state:
            self.send_discovery()

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.connected = False
        log.warning("mqtt disconnected: %s", reason_code)

    def on_message(self, client, userdata, msg):
        # Home Assistant restarted: re-announce everything.
        if msg.topic == "homeassistant/status" and msg.payload == b"online":
            log.info("Home Assistant came online, re-sending discovery")
            self.discovery_sent = False
            if self.last_state:
                self.send_discovery()

    def send_discovery(self):
        msgs = discovery_messages(self.cfg, self.last_state)
        for topic, payload in msgs:
            self.client.publish(topic, json.dumps(payload), qos=1, retain=True)
        self.discovery_sent = True
        log.info("published HA discovery for %d entities", len(msgs))

    # Main loop ----------------------------------------------------------------
    def open_port(self):
        return serial.Serial(self.cfg["serial_port"], self.cfg.getint("baud"),
                             bytesize=8, parity="N", stopbits=1, timeout=0.1)

    def run(self):
        poll = self.cfg.getfloat("poll_interval")
        stale = self.cfg.getfloat("stale_restart")

        self.client.connect_async(self.cfg["mqtt_host"], self.cfg.getint("mqtt_port"), 60)
        self.client.loop_start()

        port = None
        last_ok = time.monotonic()
        fails = 0
        while self.running:
            t0 = time.monotonic()
            try:
                if port is None:
                    port = self.open_port()
                    log.info("serial: %s @ %s", self.cfg["serial_port"], self.cfg["baud"])
                state = read_bms(port)
                self.last_state = state
                last_ok = time.monotonic()
                if fails:
                    log.info("BMS reads recovered after %d failure(s)", fails)
                fails = 0
                if self.connected:
                    if not self.discovery_sent:
                        self.send_discovery()
                    self.client.publish(f"{self.prefix}/state", json.dumps(state), qos=0)
            except serial.SerialException as e:
                fails += 1
                log.warning("serial error: %s - reopening port", e)
                try:
                    if port:
                        port.close()
                except Exception:
                    pass
                port = None
            except BMSError as e:
                fails += 1
                log.warning("bad BMS read (%d in a row): %s", fails, e)

            # Watchdog: if the BMS has been silent too long, exit and let
            # systemd restart us from a clean slate rather than sit there
            # quietly doing nothing.
            if time.monotonic() - last_ok > stale:
                log.error("no good BMS read for %.0f s - exiting for restart", stale)
                self.shutdown()
                sys.exit(1)

            time.sleep(max(0.0, poll - (time.monotonic() - t0)))

        self.shutdown()

    def shutdown(self):
        self.running = False
        try:
            self.client.publish(self.avail, "offline", qos=1, retain=True).wait_for_publish(2)
        except Exception:
            pass
        self.client.loop_stop()
        self.client.disconnect()


# --------------------------------------------------------------------------
def load_config(path):
    cp = configparser.ConfigParser(inline_comment_prefixes=(";", "#"))
    if not cp.read(path):
        sys.exit(f"cannot read config {path}")
    sec = cp["jbd-mqtt"]
    defaults = {
        "serial_port": "/dev/serial0", "baud": "9600", "poll_interval": "5",
        "mqtt_host": "localhost", "mqtt_port": "1883",
        "topic_prefix": "jbd/datouboss", "device_id": "datouboss_jbd",
        "device_name": "DATOUBOSS Battery (JBD)",
        "expire_after": "60", "stale_restart": "120",
    }
    for k, v in defaults.items():
        if k not in sec:
            sec[k] = v
    return sec


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-c", "--config",
                    default=os.environ.get("JBD_MQTT_CONF", "/etc/jbd-mqtt.conf"))
    ap.add_argument("--test", action="store_true",
                    help="read the BMS once, print the result, exit (no MQTT)")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    cfg = load_config(args.config)
    log.info("config: %s", args.config)

    if args.test:
        with serial.Serial(cfg["serial_port"], cfg.getint("baud"),
                           timeout=0.1) as port:
            print(json.dumps(read_bms(port), indent=2))
        return

    bridge = Bridge(cfg)

    def stop(signum, frame):
        log.info("signal %d, shutting down", signum)
        bridge.running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    bridge.run()


if __name__ == "__main__":
    main()
