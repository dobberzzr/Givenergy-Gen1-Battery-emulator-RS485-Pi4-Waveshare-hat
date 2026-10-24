# Givenergy-Gen1-JBD-Gateway

A Raspberry Pi 4 gateway that lets a **GivEnergy Gen 1 hybrid inverter** use a
**DATOUBOSS (JBD BMS) LiFePO4 battery** it knows nothing about.

It reads the battery's BMS over RS485 and answers the inverter's battery polls on a
second RS485 port, so the inverter sees a GivEnergy-style battery with live data. The
same readings go to Home Assistant over MQTT.

```
 DATOUBOSS BMS --RS485 /dev/ttySC0--> giv_bridge.py --RS485 /dev/ttySC1--> Gen 1 inverter
   (JBD 0x03/0x04, read-only)             |            (Modbus RTU slave 1, 9600 8N1)
                                          '--MQTT--> Home Assistant
```

This replaces an earlier ESP32 emulator
([Givenergy-Gen1-EM115-Emulator](https://github.com/dobberzzr/Givenergy-Gen1-EM115-Emulator),
retired) and a separate Pi Zero MQTT bridge.

> **Safety.** The gateway is not pack protection. The BMS is. Values marked
> `ESTIMATE` in the config are not from a datasheet or a capture: check them before
> relying on them. Use at your own risk.

## Files

| File | What it is |
|---|---|
| `giv_bridge.py` | The gateway: battery thread, inverter thread, MQTT. |
| `jbd_mqtt.py` | JBD protocol and MQTT/Home Assistant code. Imported by `giv_bridge.py`; also runs on its own as a read-only battery-to-MQTT bridge. |
| `giv-bridge.service` | systemd unit (runs as user `giv`, locked down). |
| `giv-bridge.conf.example` | Config template. Copy to `/etc/giv-bridge.conf` and fill in the MQTT login. |

## How it works

Two threads that only share a snapshot:

- **Battery thread** polls the BMS about once a second, updates the snapshot and
  publishes to MQTT.
- **Inverter thread** answers the Gen 1's polls (FC03 holding registers, FC04 input
  blocks, FC06 echo) from the snapshot. It never waits on the battery, so a slow or
  dead BMS cannot stall a reply.

Everything the inverter is told is worked out from the snapshot **and its age** at the
moment of the reply, so stale data is never presented as live:

| BMS data age | What the inverter is told |
|---|---|
| under 5 s | live values, current limit (HR25) = `limit_a` |
| 5 s or more (stage 1) | HR25 = 0: stop current flow |
| 20 s or more (stage 2) | HR25 = 0 and HR20 bits 2 + 3 ("full" + "empty") |

Any JBD protection flag, a switched-off charge/discharge FET, or a pack or cell
voltage/temperature latch also raises the matching HR20 bit and drops HR25 to 0.
Latches trip at one level and clear at another, so they cannot chatter.

Replies match the earlier ESP32 firmware byte for byte wherever it used a fixed value.
Only fields that were fixed or faked there now carry live data.

## Hardware

- Raspberry Pi 4
- Waveshare **RS485 CAN HAT (B)** (SC16IS752, two RS485 channels on SPI)
- `/dev/ttySC0` is wired to the battery's RS485, `/dev/ttySC1` to the inverter's BMS
  port. Check Waveshare's manual for which terminal block is which channel.

Lines needed in `/boot/firmware/config.txt`, then reboot:

```
enable_uart=1
dtparam=spi=on
dtoverlay=sc16is752-spi1,int_pin=24
```

Check the ports exist: `ls -l /dev/ttySC*` (two devices, group `dialout`).

## Install

```bash
# dependencies, importable by /usr/bin/python3: pyserial and paho-mqtt 2.x
sudo apt install python3-serial python3-paho-mqtt
python3 -c "import paho.mqtt.client as m; m.CallbackAPIVersion"   # must not raise
#   if it does, your distro ships paho 1.x: use  pip install -r requirements.txt

# program files
sudo mkdir -p /opt/giv-bridge
sudo cp giv_bridge.py jbd_mqtt.py /opt/giv-bridge/

# service account and config (config is readable only by root and group giv)
sudo useradd --system --no-create-home --shell /usr/sbin/nologin giv
sudo install -m 640 -o root -g giv giv-bridge.conf.example /etc/giv-bridge.conf
sudo nano /etc/giv-bridge.conf        # set mqtt_host / mqtt_username / mqtt_password

# service
sudo cp giv-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now giv-bridge
```

If you previously ran `jbd_mqtt.py` on another machine (for example a Pi Zero), **stop
it first**: both publish to the same topics.

## Check it works

```bash
python3 giv_bridge.py --selftest      # no hardware needed; prints "selftest OK"
journalctl -u giv-bridge -f           # battery/inverter port opened, "inverter told: ..." lines
python3 giv_bridge.py -v              # log every frame (stop the service first)
```

In Home Assistant the battery appears as one device with all its usual sensors, plus
the inverter side: the current limit and SOC being told to the inverter (HR25, HR21),
the alarm register (HR20), fail-safe stage, BMS data age, time since the inverter last
polled, and a plain-text "why the inverter is limited".

## Configuration

See `giv-bridge.conf.example`. The section name stays `[jbd-mqtt]` because
`giv_bridge.py` reuses `jbd_mqtt.py`'s loader. The settings that matter most:

- `battery_port` / `inverter_port`: swap them if the battery answers on the other port.
- `limit_a`: HR25 while healthy, in amps. `ESTIMATE`.
- `pack_ov_*` / `pack_uv_*`: pack voltage latches, from the BMS screen.
- `cell_ov_*` / `cell_uv_*`: per-cell backstops ahead of the BMS's own trip. `ESTIMATE`.

## Known Gen 1 behaviour

If the inverter is told to stop at night (HR20 full/empty, or HR25 = 0) it can drop
into standby and stay there until restarted, because there is no solar to wake it. This
was seen with the ESP32 emulator. It is why stage 2 waits a full 20 seconds, and why a
Home Assistant alert for "cheap rate on, battery idle at 0 W for 10 minutes" is a good
backstop.

## Licence

MIT, see `LICENSE`.
