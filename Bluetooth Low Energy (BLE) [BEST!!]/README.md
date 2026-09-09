# Bluetooth Low Energy (BLE) Streaming

Same device, same packet format, same LSL output as the WiFi bridge — only the
transport changes. The device advertises as a BLE peripheral, your computer
connects to it directly (no network to join, no IP address), and
`Python_ble_LSL.py` republishes the stream as LSL.

The V2 firmware is a copy of `V2_Devices_WiFi_hostfw` with only the connection
logic changed. The ADS1299 setup, register map, packet format, timestamping and
SD-card logging are byte-for-byte identical. The V1 firmware is the same idea
applied to the V1 USB firmware's ADS1299 layer.

## Contents

| File | What it is |
| --- | --- |
| `BLE Firmware/V2_Devices_BLE_hostfw/` | V2 / 16-channel firmware (ESP32-S3) |
| `BLE Firmware/V1_Devices_BLE_hostfw/` | V1 firmware (classic ESP32) |
| `Python_ble_LSL.py` | Host script: BLE -> LSL bridge, works with either |

## Instructions

1. Flash `V2_Devices_BLE_hostfw` with the Arduino IDE, exactly the same way as
   the WiFi firmware. No extra libraries are needed — the BLE library ships with
   the ESP32 Arduino core. (The build is *smaller* than the WiFi one: ~683 KB
   vs ~977 KB of flash, and ~18 KB less RAM.)

   Pick the sketch that matches your board - they are different chips. For
   **V1**, flash `BLE Firmware/V1_Devices_BLE_hostfw` and select
   **'ESP32 WROOM DA Module'**; the CH340 bridge on V1 is unreliable at the
   default upload speed, so also set **Tools > Upload Speed > 115200**. The
   settings below are for **V2 / 16 channel** (an ESP32-S3). The last
   two are *not* defaults — without them the flash appears to succeed but the firmware will not
   run and you will not be able to log data:

   | Setting | Value |
   | --- | --- |
   | Board | **ESP32 Arduino > 'ESP32S3 Dev Module'** |
   | USB CDC On Boot | **Enabled** |
   | USB Mode | **Hardware CDC and JTAG** |

   Port: **Tools > Port**, select the port of your Cerelog board (it enumerates
   as a native USB device, so on macOS/Linux it is a `/dev/cu.usbmodem*`).

   See the [V2 flashing instructions](<../WiFi [Note: BLE works way better]/(Works ) WiFI Firmware  (Device Host)/V2_WIFI_FW/readme.md>)
   for more, and the troubleshooting section at the bottom of this page.

2. Install the host dependencies:

   ```
   pip install bleak pylsl
   ```

3. Run the script. There is nothing to pair or connect to first — it scans,
   finds the device by name, connects, and starts streaming.

   ```
   python Python_ble_LSL.py
   ```

4. Follow the
   [OpenBCI GUI fork guide](https://github.com/Cerelog-ESP-EEG/How-to-use-OpenBCI-GUI-fork)
   from there, using this script as the LSL bridge.

**Legacy usage:** with the old OpenBCI forked GUI, swap the
`GUI_CORRECTION_FACTOR` line for the commented-out one, same as in the WiFi
script.

## What changed from the WiFi firmware

| WiFi | BLE |
| --- | --- |
| SoftAP `CERELOG_EEG` / `cerelog123` | BLE peripheral advertising as `CERELOG_EEG` |
| UDP 4445 `CERELOG_FIND_ME` discovery | BLE scan for the device name / service UUID |
| TCP socket on port 1112 | GATT notify characteristic |
| Stream starts on TCP connect | Stream starts when the host writes `CMD_START` |
| One 37-byte `client.write()` per sample | Up to 13 packets coalesced into one notification |

GATT UUIDs (they must match on both sides):

```
Service  7a1e8b00-9c3d-4b7e-8f21-6d0a5c2e91f3
Data     7a1e8b01-9c3d-4b7e-8f21-6d0a5c2e91f3   notify
Control  7a1e8b02-9c3d-4b7e-8f21-6d0a5c2e91f3   write
```

The control write is `[0x01, mtu_hi, mtu_lo]` to start and `[0x00]` to stop.
The host sends the ATT MTU it negotiated so the firmware never stages a
notification the link would truncate.

## Packet dropping

BLE has far less headroom than TCP: 250 Hz x 37 bytes is ~9.3 kB/s, which is
comfortable on a healthy link but not guaranteed on a busy 2.4 GHz band. The
firmware is built to **drop rather than stall** — a backed-up radio must never
hold up the ADC read loop or push the stream further and further behind
real time.

Three things are counted and printed over USB serial every 5 seconds:

```
[STATS] sent=1250 ble_dropped=0 (notifies=0) adc_overrun=0 sd_dropped=0
```

* `ble_dropped` — samples in a batch the BLE stack refused. The staging buffer
  is emptied unconditionally on every flush, so a refused batch is discarded
  instead of re-queued. This is what keeps latency bounded.
* `adc_overrun` — DRDY fired before `loop()` had read the previous sample, so
  the ADS1299 overwrote it. Non-zero here means the loop is being starved.
* `sd_dropped` — the pre-existing SD-card queue counter, unchanged.

Staging is bounded at 13 packets (481 bytes, one full notification), and
`stop_acquisition()` discards whatever is still staged, so a reconnect always
starts clean rather than replaying stale samples.

The host script counts independently, using the 4-byte device timestamp in
every packet to spot gaps:

```
[STATS] 1250 samples pushed | 250 Hz recent | lost=0 bad_checksum=0 resync_bytes=0 | notifications=96
```

Lost samples are **counted, never fabricated** — no zero-filling or
value-holding, so nothing artificial reaches your recording. If `lost` climbs
steadily, move closer to the device or cut 2.4 GHz interference; if it stays at
0 you are getting exactly the same data the WiFi bridge would deliver.

## Timestamps

Because packets arrive in batches of up to 13, stamping them with arrival time
would bunch samples into ~50 ms lumps. The host script instead maps the
device's own sample clock (the ms field in each packet) onto the LSL clock, so
LSL sees an evenly spaced 250 Hz stream. Set `USE_DEVICE_TIMESTAMPS = False` at
the top of the script to fall back to arrival-time stamping, which is what the
WiFi script does.

## Troubleshooting

Symptoms first — find the line you are actually seeing.

### "I don't see the device in my Bluetooth settings"

You never will, and nothing is wrong. This is a BLE peripheral with a custom
GATT service, not a pairable accessory like a keyboard or headphones. It does
not appear in macOS System Settings > Bluetooth, Windows "Add a device", or
`bluetoothctl devices`, and **there is nothing to pair**. The script finds and
connects to it directly. Just run it.

### The script sits at "Scanning for BLE device" and never finds it

Almost always something else already holds the link. **A BLE peripheral accepts
exactly one connection at a time, and it stops advertising while connected** —
so a second copy of the script, or one left running in another terminal, makes
the device invisible to everything else.

```
ps aux | grep Python_ble_LSL        # macOS / Linux
```

Kill any stragglers and try again. Otherwise: is the board powered, and is its
LED on? Re-flashing and watching the USB serial output will show
`Ready. Advertising, waiting for BLE connection...` if the firmware is healthy.

### It reaches ">>> STREAMING STARTED >>>" and then prints nothing

`[STATS]` only prints once data arrives, so total silence means **zero
notifications are getting through** — not a parsing problem. Check the device's
USB serial output; the firmware prints its own counters every 5 s:

```
[STATS] sent=1239 ble_dropped=0 (notifies=0) adc_overrun=0 sd_dropped=0
```

* `sent=` climbing but nothing at the host — the link is dropping notifications.
  Move closer, cut 2.4 GHz interference.
* `sent=0` with `adc_overrun=0` — the ADS1299 is not producing samples at all.
  That is an ADC/SPI problem, not a BLE one.

### `lost=` climbs steadily and the rate sags below 250 Hz

```
[STATS] 1071 samples pushed | 214 Hz recent | lost=231 bad_checksum=0 resync_bytes=0
```

`bad_checksum=0` with `resync_bytes=0` means nothing arrived corrupted — whole
packets never arrived. The usual cause is **CPU contention on the host**. The
bridge is single-threaded asyncio; if something pegs a core, the event loop is
not scheduled in time and the OS Bluetooth stack discards queued notifications.

The most common offender is the **OpenBCI GUI running at the same time**. Quit
it, confirm `lost=0`, then start it again and point it at the LSL stream. If the
device's own `ble_dropped` and `adc_overrun` counters are both 0 while the host
reports loss, the firmware is fine and the problem is on the computer.

### Some channels read exactly 0.0, or sit at a constant value

Expected when nothing is connected to those inputs. An open ADS1299 input rails
to full scale (`0x7FFFFF` or `0x800000`). The script's DC blocker then turns
that constant into exactly `0.0`, because a signal that never changes has no AC
content. Connect electrodes and the channels come alive. It is not a decode bug —
check the raw values before assuming otherwise.

### Linux: `RuntimeError: LSL binary library file was not found`

`pip install pylsl` does not reliably ship the native `liblsl` on Linux the way
it does on macOS and Windows. Install it from the
[liblsl releases](https://github.com/sccn/liblsl/releases) and point pylsl at it:

```
export PYLSL_LIB=/usr/lib/liblsl.so
```

### Linux: scanning fails or finds nothing

Needs **BlueZ 5.43 or newer** with `bluetoothd` running (`systemctl status
bluetooth`). On a normal desktop a regular user can scan without root; on a
headless or minimal system you may need to be in the `bluetooth` group.

The script calls bleak's `_acquire_mtu()` before reading the MTU, which BlueZ
requires — without it BlueZ reports 23, and the firmware would split every
37-byte packet across two notifications (~500/s) and drop samples. macOS and
Windows report the real MTU directly and skip that call.

### Windows

Needs **Windows 10 build 16299 or newer** for bleak's WinRT backend. No pairing
is required. As on macOS, the device will not show up in the Bluetooth settings
list — see the first entry above.

### macOS: nothing happens, or a permissions error on first run

The first run asks for Bluetooth permission, and **the app that needs it is
whatever launched the script** — Terminal, iTerm, or VS Code, not Python.
Check System Settings > Privacy & Security > Bluetooth.

### A note on MTU and platforms

The firmware is platform-agnostic: it batches according to whatever ATT MTU the
host reports. A host negotiating a smaller MTU gets fewer packets per
notification and lower headroom — never corrupted data. A healthy link reports:

```
Negotiated ATT MTU: 515 (13 packet(s) per notification)
```
