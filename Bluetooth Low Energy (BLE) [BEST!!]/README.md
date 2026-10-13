# Bluetooth Low Energy (BLE) Streaming

Same device, same packet format, same LSL output as the WiFi bridge — only the
transport changes. The device advertises as a BLE peripheral, your computer
connects to it directly (no network to join, no IP address), and
`Python_ble_LSL.py` republishes the stream as LSL.

The V2 firmware is a copy of `V2_Devices_WiFi_hostfw` with only the connection
logic changed. The ADS1299 setup, register map, packet format, timestamping and
SD-card logging are byte-for-byte identical. The V1 firmware is the same idea
applied to the V1 USB firmware's ADS1299 layer, and the 16-channel firmware the
same idea applied to `16CH_EMUL_FW`'s dual-ADS1299 layer.

One host script covers all three boards. It tells them apart by the advertised
BLE name and sizes its LSL stream accordingly — 8 channels for V1/V2,
16 channels for the 16-channel board. For a 16-channel board it additionally
publishes the stream as an OpenBCI Cyton+Daisy WiFi shield, so the **stock
OpenBCI GUI can connect natively** alongside the LSL output — see
[Native OpenBCI GUI connection](#native-openbci-gui-connection-16-channel-board-only).

## Contents

| File | What it is |
| --- | --- |
| `BLE Firmware/V2_Devices_BLE_hostfw/` | V2 firmware, 8 channels (ESP32-S3) |
| `BLE Firmware/V1_Devices_BLE_hostfw/` | V1 firmware, 8 channels (classic ESP32) |
| `BLE Firmware/16CH_Devices_BLE_hostfw/` | 16-channel firmware (ESP32-S3) |
| `Python_ble_LSL.py` | Host script: BLE -> LSL bridge, works with all three |

## Instructions

1. Flash `V2_Devices_BLE_hostfw` with the Arduino IDE, exactly the same way as
   the WiFi firmware. No extra libraries are needed — the BLE library ships with
   the ESP32 Arduino core. (The build is *smaller* than the WiFi one: ~683 KB
   vs ~977 KB of flash, and ~18 KB less RAM.)

   Pick the sketch that matches your board. For **V1**, flash
   `BLE Firmware/V1_Devices_BLE_hostfw` and select
   **'ESP32 WROOM DA Module'** - V1 is a different chip; the CH340 bridge on it
   is unreliable at the default upload speed, so also set
   **Tools > Upload Speed > 115200**. For the **16-channel** board flash
   `BLE Firmware/16CH_Devices_BLE_hostfw`, which takes exactly the same settings
   as V2 below (both are ESP32-S3). Note that these are *not* the settings the
   16-channel **USB** firmware needs: `16CH_EMUL_FW` requires USB-OTG (TinyUSB)
   with CDC-on-boot disabled so the OpenBCI GUI sees a "VCP" port, and the BLE
   sketch deliberately does not - it has no serial port to impersonate. If you
   are switching a 16-channel board from USB to BLE, change those two settings
   back. The settings below are for **V2 and 16 channel** (both ESP32-S3). The
   last two are *not* defaults — without them the flash appears to succeed but the firmware will not
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

   On a **16-channel** board you can skip the fork entirely and connect the
   stock GUI natively instead — run the script with `sudo` and see
   [Native OpenBCI GUI connection](#native-openbci-gui-connection-16-channel-board-only).
   Both outputs are live at the same time, so you can use either or both.

**Legacy usage:** with the old OpenBCI forked GUI, swap the
`GUI_CORRECTION_FACTOR` line for the commented-out one, same as in the WiFi
script.

## What changed from the WiFi firmware

| WiFi | BLE |
| --- | --- |
| SoftAP `CERELOG_EEG` / `cerelog123` | BLE peripheral advertising as `CERELOG_EEG` (`CERELOG_EEG_16` on the 16-channel board) |
| UDP 4445 `CERELOG_FIND_ME` discovery | BLE scan for the device name / service UUID |
| TCP socket on port 1112 | GATT notify characteristic |
| Stream starts on TCP connect | Stream starts when the host writes `CMD_START` |
| One 37-byte `client.write()` per sample | Up to 13 packets coalesced into one notification (8 of the 61-byte 16-channel packets) |

GATT UUIDs (they must match on both sides):

```
Service  7a1e8b00-9c3d-4b7e-8f21-6d0a5c2e91f3
Data     7a1e8b01-9c3d-4b7e-8f21-6d0a5c2e91f3   notify
Control  7a1e8b02-9c3d-4b7e-8f21-6d0a5c2e91f3   write
```

All three boards use the **same** UUIDs, so one scan filter finds any of them;
only the advertised name differs. That name is what tells the host how wide the
packets are, because the LSL outlet has to be sized before the first sample
arrives. If a board is ever matched by service UUID with no name available, the
script falls back to reading the width out of the first packet's length field.

The control write is `[0x01, mtu_hi, mtu_lo]` to start and `[0x00]` to stop.
The host sends the ATT MTU it negotiated so the firmware never stages a
notification the link would truncate.

### Packet format

Identical framing on every board - only the channel count differs:

```
 [0:2]   0xABCD                     start marker
 [2]     message length             31 (8 ch) or 55 (16 ch)
 [3:7]   uint32 big-endian          device clock, ms
 [7:10]  ADS1299 status word
 [10:..] N x 24-bit big-endian      8 or 16 channels
 [..]    checksum                   sum(bytes 2..checksum-1) & 0xFF
 [..:..] 0xDCBA                     end marker
```

Total 37 bytes for 8 channels, 61 for 16. On the 16-channel board channels 1-8
come from ADC1 and 9-16 from ADC2; the status word on the wire is ADC1's, and
each device's own status word is written to the SD log.

## Packet dropping

BLE has far less headroom than TCP: 250 Hz x 37 bytes is ~9.3 kB/s for an
8-channel board, and 250 Hz x 61 bytes is ~15.3 kB/s for the 16-channel one.
Both are comfortable on a healthy link but neither is guaranteed on a busy
2.4 GHz band. The
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

Staging is bounded at one full notification - 13 packets / 481 bytes on the
8-channel boards, 8 packets / 488 bytes on the 16-channel one - and
stopping the stream discards whatever is still staged, so a reconnect always
starts clean rather than replaying stale samples.

## Native OpenBCI GUI connection (16-channel board only)

On top of the LSL outlet, a 16-channel board is also published as an **OpenBCI
Cyton+Daisy WiFi shield**, which the stock GUI connects to natively — no fork,
no LSL bridge. Both outputs run at once from the one script; the LSL stream is
unchanged and is still the primary output.

```
sudo -E python3 Python_ble_LSL.py
```

Then in the GUI: **CYTON (LIVE) → WIFI → STATIC IP**, enter `127.0.0.1`, set
**CHANNEL COUNT** to **16** and **SAMPLE RATE** to **250**, and start the
session. The script prints the address to use (and your LAN address, if you want
to run the GUI on another machine).

The sample rate is not optional. The GUI plots, filters and FFTs against
whatever rate you pick, so picking 500 or 1000 stretches the time axis by 2x or
4x. The script watches for it and says so:

```
[GUI] *** WARNING: the GUI is set to 1000 Hz but this board streams 250 Hz. ***
```

### Why it needs sudo

BrainFlow builds its URLs as `"http://" + <the IP you typed> + "/board"` with no
port, and validates that string with `inet_pton` — so it must be a bare IPv4
address, and the HTTP port is unavoidably **80**. Ports below 1024 need root.
If the bind fails the script says so and carries on with LSL alone:

```
OpenBCI GUI emulation DISABLED: no permission to bind port 80.
```

If `sudo` breaks Bluetooth on your machine (macOS grants Bluetooth access per
app, and root has its own permission store), you still have the LSL path, which
needs no privileges.

### Why not a virtual serial port, like the USB firmware

The USB firmware (`16CH_EMUL_FW`) got itself into the GUI's port list by being a
real USB CDC device whose product string started with `VCP` — the GUI calls
jSerialComm's `getCommPorts()` and keeps only ports whose name starts with
`"FT231X USB UART"` or `"VCP"`. A script cannot reproduce that:

* the only serial endpoint a user-space process can create is a pty;
* jSerialComm enumerates macOS serial ports out of the IOKit registry, and a
  pty never appears there (`ioreg -c IOSerialBSDClient` lists only real ports);
* the GUI additionally drops every `tty*` name on macOS, which is what a pty is;
* `VCP` comes from a USB descriptor, which a pty does not have;
* devfs does not accept symlinks, so the name cannot be faked either.

A virtual serial port on macOS needs a kernel driver. The WiFi shield is the
GUI's other native 16-channel path, it is pure HTTP + TCP, and it carries the
**same 33-byte packets** — so the firmware's EMUL wire format came across
unchanged and only the transport differs.

### The protocol

BrainFlow drives five endpoints, in this order, and the data connection is
outbound — BrainFlow binds a server socket and waits for the shield to dial in:

| Request | What the script does |
| --- | --- |
| `GET /board` | reports a connected 16-channel board |
| `POST /tcp` | `{"ip","port"}` → opens a TCP connection **back** to that address |
| `POST /command` | acknowledges `~4`, `d`, `~6`, `x..X`, … and changes nothing |
| `GET /stream/start` | begins pushing packets |
| `GET /stream/stop` | stops |

Packets are the Cyton format the USB firmware already built:

```
 [0]      0xA0                      start byte
 [1]      counter                   SAME in both halves of a 16-ch sample
 [2:26]   8 x 24-bit big-endian     ch1-8 first packet, ch9-16 second
 [26:32]  6 aux bytes               zero, no accelerometer fitted
 [32]     0xC0                      standard stop byte
```

Two things differ from the USB firmware's serial output, both because
BrainFlow's `cyton_daisy_wifi` reader requires it rather than by choice:

* **Pairing.** The serial EMUL reader splits a 16-channel sample by the *parity*
  of the counter (even = ch9-16 first). The WiFi reader pairs two packets
  carrying the *same* counter, channels 1-8 first, and commits on the second.
* **Rate.** The serial path had to average two conversions down to 125 Hz,
  because BrainFlow fixes that rate for the 16-channel serial board. The WiFi
  board's rate is whatever the GUI is told, so **250 Hz goes out untouched** —
  the downsampling is gone.

### Amplitudes differ from the LSL stream, on purpose

The wire carries **raw ADS1299 counts**. BrainFlow applies
`4.5 V / (2^23-1) / gain`, and its 16-channel gain tracker defaults to gain 24
on every channel — exactly this hardware — so the GUI shows true microvolts
with the script scaling nothing. That is the physically correct figure and it is
**not** the same as the LSL stream, which carries the fork-specific
`GUI_CORRECTION_FACTOR` (1.128x) and a DC blocker. The GUI does its own
filtering, so no DC blocker is applied on this path.

### Hardware settings are ignored, as they were over USB

The GUI's Hardware Settings panel can push per-channel commands. They are
acknowledged so the GUI stays happy, and **not one ADS1299 register is
written** — the same lock the USB firmware had via
`OBCI_ALLOW_GUI_HARDWARE_SETTINGS = 0`. Gain, montage and the bias policy are
fixed by the BLE firmware's register tables. Each command is logged so you can
see it arrive.

To turn the whole thing off, set `OPENBCI_EMULATION = False` near the top of the
script.

## SD-card logging while idle

One real behavioural difference between the boards. On **V1 and V2** the
ADS1299 is held idle (START low) until the host writes `CMD_START`, so the SD
card records only while you are streaming — that gating is what keeps the
wireless stream clean on those boards.

The **16-channel** firmware instead leaves both ADS1299s free-running from boot,
exactly as its USB firmware does, so **the SD card records whenever the board is
powered**, connected or not. `CMD_START` / `CMD_STOP` open and close the BLE
transmit path only. Two reasons: logging-whenever-powered is a feature of that
board, and its START sequence is a single shared hardware edge that also keeps
ADC2 (which runs on ADC1's clock) in step, so re-running it per session would be
risk with no benefit.

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

Because packets arrive in batches (up to 13 on an 8-channel board, 8 on the
16-channel one), stamping them with arrival time
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
