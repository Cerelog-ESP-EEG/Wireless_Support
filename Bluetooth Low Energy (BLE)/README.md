# Bluetooth Low Energy (BLE) Streaming

Same device, same packet format, same LSL output as the WiFi bridge — only the
transport changes. The device advertises as a BLE peripheral, your computer
connects to it directly (no network to join, no IP address), and
`Python_ble_LSL.py` republishes the stream as LSL.

The firmware is a copy of `V2_Devices_WiFi_hostfw` with only the connection
logic changed. The ADS1299 setup, register map, packet format, timestamping and
SD-card logging are byte-for-byte identical.

## Contents

| File | What it is |
| --- | --- |
| `V2_Devices_BLE_hostfw/` | V2 device firmware (flash this) |
| `Python_ble_LSL.py` | Host script: BLE -> LSL bridge |

## Instructions

1. Flash `V2_Devices_BLE_hostfw` with the Arduino IDE, exactly the same way as
   the WiFi firmware. No extra libraries are needed — the BLE library ships with
   the ESP32 Arduino core. (The build is *smaller* than the WiFi one: ~683 KB
   vs ~977 KB of flash, and ~18 KB less RAM.)

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

**macOS:** the first run will ask for Bluetooth permission. If you run it from
VS Code or a terminal, that app is what needs the permission
(System Settings > Privacy & Security > Bluetooth).

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
