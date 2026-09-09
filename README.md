# Wireless Support — Firmware and LSL Scripts

> ### Use BLE. [Start here.](<Bluetooth Low Energy (BLE) [BEST!!]/>)
>
> **BLE is the recommended transport.** It streams the most reliably, it is the
> simplest to set up — nothing to join, nothing to pair, and your internet keeps
> working — and it draws considerably less power than WiFi, so you get
> meaningfully longer battery life on a portable recording.
>
> WiFi is still supported and produces an identical stream. Use it if you have a
> specific reason to; otherwise use BLE.

## About
This is a guide to using the ESP-EEG device wirelessly to stream data to your
computer. The device defaults to USB streaming from its original firmware.
Should you, after setting all this up, prefer USB again, flash the
[original firmware](https://github.com/Cerelog-ESP-EEG/ESP-EEG/tree/main/firmware)
back to revert the changes.

Two transports are provided. Both send the identical packet format and produce
the identical LSL stream:

| | [WiFi](<WiFi [Note: BLE works way better]/>) | [Bluetooth Low Energy (BLE)](<Bluetooth Low Energy (BLE) [BEST!!]/>) |
| --- | --- | --- |
| How you connect | Join the hotspot the device creates | The script finds and connects to it directly |
| Setup | Must switch your computer's WiFi network | Nothing to join or pair |
| Internet while streaming | Not on the same adapter | Unaffected |
| Power draw | Higher — the device runs a WiFi access point continuously | **Lower — noticeably better battery life** |
| Headroom | Comfortable | Enough for 250 Hz, with drop counting built in |
| Recommended | Only if you need it | **Yes — start here** |

Each folder has its own README with full instructions:

* **[WiFi/](<WiFi [Note: BLE works way better]/>)** — V1 and V2 firmware plus `Python_wifi_LSL.py`
* **[Bluetooth Low Energy (BLE)/](<Bluetooth Low Energy (BLE) [BEST!!]/>)** — V1 and V2 firmware plus `Python_ble_LSL.py` **(recommended)**

## For usage only with the forked OpenBCI Gui/LSL Streaming

**Note: To instead use a Brainflow API instance, currently only supported via USB (connect to a laptop not connected to mains)**

Whichever transport you choose, the flow is the same:

1. Flash the matching firmware with the [Arduino IDE](http://arduino.cc/en/software/).

   Board depends on which device you have — they are different chips, so the
   wrong choice will not run:

   | Device | `Tools > Board` |
   | --- | --- |
   | **V1** | **ESP32 Arduino > 'ESP32 WROOM DA Module'** |
   | **V2** and **16 channel** | **ESP32 Arduino > 'ESP32S3 Dev Module'** |

   On **V2 / 16 channel only**, also set these two. They are *not* defaults, and
   without them the flash appears to succeed but the firmware will not run and
   you will not be able to log data:

   | Setting | Value |
   | --- | --- |
   | USB CDC On Boot | **Enabled** |
   | USB Mode | **Hardware CDC and JTAG** |

   Port: **Tools > Port**, select the port of your Cerelog board. A V2 enumerates
   as a native USB device (on macOS/Linux, `/dev/cu.usbmodem*`); a V1 appears
   through its USB-serial bridge (`/dev/cu.usbserial-*`).

   See the [V2 flashing instructions](<WiFi [Note: BLE works way better]/(Works ) WiFI Firmware  (Device Host)/V2_WIFI_FW/readme.md>) for troubleshooting.
2. Connect to the device (join the WiFi hotspot, or just run the BLE script).
3. Run the LSL bridge script for that transport, then follow the
   [OpenBCI GUI fork guide](https://github.com/Cerelog-ESP-EEG/How-to-use-OpenBCI-GUI-fork)
   — but use the script included here instead of the one listed there.

**Careful; For Legacy Usage:** with the old OpenBCI forked GUI listed in Method C
in the link above, you must comment out line 17 and uncomment line 18 in the
Python script to get the correct scaling factor.

**Getting an error?** If you run the WiFi script inside VS Code on a Mac you
need to allow local network connections in privacy settings so the code can talk
to the device, otherwise you get a 65 error. For the BLE script the equivalent
permission is Bluetooth.
