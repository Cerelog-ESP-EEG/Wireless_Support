# Wireless Support — Firmware and LSL Scripts

## About
This is a guide to using the ESP-EEG device wirelessly to stream data to your
computer. The device defaults to USB streaming from its original firmware.
Should you, after setting all this up, prefer USB again, flash the
[original firmware](https://github.com/Cerelog-ESP-EEG/ESP-EEG/tree/main/firmware)
back to revert the changes.

Two transports are provided. Both send the identical packet format and produce
the identical LSL stream — pick whichever suits you:

| | [WiFi](WiFi/) | [Bluetooth Low Energy (BLE)](Bluetooth%20Low%20Energy%20(BLE)/) |
| --- | --- | --- |
| How you connect | Join the hotspot the device creates | The script finds and connects to it directly |
| Setup | Must switch your computer's WiFi network | Nothing to join or pair |
| Internet while streaming | Not on the same adapter | Unaffected |
| Headroom | Comfortable | Enough for 250 Hz, with drop counting built in |

Each folder has its own README with full instructions:

* **[WiFi/](WiFi/)** — V1 and V2 firmware plus `Python_wifi_LSL.py`
* **[Bluetooth Low Energy (BLE)/](Bluetooth%20Low%20Energy%20(BLE)/)** — V2 firmware plus `Python_ble_LSL.py`

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

   See the [V2 flashing instructions](WiFi/(Works%20)%20WiFI%20Firmware%20%20(Device%20Host)/V2_WIFI_FW/readme.md) for troubleshooting.
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
