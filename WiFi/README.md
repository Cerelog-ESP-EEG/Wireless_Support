# WiFi Streaming

The device brings up its own WiFi access point; your computer joins it and
`Python_wifi_LSL.py` pulls the binary stream over TCP and republishes it as LSL.

## Contents

| File | What it is |
| --- | --- |
| `(Works ) WiFI Firmware  (Device Host)/V2_Devices_WiFi_hostfw/` | V2 device firmware (flash this) |
| `(Works ) WiFI Firmware  (Device Host)/V1 Devices_esp_hostfw_V1_Device.ino` | V1 device firmware |
| `Python_wifi_LSL.py` | Host script: TCP -> LSL bridge |

## Instructions

1. Flash the firmware for your device with the Arduino IDE.

   Download [Arduino IDE](http://arduino.cc/en/software/), then configure it:
   Board: **Tools > Board > ESP32 Arduino > 'ESP32 WROOM DA Module'**.
   Port: **Tools > Port**, select the COM port of your Cerelog board.

2. Connect your computer to the WiFi hotspot the device creates. It shows up as
   network **CERELOG_EEG**; the password is **cerelog123**.

3. Run `Python_wifi_LSL.py`, then follow the
   [OpenBCI GUI fork guide](https://github.com/Cerelog-ESP-EEG/How-to-use-OpenBCI-GUI-fork)
   — but use this script instead of the one listed there.

**Legacy usage:** with the old OpenBCI forked GUI (Method C in the link above),
comment out line 17 and uncomment line 18 in `Python_wifi_LSL.py` to get the
correct scaling factor.

**Getting a 65 error?** On macOS, VS Code needs local network permission
(System Settings > Privacy & Security > Local Network) before the script can
talk to the device.
