# WiFi_Support 

## About
This is a guide to use the device wirelesly by connecting the ESP-EEG as a local WiFI network to your computer to stream data. The device defaults to USB streaming from its original firmware. Should, after setting all this up, you prefer to use USB to stream data flash this [original firmware](https://github.com/Cerelog-ESP-EEG/ESP-EEG/tree/main/firmware) back to revert changes

 
## For usage only with the forked OpenBCI Gui/LSL Streaming  

**Note: To instead use Brainflow API instance, currently only supported via USB (connect to laptop not connected to mains)** 


**To use WiFi to stream to OpenBCI GUI / LSL please read the below instructions**
  

1. You Must flash the special firmware listed here (Look inside folder above '"(Works ) WiFI Firmware (Device Host)" ) using the Arduino IDE. 

To flash: Download [Arduino Ide](http://arduino.cc/en/software/) , then config the Arduino IDE with the correct settings:

Board: Navigate to Tools > Board > ESP32 Arduino and select 'ESP32 WROOM DA Module'. Port: Navigate to Tools > Port and select the COM port corresponding to your Cerelog board.

2. You must then connect your computer to the device via the wifi hotspot it creates. The device will show up as network device "CERELOG_EEG" Type in **cerelog123**  when asked for a password;
   
3. To run OpenBCI GUI fork with this wifi LSL brige, follow this [Guide Link](https://github.com/Cerelog-ESP-EEG/How-to-use-OpenBCI-GUI-fork)  **but You must use the special WIFI LSL python connection script -> "Python_wifi_LSL.py" included here instead**


**Careful; For Legacy Usage: with the old OpenBCI forked GUI listed in Method C in the link above:
To use this wifi LSL bridge, you must comment out line 17 and uncomment line 18 in the Python_wifi_LSL.py script to have the correct scaling factor**

   Getting an error?: If you run the python script inside VS code on mac you need to allow local network connections in privacy settings so the code can talk to device otherwise you get a 65 error







