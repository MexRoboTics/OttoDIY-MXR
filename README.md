# OttoDIY-MXR
This is a version tested of OttoDIY PLUS and will change with the time. I am currently make some changes..

To chek the original project go to: https://github.com/OttoDIY/PLUS


# Description:
Here are modified libraries and sketch tested for Otto DIY robot, the capabilties are: Led Matrix Mouth, Bluetooth connection, two action mode buttons,and some more in development...

Compatibility with [Zowi App](https://play.google.com/store/apps/details?id=com.bq.zowi) or [Otto DIY App](https://play.google.com/store/apps/details?id=appinventor.ai_juanfelixmateos.OTTO_DIY_Bluetooth_Controller&hl=es App)

# Changes
In order to add more sensors and use the next Led Matrix:
![LED Matrix ](https://github.com/MexRoboTics/OttoDIY-MXR/blob/master/Matrix_Led_8x8_with_MAX7219.JPG) <!-- .element height="20%" width="20%" -->
I have made size modification to Otto parts.


# Bill of Materials
- Arduino Nano V3.0;
- Arduino Nano Shield I/O Extension Board Expansion
- Mini usb cable.
- HC-SR04 Ultrasound sensor.
- Mini servo MG90 9g x4 (each one should come with 2 pointed screws and one small screw also arm keys to attach legs and feet).
- passive Buzzer 12mm 5V
- Female to Female breadboard connectors cable 10cm x24.
- 4 AA Battery case    *
- 1.5V AA batteries x4.*
- Phillips screwdriver (important magnetized)
- micro Switch 8x8mm
- Touch sensors x3     *
- Sound sensor module
- Bluetooth module HC-05 or HC06 or BLE
- 3D printed head
- 3D printed body
- 3D printed leg x2
- 3D printed right foot
- 3D printed left foot

-  * you could replace them wit a LIPO battery and push buttons

# Give life to Otto-MXR (Programming)
- Download and Install Arduino IDE: [https://www.arduino.cc/en/Main/Software](https://www.arduino.cc/en/Main/Software)
- If you have a clone board download the next driver in order to recognize the board: [http://www.wch.cn/download/CH341SER_EXE.html](http://www.wch.cn/download/CH341SER_EXE.html) for Windows User, or [http://www.wch.cn/download/CH341SER_MAC_ZIP.html](http://www.wch.cn/download/CH341SER_MAC_ZIP.html) for MAC Users
- Copy all libraries to C:\Program Files (x86)\Arduino\libraries (or wherever your library folder is installed)
- Before you programming otto, sure that the module Bluetooth is correctly configured (Baud-rate for programming module is 9600 or 19200 or 38400, depends on a module):
  - **For Module HC-05: (38400?) 115200**
   1. Upload the sketch HC05_BT_config.ino to your Nano first, then disconnect Nano from USB
   1. Now connect BT to Nano as shown in diagram but do not connect VCC
   1. Plug in the USB to Nano and then connect VCC so BT enters AT mode. LED on BT should start to blink slower, about once every 2 seconds. (If this doesn't work, try holding the button on BT module while connecting VCC).
   1. Open serial monitor in IDE, set baud-rate to 9600 and line ending to Both NL & CR.
    1. Type AT then press enter; (if everything is right, BT should respond with OK and then enter following commands: 
      * AT+NAME=Zowi "setting the name"
      * AT+PSWD=1234 "pairing password"
      * AT+UART=115200,1,0 "baud rate"
      * AT+POLAR=1,0 "enabling STATE pin to be used as reset for programming arduino over BT"
  - **For HC-06: (9600?) 115200**
  * For HC-06 BT module things are a little simpler because module is always in AT command mode when not connected to anything. But the downside is that HC-06 module cannot be used to upload sketches to Arduino because it doesn't have reset. For configuring the module:
  1. upload the sketch HC06_BT_config.ino to your Nano first, then disconnect Nano from USB
  2.  Now connect BT to Nano as follow
    * Tx -> Rx
    * Rx -> Tx
    * VCC -> 5V
    * GND -> GND
  3. Power on your Nano and after about 10-15 seconds everything should be finished and your BT should be configured (LED13 should start blinking).
 
- The BT code(OTTO_BT_easy.ino and OTTO_BT.ino) has 115200 baud-rate so BT module must match that speed to be able to communicate with Arduino Nano via serial interface(UART).

- Now open the sketch OTTO_BT_Matrix_Led.ino
- Check the pins connection section, if you have different pins associated, change this connections in the file Otto.h
- Disconnect the VCC pin of Bluetooth module before upload the code
- Make sure in tools you have "Board: Arduino Nano" "Processor ATmega328" and your Otto is connected to the corresponding port
- Upload the code and connect the VCC pin of Bluetooth module
- Open the Zowi or Otto App and enjoy!

Share it in Otto builder Facebook group https://www.facebook.com/ottodiy/groups/

# License CC-BY-SA
[![License: CC BY-SA 4.0](https://licensebuttons.net/l/by-sa/4.0/80x15.png)](https://creativecommons.org/licenses/by-sa/4.0/)
Creative Commons License
Otto DIY by www.ottodiy.com is licensed under a [Creative Commons Attribution-ShareAlike 4.0 International License.](https://creativecommons.org/licenses/by-sa/4.0/).
