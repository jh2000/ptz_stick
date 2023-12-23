This project let's you create a small&cheap wireless PTZ Controller for various applications.
The code is currently for TRENDnetView EVO, a central recording/viewing solution (free but a little limited).
The PTZ-Cameras i tested it with are TRENDnet TV-IP420P and TV-IP440PI.


Parts used:
AZ-Delivery ESP32-WROOM-32/NodeMCU or similar
DollaTek 1.8" TFT Display ST7735S from https://www.amazon.de/dp/B07QJVG8QX
TEKO 790.9 Stand Case from https://www.amazon.de/dp/B003A69O6C
JH-D400X-R4 10K 4D PTZ-Stick from https://www.amazon.de/dp/B088W8D8SB
KY-040 360° rotary controller from https://www.amazon.de/dp/B07B68H6R8
USB-B to micro-USB Adapter from https://www.amazon.de/dp/B0C3HGHKTF
short micro-usb cable from https://www.amazon.de/dp/B095JZSHXQ

In the initial release the camera IDs are hardcoded but should be queried from the Server in a future release.
The display shows some raw values aswell as calculated things, but kinda unsorted and is hard WIP.
The stick can be calibrated to give a good zero position with a dead zone and linear movement.
NVS is not in use but i intend to save the calibration or something similar to it.


Initialy i wanted to use a 1602 LCD Display which didn't fit into the case. So there is no intention (and i think it just won't work) to get the camera picture onto the TFT. This needs to be done on separate hardware!

Libs used:
WiFi                               2.0.0  (Arduino Internal) 
ESPmDNS                            2.0.0  (Arduino Internal)
ArduinoOTA                         2.0.0  (Arduino Internal)
Update                             2.0.0  (Arduino Internal)
Wire                               2.0.0  (Arduino Internal)
LittleFS                           2.0.0  (Arduino Internal)
SPI                                2.0.0  (Arduino Internal)
HTTPClient                         2.0.0  (Arduino Internal)
WiFiClientSecure                   2.0.0  (Arduino Internal)
FS                                 2.0.0  (Arduino Internal)
Adafruit GFX Library               1.11.9 Arduino\libraries\Adafruit_GFX_Library
Adafruit BusIO                     1.14.5 Arduino\libraries\Adafruit_BusIO
Adafruit ST7735 and ST7789 Library 1.10.3 Arduino\libraries\Adafruit_ST7735_and_ST7789_Library
Ai Esp32 Rotary Encoder            1.6    Arduino\libraries\Ai_Esp32_Rotary_Encoder
NTPClient_Generic                  3.7.5  Arduino\libraries\arduino_362215
Time                               1.6.1  Arduino\libraries\Time
Timezone_Generic                   1.10.1 Arduino\libraries\Timezone_Generic

