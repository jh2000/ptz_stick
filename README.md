# Description
This project let's you create a small&cheap wireless PTZ Controller for various applications.<br>
The code is currently for TRENDnetView EVO/Luxriot EVO, a central recording/viewing solution and their cameras (free but limited).<br>
The PTZ-Cameras i tested it with are TRENDnet TV-IP420P and TV-IP440PI.<br>
Since no special functions are used, it should work with almost every Luxriot EVO/TrendnetView Server.


~~In the initial release the camera IDs are hardcoded but should be queried from the Server in a future release.~~ The camera IDs are queried from the Server and if the PTZ-capabilities are set, PTZ-commands are being sent.<br>
The display shows some raw values aswell as calculated things, but kinda unsorted and is hard WIP.<br>
The stick can be calibrated to give a good zero position with a dead zone and linear movement.<br>
NVS is currently not in use but i intend to save the calibration/configuration or something similar to it.<br>


Initialy i wanted to use a 1602 LCD Display which didn't fit into the case. So there was no intention (and i think it just won't work) to get the camera picture onto the TFT. This needs to be done on separate hardware!<br>

I don't have access to the API Docs since they require a NDA.<br>
The Webserver does some non-standard things and ignores "Connection: close" for example, for which i couldn't use the default HTTP client. This might come from the digest-auth needed.<br>
Anonymous-PTZ won't work, you always need a valid user+password. You can grant PTZ-rights to Anonymous, but the Webserver endpoint still needs the valid credentials.<br>
The PTZ-Stick can be read at 2000+ FPS but it is very sensitive to noise then.<br>
An update rate to the server of up to 10 seems to be enough.<br>

# Features
* Compatible to TrendnetView + Luxriot EVO
* OTA-Updateable
* Direct ESP to Server connection (no https by now)
* No setup except Wifi+Server credentials needed
* Many pins left for other extensions
* Could run on battery quite some time
* no additional parts are required, but a few resistors and caps are what you want.
* almost same small delay/update rate as the monitor application

# Parts
AZ-Delivery ESP32-WROOM-32/NodeMCU or similar<br>
DollaTek 1.8" TFT Display ST7735S from https://www.amazon.de/dp/B07QJVG8QX<br>
TEKO 790.9 Stand Case from https://www.amazon.de/dp/B003A69O6C<br>
JH-D400X-R4 10K 4D PTZ-Stick from https://www.amazon.de/dp/B088W8D8SB<br>
KY-040 360° rotary controller from https://www.amazon.de/dp/B07B68H6R8<br>
USB-B to micro-USB Adapter from https://www.amazon.de/dp/B0C3HGHKTF<br>
short micro-usb cable from https://www.amazon.de/dp/B095JZSHXQ

# Libs
|Lib                                |Version | Name|
| :---                             | :---  | :--- |
|WiFi                               | 2.0.0  |  (Arduino Internal)|
|ESPmDNS                            | 2.0.0  | (Arduino Internal)|
|ArduinoOTA                         | 2.0.0  | (Arduino Internal)|
|Update                             | 2.0.0  | (Arduino Internal)|
|Wire                               | 2.0.0  | (Arduino Internal)|
|LittleFS                           | 2.0.0  | (Arduino Internal)|
|SPI                                | 2.0.0  | (Arduino Internal)|
|WiFiClientSecure                   | 2.0.0  | (Arduino Internal)|
|FS                                 | 2.0.0  | (Arduino Internal)|
|Adafruit GFX Library               | 1.11.9 | Arduino\libraries\Adafruit_GFX_Library|
|Adafruit BusIO                     | 1.14.5 | Arduino\libraries\Adafruit_BusIO|
|Adafruit ST7735 and ST7789 Library | 1.10.3 | Arduino\libraries\Adafruit_ST7735_and_ST7789_Library|
|Ai Esp32 Rotary Encoder            | 1.6    | Arduino\libraries\Ai_Esp32_Rotary_Encoder|
|NTPClient_Generic                  | 3.7.5  | Arduino\libraries\NTPClient_Generic|
|Time                               | 1.6.1  | Arduino\libraries\Time|
|Timezone_Generic                   | 1.10.1 | Arduino\libraries\Timezone_Generic|
|ArduinoJson                        | 6.21.4 | Arduino\libraries\ArduinoJson|
|ArduinoMD5                         | git    | Arduino\libraries\ArduinoMD5-master|

