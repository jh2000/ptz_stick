#include <custom.h> // Includes my credentials outside git-repo, delete this line in your project!

// Wifi+OTA
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

// TFT-Display + Rotary Encoder:
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <AiEsp32RotaryEncoder.h> // https://github.com/igorantolic/ai-esp32-rotary-encoder  Lib for KY-040 Rotary-Encoders
//#include <esp32-hal-adc.h>

// Time:
#include <WiFiUdp.h>
#include <NTPClient_Generic.h> // https://github.com/khoih-prog/NTPClient_Generic
#include <Timezone_Generic.h>  // https://github.com/khoih-prog/Timezone_Generic
#include <TimeLib.h>           // https://github.com/PaulStoffregen/Time

// Functions:
#include <ArduinoJson.h>       // https://github.com/bblanchon/ArduinoJson
#include <MD5.h>               // https://github.com/tzikis/ArduinoMD5




//#include "nvs_flash.h"
//#include "nvs.h"
#define TIME_ZONE_OFFSET_HRS            (+1)

// ESP-Pin             TFT-Pin
#define TFT_CS        5        // HW-SPI  CS
#define TFT_RST       22       //         RES
#define TFT_DC        19       // HW-SPI  MISO
#define TFT_MOSI      23       // HW-SPI  SDA
#define TFT_SCLK      18       // HW-SPI  SCL
#define TFT_BACKLIGHT 21       //         BL
#define TFT_IDLE_TIMEOUT 60    // Time until display auto off, also disconnects from EVO-Server

#define AIN0      34 // ADC1-CH6 Pan
#define AIN1      35 // ADC1-CH7 Tilt
#define AIN2      32 // ADC1-CH4 Zoom
#define STICK_BTN 33 // PTZ-Stick Button
#define ENC0_DT   25
#define ENC0_CLK  26
#define ENC0_BTN  27
#define ROTARY_ENCODER_VCC_PIN 14
#define ROTARY_ENCODER_STEPS 4

#ifndef SSID_NAME
#define SSID_NAME "<insert wifi name here>"
#define SSID_PASS "<insert wifi password here>"
#endif
#ifndef OTA_PASS
#define OTA_PASS "<insert OTA-Password here>"
#endif
#ifndef EVO_HOST // Trendnet EVO WebServer Port
#define EVO_HOST "<host:port here>"
#define EVO_USER "<user here>"
#define EVO_PASS "<password here>"
#endif

const char* ssid = SSID_NAME;
const char* password = SSID_PASS;

struct ain_struct { // 300-650 485
  unsigned int ain_chan = 0;
  unsigned int ain_raw = 0;
  unsigned int ain_rev = 0;          // invert axis
  unsigned int ain_min = 390;        // 1500; // Lower limit (ain_val = -100)
  unsigned int ain_dead_low = 460;   // 2100; // Lower Deadzone (ain_val = -1)
  unsigned int ain_dead_high = 520;  // 2200; // Upper Deadzone (ain_val = +1)
  unsigned int ain_max = 600;        // 2800; // Upper limit (ain_val = +100)
  int ain_val = 0;
};

ain_struct ain[3];
unsigned int Ain_clicked = 0;
//unsigned int Enc0_clicked = 0;

struct cam_struct {
  String title;
  int id;
  int ptzCapabilities;
};
int num_cams = 0;
cam_struct cam_data[99];
struct digest_struct {
  String cnonce;
  String nonce;
  String HA1;
  String AuthMethod;
  String realm;
  unsigned int nc = 1;
  String qop;
  String opaque;
};
digest_struct digest;

// String cnonce = "";
// String nonce = "";
String tmpstring = "";
// String HA1 = "";
String HA2 = "";
String authResponse = "";
String authHeaderString = "";
String AuthMethod = "";
// String realm = "";
// String qop = "";
// String opaque = "";

long Enc0_value = 0;
int httpCode = 0;
int fps = 0;
int fps_last = 0;
unsigned int chan = 0;
unsigned int idle = 0;
unsigned int lcd_menu = 0;
unsigned int lcd_change = 1;
unsigned int secs = 0;
unsigned int secs_old = 0;
int wifistatus = 0;
int wifistatus_old = 0;
unsigned int total_buf = 0;
unsigned int progress_buf = 0;
unsigned int progress_old = 0;

int P_old = 0;
int T_old = 0;
int Z_old = 0;
int cam_select = 0; // 111 hinten, 115 vorne
char tmpchar[150];
const char *headerKeys[] = {"WWW-Authenticate"};
size_t numberOfHeaders = sizeof(headerKeys) / sizeof(char*);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     // Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       // Central European Standard Time
Timezone *CE;
time_t t;
DynamicJsonDocument doc(1024);
int16_t textsize_x, textsize_y;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); // 128x160
GFXcanvas16 canvas(128, 160); // 16-bit
AiEsp32RotaryEncoder Encoder0 = AiEsp32RotaryEncoder(ENC0_CLK, ENC0_DT, ENC0_BTN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
WiFiClient evo_client;

//EEPROMClass  TRIMDATA("eeprom0");

//nvs_handle_t ain_trim;

esp_err_t err;
void IRAM_ATTR readEncoderISR()
{
  Encoder0.readEncoder_ISR();
}

void lcd_print(String text, int len = 26) {
  tft.printf("%-*s ", len, text.c_str());
}
void setup() {
  Serial.begin(115200);
  /*  err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    err = nvs_open("storage", NVS_READWRITE, &ain_trim);
    if (err != ESP_OK) {
      Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
      Serial.printf("Done\n");
  */
  ain[0].ain_chan = AIN0;
  ain[1].ain_chan = AIN1;
  ain[2].ain_chan = AIN2;
  ain[0].ain_rev = true;
  ain[1].ain_rev = true;
  /* size_t ain_size;
    err = nvs_get_blob(ain_trim, "ain_trim", &ain, &ain_size);
    switch (err) {
     case ESP_OK:
       Serial.printf("Done\n");
       break;
     case ESP_ERR_NVS_NOT_FOUND:
       Serial.printf("The value is not initialized yet!\n");
       break;
     default :
       Serial.printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
    }
  */
  Serial.println("Bootup...");
  analogReadResolution(10);
  //analogSetAttenuation(ADC_0db);
  adcAttachPin(AIN0);
  adcAttachPin(AIN1);
  adcAttachPin(AIN2);
  Serial.println("LCD...");
  tft.setSPISpeed(40000000);
  tft.initR(INITR_GREENTAB);
  pinMode(TFT_BACKLIGHT, OUTPUT);
  tft.setRotation(1);
  digitalWrite(TFT_BACKLIGHT, HIGH);
  delay(100);
  /*uint16_t textpos_w;
    uint16_t textpos_h;
    char onecharstring[2] = {"X"};
    tft.getTextBounds(onecharstring,0,0,&textsize_x,&textsize_y, &textpos_w, &textpos_h);*/
  textsize_x = 6;
  textsize_y = 8;
  tft.setTextWrap(false); // Allow text to run off right edge
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.fillScreen(ST77XX_BLACK);
  Serial.print("Height ");
  Serial.println(tft.height(), DEC);
  Serial.print("Width ");
  Serial.println(tft.width(), DEC);
  Serial.print("Textsize X ");
  Serial.println(textsize_x, DEC);
  Serial.print("Textsize Y ");
  Serial.println(textsize_y, DEC);
  delay(1000);
  tft.fillScreen(ST77XX_BLACK);
  setCursor(0, 0);
  lcd_print("PTZ-Controller");
  setCursor(0, 1);
  lcd_print("WIFI");
  Serial.println("Wifi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("OTA...");
  setCursor(0, 1);
  lcd_print("OTA");
  ArduinoOTA.setHostname("ESP32_PTZ");
  ArduinoOTA.setPassword(OTA_PASS);
  // put your setup code here, to run once:
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    tft.fillScreen(ST77XX_YELLOW);
    delay(10);
    lcd_bl(1);
    setCursor(0, 0);
    lcd_print("Updating " + String(type));
  })
  .onEnd([]() {
    tft.fillScreen(ST77XX_GREEN);
    setCursor(0, 1);
    lcd_print("Success");
    delay(1000);
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    if (total_buf == 0) total_buf = total / 100;
    progress_buf = progress / total_buf;
    if (progress_buf != progress_old) {
      progress_old = progress_buf;
      setCursor(0, 1);
      tft.printf("Progress: %u%%", (progress / (total / 100)));
    }
  })
  .onError([](ota_error_t error) {
    tft.fillScreen(ST77XX_RED);
    Serial.printf("Error[%u]: ", error);
    setCursor(0, 1);
    if (error == OTA_AUTH_ERROR) lcd_print("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) lcd_print("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) lcd_print("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) lcd_print("Receive Failed");
    else if (error == OTA_END_ERROR) lcd_print("End Failed");
    delay(5000);
  });

  ArduinoOTA.begin();

  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  setCursor(0, 1);
  lcd_print("Outputs");
  Serial.println("Outputs...");
  pinMode(ENC0_BTN, INPUT_PULLUP);
  pinMode(STICK_BTN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(STICK_BTN), Stick_Click, RISING);
  //attachInterrupt(digitalPinToInterrupt(ENC0_BTN), Enc0_Click, RISING);
  Encoder0.begin();
  Encoder0.setup(readEncoderISR);
  Encoder0.setBoundaries(0, 20, true);
  Encoder0.disableAcceleration();
  //Encoder0.setAcceleration(250);
  setCursor(0, 1);
  lcd_print("NTP");
  CE    = new Timezone(CEST, CET);
  timeClient.begin();
  timeClient.setTimeOffset(3600);
  Serial.println("Bootup done.");
  setCursor(0, 1);
  lcd_print("Boot done");
  // http.setReuse(true);
  delay(300);
  // lcd.clear();
  tft.fillScreen(ST77XX_BLACK);
  setCursor(0, 0);
  // Fill whole display:
  /*tft.println("1BCDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("2BCDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("3BCDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("4BCDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("5BCDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("6BCDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("7BCDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("8BCDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("9BCDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("10CDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("11CDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("12CDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("13CDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("14CDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("15CDEFGHIJKLMNOPQRSTUVWXYZ");
    tft.println("16CDEFGHIJKLMNOPQRSTUVWXYZ");*/
}

void loop() {
  secs = millis() / 1000;
  wifistatus = WiFi.status();

  if (wifistatus == WL_CONNECTED) {
    ArduinoOTA.handle();
    timeClient.update();
    //if (timeClient.isTimeSet()) {
    //          sprintf(buf, "%d-%d-%.2d %.2d:%.2d:%.2d",
    //                year(t), month(t), day(t), hour(t), minute(t), second(t));
  }
  refresh_analog();
  button_loop();
  if (secs - idle > TFT_IDLE_TIMEOUT) {
    if (lcd_status() == 1) {
      lcd_bl(0);
      if (evo_client.connected()) evo_client.stop();
    }
  }
  else {
    lcd_loop();
    if (lcd_status() == 0) lcd_bl(1);
  }
}

int lcd_status () {
  return digitalRead(TFT_BACKLIGHT);
}
void lcd_bl (int lcd_backlight) {
  digitalWrite(TFT_BACKLIGHT, lcd_backlight);
}
void alive(int soft = 0) {
  idle = secs;
  if (soft == 0) lcd_change = 1;
}
void button_loop() {
  if (Ain_clicked == 1) {
    /*  err = nvs_set_blob(ain_trim, "ain_trim", ain, sizeof(ain));
      Serial.printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      err = nvs_commit(ain_trim);
      Serial.printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    */
    PTZ(cam_select, 0, 0, 0);
    /*
      if (cam_select == 115)
        cam_select = 111;
      else cam_select = 115;*/
    if (num_cams == 0) cam_select = 0;
    else {
      cam_select++;
      Serial.print("Select increased...");
      Serial.print(cam_select);
      Serial.print(" of ");
      Serial.println(num_cams);
      if (cam_select > num_cams) {
        Serial.println("Back to 1");
        cam_select = 1;
      }
    }
    Ain_clicked = 0;

    alive();
  }
  if (Encoder0.encoderChanged())
  {


    chan = Encoder0.readEncoder();
    lcd_menu = chan;
    //   Serial.print("Value: ");
    //   Serial.println(chan);
    alive();
  }
  if (Encoder0.isEncoderButtonClicked()) {
    //switch lcd_menu
    switch (lcd_menu) {
      case 4:
        HttpSendDigest("/channels");
        break;
      default:
        for (int i = 0; i < 3; i++) {
          ain[i].ain_dead_low  = ain[i].ain_raw - 25;
          ain[i].ain_dead_high = ain[i].ain_raw + 25;
          ain[i].ain_min = ain[i].ain_raw - 50;
          ain[i].ain_max = ain[i].ain_raw + 50;
        }
    }

    // HttpSendDigest("http://" + EVO_HOST + "/ptz/111/zoom?speed=-1", "/ptz/111/zoom?speed=1", EVO_USER, EVO_PASS);
    alive();
  }
}

void lcd_loop() {
  //lcd_change = 1;
  if (secs != secs_old) {
    secs_old = secs;
    fps_last = fps;
    fps = 0;
    lcd_change = 1;
  }
  if (lcd_change) {
    t = CE->toLocal(timeClient.getUTCEpochMillis() / 1000);
    setCursor(18, 0);
    tft.printf("%.2d:%.2d:%.2d", hour(t), minute(t), second(t));;
    setCursor(18, 5);
    lcd_print(String(secs - idle), 6);
    setCursor(0, 14);
    lcd_print(String("FPS ") + String(fps_last));
    setCursor(0, 0);
    tft.print("CAM: ");
    if (cam_select > 0) {
      lcd_print(String(cam_data[cam_select].id), 4);
    }
    else {
      lcd_print("N/A", 4);
    }
    switch (lcd_menu) {
      case 0: {
          setCursor(0, 1);
          lcd_print(String("0 ") + String(ain[0].ain_raw) + String(" ") + String(ain[0].ain_val));
          break;
        }
      case 1: {
          setCursor(0, 1);
          lcd_print(String("1 ") + String(ain[1].ain_raw) + String(" ") + String(ain[1].ain_val));
          break;
        }
      case 2: {
          setCursor(0, 1);
          lcd_print(String("2 ") + String(ain[2].ain_raw) + String(" ") + String(ain[2].ain_val));
          break;
        }
      case 3: {
          setCursor(0, 4);
          lcd_print("Cams:");
          setCursor(0, 5);
          lcd_print(String(num_cams));
          if (num_cams > 0 && cam_select > 0) {
            setCursor(0, 6);
            lcd_print("Cam ID:");
            setCursor(0, 7);
            lcd_print(String(cam_data[cam_select].id));
            setCursor(0, 8);
            lcd_print("Cam Title:");
            setCursor(0, 9);
            lcd_print(cam_data[cam_select].title);
            setCursor(0, 10);
            lcd_print("Cam PTZ Features:");
            setCursor(0, 11);
            lcd_print(String(cam_data[cam_select].ptzCapabilities));
          }
          break;
        }
      case 4: {
          setCursor(0, 1);
          lcd_print("Refresh cams");
          break;
        }
      case 5: {
          setCursor(0, 1);
          lcd_print("STOP");
          setCursor(6, 1);
          lcd_print(String(PTZ(cam_select, 0, 0, 0)));
          break;
        }
      case 6: {
          setCursor(0, 1);
          lcd_print("GO");
          setCursor(0, 2);
          lcd_print(String("P") + String(ain[1].ain_val) + String(" T") + String(ain[0].ain_val) + String(" Z") + String(ain[2].ain_val));
          break;
        }
      case 7: {
          setCursor(0, 1);
          lcd_print("STOP");

          setCursor(6, 1);
          lcd_print(String(PTZ(cam_select, 0, 0, 0)));
          break;
        }
      case 9: {
          setCursor(0, 1);
          lcd_print(String("IP ") + String(WiFi.localIP().toString()));
          break;
        }
      case 10: {
          setCursor(0, 1);
          lcd_print(String("RSSI ") + String(WiFi.RSSI()));
          break;
        }
      case 11: {
          setCursor(0, 1);
          lcd_print(String("WiFi Status ") + String(WiFi.status()));
          break;
        }
      case 12: {
          setCursor(0, 1);
          lcd_print(String("FPS ") + String(fps));
          break;
        }
      default: {
          setCursor(0, 1);
          Serial.println(String("Menü error: ") + String(lcd_menu));
          lcd_print(String("Menu error: ") + String(lcd_menu));
        }
    }
  }
  lcd_change = 0;
}

int PTZ (int cam, int P, int T, int Z) {
  char httpRequestData[50];
  httpCode = 299;
  if (cam > 0 && cam_data[cam].ptzCapabilities > 10) {
    if (P != P_old || T != T_old) {
      if (P > 90) P = 100;
      if (T > 90) T = 100;
      if (P < -90) P = -100;
      if (T < -90) T = -100;
      P_old = P;
      T_old = T;
      float Pf = P / 100.0;
      float Tf = T / 100.0;
      sprintf(httpRequestData, "/ptz/%d/pantilt?pan=%.2f&tilt=%.2f", cam_data[cam].id, Pf, Tf);
      httpCode = HttpSendDigest(String(httpRequestData));
      lcd_change = 1;
    }
    if (Z != Z_old) {
      if (Z > 90) Z = 100;
      if (Z < -90) Z = -100;
      Z_old = Z;
      float Zf = Z / 100.0;
      sprintf(httpRequestData, "/ptz/%d/zoom?speed=%.2f", cam_data[cam].id, Zf);
      httpCode = HttpSendDigest(String(httpRequestData));
      lcd_change = 1;
      return httpCode;
    }
  }
  else {
    return 298; // No camera selected / PTZ-Supported
  }
}
void Stick_Click() {
  Ain_clicked = 1;
}
/*void Enc0_Click() {
  Enc0_clicked = 1;
  }*/

void setCursor(int col, int row) {
  tft.setCursor(col * textsize_x, row * textsize_y);
}

void refresh_analog() {
  delay(10); // Wait to reduce noise from other things.
  fps++;
  for (int i = 0; i < 3; i++) {
    ain[i].ain_raw = analogRead(ain[i].ain_chan); // ain[i].ain_chan
    if (ain[i].ain_raw > ain[i].ain_dead_high) {
      ain[i].ain_val = map(ain[i].ain_raw, ain[i].ain_dead_high, ain[i].ain_max, 1, 100);
      if (ain[i].ain_val > 5) alive(1);
    }
    else if (ain[i].ain_raw < ain[i].ain_dead_low) {
      ain[i].ain_val = map(ain[i].ain_raw, ain[i].ain_min, ain[i].ain_dead_low, -100, -1);
      if (ain[i].ain_val < -5) alive(1);
    }
    else
    {
      ain[i].ain_val = 0;
    }
    if (ain[i].ain_rev) ain[i].ain_val = ain[i].ain_val * -1;
    //   Serial.println("Analog Channel " + String(i) + String(" Raw ") + String(analogRead(ain[i].ain_chan)) + String(" Val: ") + String(ain[i].ain_val));
    if (ain[i].ain_raw > ain[i].ain_max) {
      //   Serial.println(String("Value overflow @chan ") + String(i) + String(" Value ") + String(ain[i].ain_raw) + String(" Max ") + String(ain[i].ain_max));
      ain[i].ain_max = ain[i].ain_raw;
    }
    if (ain[i].ain_raw < ain[i].ain_min) {
      //  Serial.println(String("Value underrun @chan ") + String(i) + String(" Value ") + String(ain[i].ain_raw) + String(" Min ") + String(ain[i].ain_min));
      ain[i].ain_min = ain[i].ain_raw;
    }
  }
  if (lcd_menu == 6) {
    PTZ(cam_select, ain[1].ain_val, ain[0].ain_val, ain[2].ain_val);
    //    delay(333);
  }
}


void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  /*  Serial.println("Disconnected from WiFi access point");
    Serial.print("WiFi lost connection. Reason: ");
    Serial.println(info.wifi_sta_disconnected.reason);
    Serial.println("Trying to Reconnect");*/
  WiFi.begin(ssid, password);
}

bool HttpSendDigest(String uri)
{
  int readauth = 0;
  int ptz_request;
  if (uri.substring(0, 9) != "/channels") ptz_request = 1; else ptz_request = 0;
  int body_count = 0;
  String user = EVO_USER;
  String password = EVO_PASS;
  char host[] = EVO_HOST;
  int    port = EVO_PORT;
  /* Serial.print("URI: ");
    Serial.println(uri);
    Serial.print("Host: ");
    Serial.println(host);
    Serial.print("Port: ");
    Serial.println(port); */
  if (!evo_client.connected()) {
    if (evo_client.connect(host, port)) {
      Serial.println("Connect OK");
      evo_client.print("GET ");
      evo_client.print(uri);
      evo_client.println(" HTTP/1.1");
      evo_client.print("Host: ");
      evo_client.println(host);
      evo_client.println("User-Agent: ESP32-PTZ/1.0.0");   // Optional ?
      evo_client.println("Accept: */*");               // Optional ?
      evo_client.println("Accept-Encoding: identity"); // Optional ?
      evo_client.println();
      digest.nc = 1;
      readauth = 0;
    }
    else {
      Serial.println("Connect Error");
    }
  }
  else {
    sprintf(tmpchar, "GET:%s", uri.c_str());
    //tmpstring = String("GET:") + String(uri);
    //   Serial.println(tmpstring);
    HA2 = md5(String(tmpchar));
    //   Serial.println(HA2);
    //digest.cnonce = String(random(8556824323));
    //   Serial.println("cnonce: " + String(cnonce));
    //   Serial.println("Calculate authResponse...");
    //tmpstring = digest.HA1 + ":" + String(digest.nonce) + ":00000001:" + String(digest.cnonce) + ":" + String(digest.qop) + ":" + HA2;
    sprintf(tmpchar, "%s:%s:%08d:%s:%s:%s", digest.HA1.c_str(), digest.nonce.c_str(), digest.nc, digest.cnonce.c_str(), digest.qop.c_str(), HA2.c_str());
    //   Serial.println(tmpstring);
    authResponse = md5(String(tmpchar)); //MD5(HA1:nonce:nonceCount:cnonce:qop:HA2)
    //   Serial.println("authResponse: " + String(authResponse));

    evo_client.print("GET ");
    evo_client.print(uri);
    evo_client.println(" HTTP/1.1");
    evo_client.print("Host: ");
    evo_client.println(host);
    evo_client.println("User-Agent: ESP32-PTZ/1.0.0");   // Optional ?
    evo_client.println("Accept: */*");               // Optional ?
    evo_client.println("Accept-Encoding: identity"); // Optional ?
    evo_client.printf("Authorization: Digest username=\"%s\", realm=\"%s\", nonce=\"%s\", uri=\"%s\", response=\"%s\", qop=%s, nc=%08d, cnonce=\"%s\", opaque=\"%s\"", user.c_str(), digest.realm.c_str(), digest.nonce.c_str(), uri.c_str(), authResponse.c_str(), digest.qop.c_str(), digest.nc, digest.cnonce, digest.opaque.c_str());
    evo_client.println();
    evo_client.println();
    digest.nc++;
    readauth = 1;
  }

  int em_exit = 0;
  while (evo_client.connected()) {
    em_exit++;
    if (em_exit > 5000) {
      evo_client.stop();
      Serial.println("Emergency stop1 reached");
    }
    String response = "";
    while (evo_client.available()) {
      char c = evo_client.read();
      response += String(c);
      if (response.indexOf("\r\n") != -1) break;
    }
    if (response.startsWith("HTTP/1.1 401")) { // Needs to (re)-authenticate.
      Serial.println("(Re)auth");
      readauth = 1;
    }
    else if (response.startsWith("HTTP/1.1 200")) {
      readauth = 2;
    }
    else if (response.startsWith("HTTP/1.1 204")) {
      return 0;
    }
    else if (response.startsWith("HTTP/1.1 ")) { // Busy, Error, Full, ...
      Serial.println("Unexpected response: ");
      Serial.println(response);
      evo_client.stop();
    }
    else if (response.startsWith("WWW-Authenticate")) {
      digest.AuthMethod = "Digest";
      tmpstring = response.substring(7 + response.indexOf("realm=\""));
      digest.realm = tmpstring.substring(0, tmpstring.indexOf("\""));
      tmpstring = response.substring(7 + response.indexOf("nonce=\""));
      digest.nonce = tmpstring.substring(0, tmpstring.indexOf("\""));
      tmpstring = response.substring(8 + response.indexOf("opaque=\""));
      digest.opaque = tmpstring.substring(0, tmpstring.indexOf("\""));
      tmpstring = response.substring(5 + response.indexOf("qop=\""));
      digest.qop = tmpstring.substring(0, tmpstring.indexOf("\""));

      /*         Serial.println("AuthMethod: " + String(digest.AuthMethod));
               Serial.println("realm: " + String(digest.realm));
               Serial.println("nonce: " + String(digest.nonce));
               Serial.println("qop: " + String(digest.qop));
               Serial.println("opaque: " + String(digest.opaque));
               Serial.print("Calculate HA1...");*/
      digest.HA1 = md5(user + ":" + digest.realm + ":" + password);
      //Serial.println(digest.HA1);
      //Serial.print("Calculate HA2... ");
      tmpstring = String("GET:") + String(uri);
      //Serial.println(tmpstring);
      HA2 = md5(tmpstring);
      //Serial.println(HA2);

      digest.cnonce = String(random(8556824323));
      sprintf(tmpchar, "%s:%s:%08d:%s:%s:%s", digest.HA1.c_str(), digest.nonce.c_str(), digest.nc, digest.cnonce.c_str(), digest.qop.c_str(), HA2.c_str());
      authResponse = md5(String(tmpchar));
      //   Serial.println("authResponse: " + String(authResponse));
      evo_client.print("GET ");
      evo_client.print(uri);
      evo_client.println(" HTTP/1.1");
      evo_client.print("Host: ");
      evo_client.println(host);
      evo_client.println("User-Agent: Wget/1.21.4");   // Optional ?
      evo_client.println("Accept: */*");               // Optional ?
      evo_client.println("Accept-Encoding: identity"); // Optional ?
      //evo_client.println(authHeaderString);
      //      Serial.printf("Authorization: Digest username=\"%s\", realm=\"%s\", nonce=\"%s\", uri=\"%s\", response=\"%s\", qop=%s, nc=%08d, cnonce=\"%s\", opaque=\"%s\"", user.c_str(), digest.realm.c_str(), digest.nonce.c_str(), uri.c_str(), authResponse.c_str(), digest.qop.c_str(), digest.nc, digest.cnonce, digest.opaque.c_str());
      //      Serial.println();
      evo_client.printf("Authorization: Digest username=\"%s\", realm=\"%s\", nonce=\"%s\", uri=\"%s\", response=\"%s\", qop=%s, nc=%08d, cnonce=\"%s\", opaque=\"%s\"", user.c_str(), digest.realm.c_str(), digest.nonce.c_str(), uri.c_str(), authResponse.c_str(), digest.qop.c_str(), digest.nc, digest.cnonce, digest.opaque.c_str());
      evo_client.println();
      evo_client.println();
      digest.nc++;
    }
    else if (response.length() == 2) {
      //  Serial.println("Auth Header done");
      if (readauth == 0) {
        evo_client.stop();
      }
      else if (readauth == 2) {
        //  Serial.println("Request Header done");
        if (ptz_request == 1) {
          //    Serial.println("PTZ Request done");
          return 0;
        }
        readauth = 3;
      }
      else {
        //     Serial.println("End of ...");
        //     Serial.println(readauth);
      }
    }
    else if (readauth == 3) {
      body_count++;
      if (body_count > 100) {
        evo_client.stop();
        Serial.println("Emergency stop B reached");
      }
      if (response.startsWith("{\"added")) { // JSON, we only need the "added" json, not the following "updated" ones
        StaticJsonDocument<200> filter;
        filter["added"]["channels"][0]["id"] = true;
        filter["added"]["channels"][0]["title"] = true;
        filter["added"]["channels"][0]["ptzCapabilities"] = true;


        deserializeJson(doc, response, DeserializationOption::Filter(filter));
        Serial.print("Got ");
        Serial.print(doc["added"]["channels"].size());
        Serial.println(" Cameras");
        JsonArray added_channels = doc["added"]["channels"];
        num_cams = doc["added"]["channels"].size();
        for (int i = 0; i < added_channels.size(); i++) {
          JsonObject channel_data = added_channels[i];
          cam_data[i + 1].id = channel_data["id"];
          cam_data[i + 1].title = String((const char*) channel_data["title"]);
          cam_data[i + 1].ptzCapabilities = channel_data["ptzCapabilities"];
          Serial.print("Camera ");
          Serial.print(i);
          Serial.println(":");
          Serial.println((int)channel_data["id"]);
          Serial.println((const char*)channel_data["title"]);
          Serial.println((int)channel_data["ptzCapabilities"]);
        }
        serializeJsonPretty(doc, Serial);
      }
      else if (response.length() > 0) {
        Serial.println("Response: ");
        Serial.println(response);
      }
      //Serial.println("Body received");
    }
    else if (response.length() > 0) {
      //  Serial.println("Response: ");
      //  Serial.println(response);
    }
  }

  //  Serial.println("---Done---");
  return false;
}

String md5(String text) {
  char cstr[150]; // 121 minimum
  text.toCharArray(cstr, text.length() + 1);
  unsigned char* hash;
  hash = MD5::make_hash(cstr);
  char *md5str = MD5::make_digest(hash, 16);
  return md5str;
}
