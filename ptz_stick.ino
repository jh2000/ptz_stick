#include <custom.h> // Includes credentials outside git-repo
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <HTTPClient.h>
#include <MD5Builder.h>
#include <AiEsp32RotaryEncoder.h>
#include <esp32-hal-adc.h>
#include <NTPClient_Generic.h>
#include <Timezone_Generic.h>
#include <TimeLib.h>
#include <SPI.h>



#include "nvs_flash.h"
#include "nvs.h"
#define TIME_ZONE_OFFSET_HRS            (+1)

                   // ESP-Pin             TFT-Pin
#define TFT_CS        5        // HW-SPI  CS
#define TFT_RST       22       //         RES
#define TFT_DC        19       // HW-SPI  MISO
#define TFT_MOSI      23       // HW-SPI  SDA
#define TFT_SCLK      18       // HW-SPI  SCL
#define TFT_BACKLIGHT 21       //         BL

#define AIN0      34 // ADC1-CH6
#define AIN1      35 // ADC1-CH7
#define AIN2      32 // ADC1-CH4
#define STICK_BTN 33
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
#ifndef EVO_IP // Trendnet EVO WebServer Port
#define EVO_IP "<host:port here>"
#define EVO_USER "<user here>"
#define EVO_PASS "<password here>"
#endif

const char* ssid = SSID_NAME;
const char* password = SSID_PASS;

struct ain_struct {
  unsigned int ain_chan = 0;
  unsigned int ain_raw = 0;
  unsigned int ain_min = 390;        // 1500; // Unterer Anschlag
  unsigned int ain_dead_low = 520;   // 2100; // Niedriger Totpunkt (Mitte - 50)
  unsigned int ain_dead_high = 540;  // 2200; // Hoher Totpunkt (Mitte + 50)
  unsigned int ain_max = 670;        // 2800; // Oberer Anschlag
  int ain_val = 0;
};

ain_struct ain[3];
unsigned int Ain_clicked = 0;
unsigned int Enc0_clicked = 0;

long Enc0_value = 0;
int httpCode = 0;
int fps = 0;
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
int cam_select = 111; // 111 hinten, 115 vorne

const char *headerKeys[] = {"WWW-Authenticate"};
size_t numberOfHeaders = sizeof(headerKeys) / sizeof(char*);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     // Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       // Central European Standard Time
Timezone *CE;
time_t t;
int16_t textsize_x, textsize_y;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); // 128x160
GFXcanvas16 canvas(128, 160); // 16-bit, 120x30 pixels
WiFiClient client;
HTTPClient http; // = HttpClient("10.0.83.9",8080);
AiEsp32RotaryEncoder Encoder0 = AiEsp32RotaryEncoder(ENC0_CLK, ENC0_DT, ENC0_BTN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

//EEPROMClass  TRIMDATA("eeprom0");
nvs_handle_t ain_trim;
esp_err_t err;
void IRAM_ATTR readEncoderISR()
{
  Encoder0.readEncoder_ISR();
}

void lcd_print(String text, int len = 26) {
  tft.printf("%-*s ", len, text);
}
void setup() {
  Serial.begin(115200);
  err = nvs_flash_init();
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
    ain[0].ain_chan = AIN0;
    ain[1].ain_chan = AIN1;
    ain[2].ain_chan = AIN2;
    size_t ain_size;
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

  Serial.println("Bootup...");
  analogReadResolution(10);
  //analogSetAttenuation(ADC_0db);
  //analogSetCycles(255);
  adcAttachPin(AIN0);
  adcAttachPin(AIN1);
  adcAttachPin(AIN2);
  Serial.println("LCD...");
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
  tft.setTextWrap(true); // Allow text to run off right edge
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
    // lcd.clear();
    // lcd.backlight();
    setCursor(0, 0);
    lcd_print("Updating " + type);
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
  http.setReuse(true);
  delay(300);
  // lcd.clear();
  tft.fillScreen(ST77XX_BLACK);
  setCursor(0, 0);
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

  if (secs - idle > 30) {
    if (lcd_status() == 1) {
      lcd_bl(0);
    }
  }
  else {
    if (lcd_status() == 0) {
      lcd_bl(1);
    }
  }

  lcd_loop();



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
    if (cam_select == 115)
      cam_select = 111;
    else cam_select = 115;

    Ain_clicked = 0;

    // HttpSendDigest("http://" + EVO_HOST + "/ptz/111/zoom?speed=1", "/ptz/111/zoom?speed=1", EVO_USER, EVO_PASS);
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

    for (int i = 0; i < 3; i++) {
      ain[i].ain_dead_low  = ain[i].ain_raw - 10;
      ain[i].ain_dead_high = ain[i].ain_raw + 10;
      ain[i].ain_min = ain[i].ain_raw - 55;
      ain[i].ain_max = ain[i].ain_raw + 55;
    }
    // HttpSendDigest("http://" + EVO_HOST + "/ptz/111/zoom?speed=-1", "/ptz/111/zoom?speed=1", EVO_USER, EVO_PASS);
    alive();
  }
}

void lcd_loop() {
  if (lcd_change || secs != secs_old) {
    // lcd.clear();
    t = CE->toLocal(timeClient.getUTCEpochMillis() / 1000);
    setCursor(18, 0);
    tft.printf("%.2d:%.2d:%.2d", hour(t), minute(t), second(t));;

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
      case 5: {
          setCursor(0, 2);
          lcd_print(String("STOP"));

          setCursor(6, 2);
          lcd_print(String(PTZ(cam_select, 0, 0, 0)));
          break;
        }
      case 6: {
          setCursor(0, 0);
          tft.print(String(cam_select));
          setCursor(0, 1);
          lcd_print(String("GO P"));
          //PTZ(cam_select,ain[1].ain_val, ain[0].ain_val, ain[2].ain_val);
          setCursor(4, 3);
          lcd_print(String(ain[1].ain_val) + String(" T") + String(ain[0].ain_val) + String(" Z") + String(ain[2].ain_val));
          break;
        }
      case 7: {
          setCursor(0, 1);
          lcd_print(String("STOP"));

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
    fps = 0;
  }
  secs_old = secs;
  lcd_change = 0;
}

int PTZ (int cam, int P, int T, int Z) {
  if (P != P_old || T != T_old || Z != Z_old) {
    if (P > 90) P = 100;
    if (T > 90) T = 100;
    if (Z > 90) Z = 100;
    P_old = P;
    T_old = T;
    Z_old = Z;
    http.begin(client, "http://10.0.83.1/ponvif/ptz3.php");
    http.addHeader("Connection", "close");
    //http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String httpRequestData = "cam=";
    httpRequestData += cam;
    httpRequestData += "&P=";
    httpRequestData += P;
    httpRequestData += "&T=";
    httpRequestData += T;
    httpRequestData += "&Z=";
    httpRequestData += Z;
    // Serial.println(httpRequestData);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int httpCode = http.POST(httpRequestData);
    // Serial.println(http.errorToString(httpCode).c_str());
    // Serial.println(httpCode);
    // http.end();
  }
  else {
    httpCode = 299;
  }
  return httpCode;
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
  fps++;
  //Serial.println(String(fps));
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
    delay(333);
  }
}


void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  /*  Serial.println("Disconnected from WiFi access point");
    Serial.print("WiFi lost connection. Reason: ");
    Serial.println(info.wifi_sta_disconnected.reason);
    Serial.println("Trying to Reconnect");*/
  WiFi.begin(ssid, password);
}
/*
  bool HttpSendDigest(String uri, String path, String user, String password)
  {

  Serial.println("HttpSendDigest::GET request");
  http.begin(uri);
  // http.get(uri);
  http.collectHeaders(headerKeys, numberOfHeaders);
  int statusCode = http.GET();

  String header = "";
  // read the status code and body of the response
  //int statusCode = http.responseStatusCode();
  /*  if (http.headerAvailable()){
      while(!http.endOfHeadersReached()){
        header = header + char(http.readHeader());
      }
    }
  Serial.println("Num-Headers: " + String(http.headers()));
  for (int i = 0; i < http.headers(); i++) {
    Serial.println("Headers: " + String(http.header(i)) + String(" -> ") + String(http.headerName(i)));
  }
  String headerValue = http.header("WWW-Authenticate");
  //String headerValue = http.readHeaderValue();
  String response = http.getString();

  //Serial.println("HttpSendDigest::Headername: "+ headerName);
  Serial.println("HttpSendDigest::Headervalue: " + String(headerValue));
  Serial.print("HttpSendDigest::statuscode: ");
  Serial.println(statusCode);
  //Serial.println("HttpSendDigest::response: ");
  //Serial.println(response);


  if (statusCode == 401)
  {
    Serial.println("HttpSendDigest::401+WWW-Authenticate detected");
    // String AuthMethod = headerValue.substring(0, headerValue.indexOf(' '));
    String AuthMethod = "Digest";
    String realm = "CMSSystem"; //headerValue.substring()   strGetValue(headerValue,"realm=\"","\"");
    String nonce = headerValue.substring(7 + headerValue.indexOf("nonce=\""), 7 + headerValue.indexOf("nonce=\"") + 32);
    String opaque = headerValue.substring(8 + headerValue.indexOf("opaque=\""), 8 + headerValue.indexOf("opaque=\"") + 32);
    String qop = "auth"; //strGetValue(headerValue,"qop=\"","\"");

    Serial.println("HttpSendDigest::AuthMethod: " + String(AuthMethod));
    Serial.println("HttpSendDigest::realm: " + String(realm));
    Serial.println("HttpSendDigest::nonce: " + String(nonce));
    Serial.println("HttpSendDigest::qop: " + String(qop));
    Serial.println("HttpSendDigest::opaque: " + String(opaque));
    Serial.print("HttpSendDigest::Calculate HA1...");

    String HA1 = md5(user + ":" + realm + ":" + password);
    Serial.println(HA1);

    Serial.print("HttpSendDigest::Calculate HA2...");
    String HA2 = md5("GET:" + path);
    Serial.println(HA2);

    String cnonce = String(random(8556824323));
    Serial.println("HttpSendDigest::cnonce: " + String(cnonce));
    Serial.println("HttpSendDigest::Calculate authResponse...");
    String authResponse = md5(HA1 + ":" + String(nonce) + ":00000001:" + String(cnonce) + ":" + String(qop) + ":" + String(HA2)); //MD5(HA1:nonce:nonceCount:cnonce:qop:HA2)
    Serial.println("HttpSendDigest::authResponse: " + String(authResponse));

    String authHeaderString = "Digest username=\"" + String(user) +
                              "\", realm=\"" + String(realm) +
                              "\", nonce=\"" + String(nonce) +
                              "\", uri=\"" + String(path) +
                              "\", response=\"" + String(authResponse) +
                              "\", qop=auth, nc=00000001, cnonce=\"" + String(cnonce) + "\", opaque=\"" + String(opaque) + "\"";

    Serial.println("HttpSendDigest::authHeaderString: " + String(authHeaderString));

    Serial.println("HttpSendDigest::---auth post---");
    http.begin(uri);
    http.collectHeaders(headerKeys, numberOfHeaders);
    //http.get(uri);
    http.addHeader("Authorization", authHeaderString);
    //  client.sendHeader(HTTP_HEADER_CONTENT_TYPE, "application/x-www-form-urlencoded");
    //   client.sendHeader(HTTP_HEADER_CONTENT_LENGTH, data.length());
    //    client.beginBody();
    //    client.print(data);
    statusCode = http.GET();
    //Serial.println("HttpSendDigest::---authresponse---");
    header = "";
    // read the status code and body of the response
    // statusCode = http.responseStatusCode();
    // if (http.headerAvailable()){
    //   while(!http.endOfHeadersReached()){
    //     header = header + char(http.readHeader());
    //   }
    // }

    //      headerName = http.headerName();
    //  headerValue = http.headerValue();
    response = http.getString();

    //   Serial.println("HttpSendDigest::Headername: "+ headerName);
    // Serial.println("HttpSendDigest::Headervalue: "+ headerValue);
    Serial.print("HttpSendDigest::statuscode: ");
    Serial.println(statusCode);
    Serial.println("HttpSendDigest::response: ");
    Serial.println(String(response));
  }
  if (statusCode == 200)
  {
    return true;
  }
  Serial.println("---Done---");
  return false;
  }

  String md5(String text) {
  MD5Builder builder;
  builder.begin();
  builder.addHexString(text);
  builder.calculate();
  return builder.toString();
  }
*/
