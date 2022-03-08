const char* ssid = "Robert32CAM";

#include "esp_wifi.h"
#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFiManager.h>
#include "SSD1306Wire.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCameraServer();

const int MotPin0 = 12;
const int MotPin1 = 13;
const int MotPin2 = 14;
const int MotPin3 = 15;

void initMotors()
{
  ledcSetup(3, 2000, 8); // 2000 hz PWM, 8-bit resolution
  ledcSetup(4, 2000, 8); // 2000 hz PWM, 8-bit resolution
  ledcSetup(5, 2000, 8); // 2000 hz PWM, 8-bit resolution
  ledcSetup(6, 2000, 8); // 2000 hz PWM, 8-bit resolution
  ledcAttachPin(MotPin0, 3);
  ledcAttachPin(MotPin1, 4);
  ledcAttachPin(MotPin2, 5);
  ledcAttachPin(MotPin3, 6);
}

const int ServoPin = 2;
void initServo(){
  ledcSetup(8, 50, 16); // 50 hz PWM, 16-bit resolution, range from 3250 to 6500.
  ledcAttachPin(ServoPin, 8);
}

SSD1306Wire display(0x3c, 1, 3, GEOMETRY_128_32);

#define pulsador 16

//temperatura interna
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

//mini menu pulsador
bool Pmenu = true;

//modo inicio
bool wifi=true;

void setup(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  pinMode(pulsador, INPUT_PULLUP);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  // Remote Control Car
  initMotors();
  initServo();

  ledcSetup(7, 5000, 8);
  ledcAttachPin(4, 7);  //pin4 is LED

  //iniciacion de Oled
  delay(1000);
  display.init();

  display.flipScreenVertically();
  //display.setFont(ArialMT_Plain_10);
  display.clear();
  
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, ssid);
  display.drawString(0, 15, "IP:192.168.4.1");
  display.display();

  //creacion de red privada o Local
  if(digitalRead(pulsador) == HIGH){    // se conecta por Wifi Manager
    
    WiFi.mode(WIFI_STA);
    WiFiManager wm;
    //wm.resetSettings();
  
    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect(ssid); // password protected ap
  
    if(!res) {
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 10, "Fallo en Conexion");
        display.display();
        Serial.println("Failed to connect");
        delay(2000);
        ESP.restart();
    } 
    else {
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(46, 0, "WiFi");
        display.drawString(24, 15, "Conectado");
        display.display();
        Serial.println("conectado! :D");
        delay(2000);
      }
    drawIP(WiFi.localIP().toString()); // imprime el IP de la red Conectada
    
    for (int i = 0; i < 5; i++) {
      ledcWrite(7, 10); // flash led
      delay(50);
      ledcWrite(7, 0);
      delay(50);
    }
  }
  
  if(digitalRead(pulsador) == LOW){   //AccesPoint Crea una red wifi llamada RobertCam32
    WiFi.softAP(ssid);
    IPAddress miIP = WiFi.softAPIP();
    wifi = false;
    delay(3000);
    for (int i = 0; i < 3; i++) {
      ledcWrite(7, 10); // flash led
      delay(100);
      ledcWrite(7, 0);
      delay(100);
    }
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  startCameraServer();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(digitalRead(pulsador) == LOW){
    Pmenu = !Pmenu;
    delay(500);
  }

  //Pmenu
  if(Pmenu == true){
    if(wifi==true){
      drawIP(WiFi.localIP().toString());
    }
    else{
      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, ssid);
      display.drawString(0, 15, "IP:192.168.4.1");
      display.display();
    }
  }
  if(Pmenu == false){
    int tempInt = (temprature_sens_read() - 32) / 1.8;
    String myStr = String(tempInt);
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, myStr + "C°");
    display.display();
    delay(50);
  }
  ArduinoOTA.handle();
  Serial.printf("RSSi: %ld dBm\n", WiFi.RSSI());
}


void drawIP(String MyIP) {
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Conect IP:");
  display.drawString(0, 15, String(MyIP));
  display.display();
}
