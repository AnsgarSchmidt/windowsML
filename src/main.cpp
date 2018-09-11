#include <Wire.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <Adafruit_PWMServoDriver.h>

#define UPDATE_HOST   "mqtt.23-5.eu"
#define UPDATE_PORT   2342
#define UPDATE_PATH   ""
#define BUILD_VERSION "windowsMLRight-00"

#define WIFI_SSID     ""
#define WIFI_PASSWD   ""

#define MQTT_HOST     "172.22.1.2"
#define MQTT_PORT     1883

#define MOTION        10

#define WINDOW_LL_R   0
#define WINDOW_LL_G   1
#define WINDOW_LL_B   2
#define WINDOW_LC_R   3
#define WINDOW_LC_G   4
#define WINDOW_LC_B   5
#define WINDOW_CC_R   6
#define WINDOW_CC_G   7
#define WINDOW_CC_B   8
#define WINDOW_CR_R   9
#define WINDOW_CR_G  10
#define WINDOW_CR_B  11
#define WINDOW_RR_R  12
#define WINDOW_RR_G  13
#define WINDOW_RR_B  14
#define STRIP        15

WiFiClient              espClient;
PubSubClient            client(espClient);
Adafruit_PWMServoDriver pwm                = Adafruit_PWMServoDriver();

//uint32_t lastStatusMessage  = 0;
//uint32_t lastFirmwareCheck  = 0;
uint32_t lastMotion         = 0;

char  topic[100];
char  msg[100];

void checkForNewFirmware(void){

    t_httpUpdate_return ret = ESPhttpUpdate.update(UPDATE_HOST, UPDATE_PORT, UPDATE_PATH, BUILD_VERSION);

    switch(ret) {
        case HTTP_UPDATE_FAILED:
            Serial.println("[update] Update failed.");
            break;
        case HTTP_UPDATE_NO_UPDATES:
            Serial.println("[update] No Update.");
            break;
        case HTTP_UPDATE_OK:
            Serial.println("[update] Update ok."); // may not called we reboot the ESP
            break;
    }

}

void connectToWifi(void){
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = BUILD_VERSION;
    clientId += String(random(0xffff), HEX);
    Serial.println(clientId.c_str());
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      snprintf (topic, sizeof(topic), "seminarroom/windowsmlright/startup");
      client.publish(topic, BUILD_VERSION);
      snprintf (topic, sizeof(topic), "seminarroom/windowsmlright/#");      
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {

  if(String("seminarroom/windowsmlright/ll") == String(topic) && length > 2 ){
    uint16_t v = 0;
    v = 16 * payload[0];
    pwm.setPWM(WINDOW_LL_R, 0, v);
    v = 16 * payload[1];
    pwm.setPWM(WINDOW_LL_G, 0, v);
    v = 16 * payload[2];
    pwm.setPWM(WINDOW_LL_B, 0, v);
  }

  if(String("seminarroom/windowsmlright/lc") == String(topic) && length > 2 ){
    uint16_t v = 0;
    v = 16 * payload[0];
    pwm.setPWM(WINDOW_LC_R, 0, v);
    v = 16 * payload[1];
    pwm.setPWM(WINDOW_LC_G, 0, v);
    v = 16 * payload[2];
    pwm.setPWM(WINDOW_LC_B, 0, v);
  }

  if(String("seminarroom/windowsmlright/cc") == String(topic) && length > 2 ){
    uint16_t v = 0;
    v = 16 * payload[0];
    pwm.setPWM(WINDOW_CC_R, 0, v);
    v = 16 * payload[1];
    pwm.setPWM(WINDOW_CC_G, 0, v);
    v = 16 * payload[2];
    pwm.setPWM(WINDOW_CC_B, 0, v);
  }

  if(String("seminarroom/windowsmlright/cr") == String(topic) && length > 2 ){
    uint16_t v = 0;
    v = 16 * payload[0];
    pwm.setPWM(WINDOW_CR_R, 0, v);
    v = 16 * payload[1];
    pwm.setPWM(WINDOW_CR_G, 0, v);
    v = 16 * payload[2];
    pwm.setPWM(WINDOW_CR_B, 0, v);
  }

  if(String("seminarroom/windowsmlright/rr") == String(topic) && length > 2 ){
    uint16_t v = 0;
    v = 16 * payload[0];
    pwm.setPWM(WINDOW_RR_R, 0, v);
    v = 16 * payload[1];
    pwm.setPWM(WINDOW_RR_G, 0, v);
    v = 16 * payload[2];
    pwm.setPWM(WINDOW_RR_B, 0, v);
  }

  if(String("seminarroom/windowsmlright/stripe") == String(topic) && length > 0 ){
    uint16_t v = 16 * payload[0];
    pwm.setPWM(STRIP, 0, v);
  }

}

void setup(){

    pwm.begin();
    pwm.setPWMFreq(60);
    for (uint8_t i = 0; i < 16; i++ ){
      pwm.setPWM(i, 0, 0);
    }  

    pinMode(MOTION, INPUT);

    Serial.begin(9600);

    for(uint8_t t = 5; t > 0; t--) {
        Serial.printf("[SETUP] WAIT %d...\n", t);
        Serial.flush();
        delay(500);
    }

    Serial.print("My version:");
    Serial.println(BUILD_VERSION);

    connectToWifi();

    client.setServer(MQTT_HOST, MQTT_PORT);
    client.setCallback(callback);

}

void loop(){

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  //if (millis() < lastStatusMessage) {
  //  lastStatusMessage = 0;
  //}

  //if (millis() - lastStatusMessage > (23 * 1000)) {
  //  lastStatusMessage = millis();
  //  snprintf (topic, sizeof(topic), "seminarroom/windowsmlleft/status");
  // snprintf (msg, sizeof(msg), "{\"uptime\":%lu, \"version\":\"%s\"}", millis(), BUILD_VERSION);
  //  client.publish(topic, msg);
  //}

  //if (millis() < lastFirmwareCheck) {
  //  lastFirmwareCheck = 0;
  //}

  //if (millis() - lastFirmwareCheck > (1 * 60 * 1000)) {
  //  lastFirmwareCheck = millis();
  //  checkForNewFirmware();
  //}

  if (millis() < lastMotion) {
    lastMotion = 0;
  }

  if (digitalRead(MOTION)) {
    if (millis() - lastMotion > 1000){
      lastMotion = millis();
      snprintf (topic, sizeof(topic), "seminarroom/windowsmlright/motion");
      snprintf (msg, sizeof(msg), "now");
      client.publish(topic, msg);
    }
  }

  delay(10);

}
