#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <ArduinoOTA.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <IotWebConfTParameter.h>
#include <IotWebConfESP32HTTPUpdateServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <uptime.h>
#include <algorithm>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <GRGB.h>
#include "soc/rtc_wdt.h"

#define STRING_LEN 128
#define nils_length(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))
// #define nils_length( x ) ( sizeof(x) )

// ports
const int DHT11PIN = 11;
const int FANPIN = 1;
const int FRIDGEPIN = 37;
const int BUZZERPIN = 18;
const int TRAYPIN = 35;
const int LEDRED = 33;
const int LEDGREEN = 3;
const int LEDBLUE = 39;
const int LEDPIN = 9;
const int LEDPPIN = 2;
const int TOUCHLEDPIN = T5;
const int TOUCHONOFFPIN = T4;

bool running = false;
unsigned long sleepTime = 0;
int thresholdOnOff = 27600;
int thresholdLed = 31400;
bool touchOnOff = false;
bool touchLed = false;
bool touchOnOffTrigger = false;
bool touchLedTrigger = false;
unsigned long touchMillis = 0;


//DHT11 Sensor
#define DHTTYPE    DHT11 
DHT_Unified dht(DHT11PIN, DHTTYPE);
float temp = 0.0;
float humidity = 0.0;
float humidity_max = 75.0;
unsigned long timer10sMillis = 0;
unsigned long timer1sMillis = 0;

//RGB LED
GRGB led(COMMON_CATHODE, LEDRED, LEDGREEN, LEDBLUE);
bool ledOnOff = true;

// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883
#define MQTT_PUB_TEMP "air/dehum/temp"
#define MQTT_PUB_HUMIDITY "air/dehum/humidity"
#define MQTT_PUB_RUNNING "air/dehum/running"
#define MQTT_PUB_TRAY "air/dehum/tray"
#define MQTT_PUB_INFO "air/dehum/info"
#define MQTT_SUB_LED "air/dehum/led"
#define MQTT_SUB_SLEEP "air/dehum/sleep"

AsyncMqttClient mqttClient;
char mqttServer[STRING_LEN];
char mqttUser[STRING_LEN];
char mqttPassword[STRING_LEN];
// char mqttPumpTopic[STRING_LEN];
// char mqttPumpValue[STRING_LEN];

Ticker mqttReconnectTimer;
// Ticker sec10Timer;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
char ntpServer[STRING_LEN];
char ntpTimezone[STRING_LEN];
char hostname[STRING_LEN];
time_t now;
struct tm localTime;

#define CONFIG_VERSION "1"
bool needReset = false;
Preferences preferences;
// int iotWebConfPinState = HIGH;
// unsigned long iotWebConfPinChanged = 0;
DNSServer dnsServer;
WebServer server(80);
HTTPUpdateServer httpUpdater;
IotWebConf iotWebConf("Luftentfeuchter", &dnsServer, &server, "", CONFIG_VERSION);
IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("server", "mqttServer", mqttServer, STRING_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("user", "mqttUser", mqttUser, STRING_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("password", "mqttPassword", mqttPassword, STRING_LEN);
// IotWebConfTextParameter mqttPumpTopicParam = IotWebConfTextParameter("external pump start topic", "mqttPumpTopic", mqttPumpTopic, STRING_LEN, "");
// IotWebConfTextParameter mqttPumpValueParam = IotWebConfTextParameter("external pump start Value", "mqttPumpValue", mqttPumpValue, STRING_LEN, "");
IotWebConfParameterGroup ntpGroup = IotWebConfParameterGroup("ntp", "NTP configuration");
IotWebConfTextParameter ntpServerParam = IotWebConfTextParameter("server", "ntpServer", ntpServer, STRING_LEN, "de.pool.ntp.org");
IotWebConfTextParameter ntpTimezoneParam = IotWebConfTextParameter("timezone", "ntpTimezone", ntpTimezone, STRING_LEN, "CET-1CEST,M3.5.0/02,M10.5.0/03");
IotWebConfParameterGroup dehumGroup = IotWebConfParameterGroup("dehum", "dehumidifier configuration");
iotwebconf::FloatTParameter dehumHumidityThresholdParam = iotwebconf::Builder<iotwebconf::FloatTParameter>("dehumHumidityThresholdParam").label("humidity thresholdOnOff").defaultValue(55.0).step(0.5).placeholder("e.g. 55.5").build();

int mod(int x, int y)
{
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

String verbose_print_reset_reason(esp_reset_reason_t reason)
{
  switch (reason)
  {
    case ESP_RST_UNKNOWN  : return(" Reset reason can not be determined");
    case ESP_RST_POWERON  : return("Reset due to power-on event");
    case ESP_RST_EXT  : return("Reset by external pin (not applicable for ESP32)");
    case ESP_RST_SW  : return("Software reset via esp_restart");
    case ESP_RST_PANIC  : return("Software reset due to exception/panic");
    case ESP_RST_INT_WDT  : return("Reset (software or hardware) due to interrupt watchdog");
    case ESP_RST_TASK_WDT  : return("Reset due to task watchdog");
    case ESP_RST_WDT  : return("Reset due to other watchdogs");
    case ESP_RST_DEEPSLEEP : return("Reset after exiting deep sleep mode");
    case ESP_RST_BROWNOUT : return("Brownout reset (software or hardware)");
    case ESP_RST_SDIO : return("Reset over SDIO");
    default : return("NO_MEAN");
  }
}

// -- SECTION: Wifi Manager
void handleRoot()
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  char tempStr[128];

  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";  
  s += iotWebConf.getHtmlFormatProvider()->getStyle();
  s += "<title>Dehumidifier</title>";
  s += iotWebConf.getHtmlFormatProvider()->getHeadEnd();
  s += "<fieldset id=" + String(mqttGroup.getId()) + ">";
  s += "<legend>" + String(mqttGroup.label) + "</legend>";
  s += "<table border = \"0\"><tr>";
  s += "<td>" + String(mqttServerParam.label) + ": </td>";
  s += "<td>" + String(mqttServer) + "</td>";
  s += "</tr></table></fieldset>";
 
  s += "<fieldset id=" + String(ntpGroup.getId()) + ">";
  s += "<legend>" + String(ntpGroup.label) + "</legend>";
  s += "<table border = \"0\"><tr>";
  s += "<td>" + String(ntpServerParam.label) + ": </td>";
  s += "<td>" + String(ntpServer) + "</td>";
  s += "</tr><tr>";
  s += "<td>" + String(ntpTimezoneParam.label) + ": </td>";
  s += "<td>" + String(ntpTimezone) + "</td>";
  s += "</tr><tr>";
  s += "<td>actual local time: </td>";
  strftime(tempStr, 40, "%d.%m.%Y %T", &localTime);
  s += "<td>" + String(tempStr) + "</td>";
  s += "</tr></table></fieldset>";

  s += "<fieldset id=Sensor>";
  s += "<legend>Sensor</legend>";
  s += "<table border = \"0\"><tr>";
  s += "<td>Temperatur: </td>";
  s += "<td>" + String(temp) + "&#8451;</td>";
  s += "</tr><tr>";
  s += "<td>Humidity: </td>";
  s += "<td>" + String(humidity) + "% / " + dehumHumidityThresholdParam.value() + "%</td>";
  s += "</tr><tr>";
  s += "<td>Status: </td>";
  s += "<td>";
  if (running)
    s += "running";
  else 
    s += "stopped";
  s += "</td>";
  s += "</tr><tr>";
  s += "<td>Tray:</td><td>";
  if (digitalRead(TRAYPIN))
    s += "empty";
  else 
    s += "full";  
  s += "</td>";
  s += "</tr></table></fieldset>";
  uptime::calculateUptime();
  sprintf(tempStr, "%04u Tage %02u:%02u:%02u", uptime::getDays(), uptime::getHours(), uptime::getMinutes(), uptime::getSeconds());
  s += "<p>Uptime: " + String(tempStr);
  s += "<p>Last reset: " + verbose_print_reset_reason(esp_reset_reason());
  s += "</fieldset>";
  s += "<p>Go to <a href='config'>Configuration</a>";
  s += "<br>";
  s += touchRead(TOUCHONOFFPIN);
  s += "<br>";
  s += touchRead(TOUCHLEDPIN);
  s += iotWebConf.getHtmlFormatProvider()->getEnd();

  server.send(200, "text/html", s);
}

void configSaved()
{
  preferences.putString("apPassword", String(iotWebConf.getApPasswordParameter()->valueBuffer));
  preferences.putString("wifiSsid", String(iotWebConf.getWifiAuthInfo().ssid));
  preferences.putString("wifiPassword", String(iotWebConf.getWifiAuthInfo().password));

  Serial.println("Configuration saved.");
  // TODO: Neustart bei normalen Parametern vermeiden  
  needReset = true;
}

bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper)
{
  Serial.println("Validating form.");
  bool valid = true;

  // int l = webRequestWrapper->arg(mqttServerParam.getId()).length();
  // if (l < 3)
  // {
  //   mqttServerParam.errorMessage = "Please enter at least 3 chars!";
  //   valid = false;
  // }

  return valid;
}

//-- SECTION: connection handling
void setTimezone(String timezone)
{
  Serial.printf("  Setting Timezone to %s\n", ntpTimezone);
  setenv("TZ", ntpTimezone, 1); //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnected()
{
  Serial.println("Connected to Wi-Fi.");
  Serial.println(WiFi.localIP());
  connectToMqtt();
  timeClient.begin();
}

void onWifiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  timeClient.end();
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub;
  
  packetIdSub = mqttClient.subscribe(MQTT_SUB_LED, 2);
  Serial.print("Subscribed to topic: ");
  Serial.println(String(MQTT_SUB_LED) + " - " + String(packetIdSub));
  digitalWrite(LED_BUILTIN, HIGH);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  String text;
  switch (reason)
  {
  case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
    text = "TCP_DISCONNECTED";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
    text = "MQTT_UNACCEPTABLE_PROTOCOL_VERSION";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
    text = "MQTT_IDENTIFIER_REJECTED";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
    text = "MQTT_SERVER_UNAVAILABLE";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
    text = "MQTT_MALFORMED_CREDENTIALS";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
    text = "MQTT_NOT_AUTHORIZED";
    break;
  }
  Serial.printf(" [%8u] Disconnected from the broker reason = %s\n", millis(), text.c_str());
  Serial.printf(" [%8u] Reconnecting to MQTT..\n", millis());
  digitalWrite(LED_BUILTIN, LOW);

  if (WiFi.isConnected())
  {
    mqttReconnectTimer.once(5, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.printf(" [%8u] Subscribe acknowledged id: %u, qos: %u\n", millis(), packetId, qos);
}

void onMqttPublish(uint16_t packetId)
{
  // Serial.print("Publish acknowledged.");
  // Serial.print("  packetId: ");
  // Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  // Serial.println("Publish received.");
  // Serial.print("  topic: ");
  // Serial.println(topic);
  // Serial.print("  qos: ");
  // Serial.println(properties.qos);
  // Serial.print("  payload: ");
  // Serial.println(payload);
  // Serial.print("  dup: ");
  // Serial.println(properties.dup);
  // Serial.print("  retain: ");
  // Serial.println(properties.retain);
  // Serial.print("  len: ");
  // Serial.println(len);
  // Serial.print("  index: ");
  // Serial.println(index);
  // Serial.print("  total: ");
  // Serial.println(total);
  char new_payload[len + 1];
  strncpy(new_payload, payload, len);
  new_payload[len] = '\0';
  if (strcmp(topic, MQTT_SUB_LED) == 0)
  {
    Serial.print("mqtt LED: ");
    Serial.println(new_payload);
    ledOnOff = (strcmp(MQTT_SUB_LED, new_payload) == 0);
  }
  if (strcmp(topic, MQTT_SUB_SLEEP) == 0)
  {
    Serial.print("mqtt SLEEP: ");
    Serial.println(new_payload);
    sleepTime = millis() + (atoi(new_payload) * 1000);
  }
}
//-- END SECTION: connection handling


void mqttSendtemp()
{
  char msg_out[20];
  dtostrf(humidity, 2, 2, msg_out);
  mqttClient.publish(MQTT_PUB_HUMIDITY, 0, true, msg_out);
  dtostrf(temp, 2, 2, msg_out);
  mqttClient.publish(MQTT_PUB_TEMP, 0, true, msg_out);
  dtostrf(digitalRead(TRAYPIN), 2, 2, msg_out);
  mqttClient.publish(MQTT_PUB_TRAY, 0, true, msg_out);
}

void getLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time 1");
    return;
  }
  localTime = timeinfo;
}

void updateTime()
{
  if (iotWebConf.getState() == 4)
  {
    timeClient.update();
    getLocalTime();
  }
}

void publishUptime()
{
  char msg_out[20];
  uptime::calculateUptime();
  sprintf(msg_out, "%04u Tage %02u:%02u:%02u", uptime::getDays(), uptime::getHours(), uptime::getMinutes(), uptime::getSeconds());
  // Serial.println(msg_out);
  mqttClient.publish(MQTT_PUB_INFO, 0, true, msg_out);
}

void updateLed()
{
  // 360 auf 255 Schritte runterrechnen
  uint8_t hsv_min = 0;
  uint8_t hsv_max = 240;
  uint8_t hsvCurrent;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  float humidity_thresholdOnOff = dehumHumidityThresholdParam.value();
  float percent = (humidity - humidity_thresholdOnOff) / (humidity_max - humidity_thresholdOnOff);

  if (ledOnOff)
  {    
    if (percent > 1)
    {
      led.enable();
      hsvCurrent = hsv_min;
      uint8_t hue = (hsvCurrent / 360.0 * 255.0);      
      led.setHSV(hue, 255, 255);    
    }
    else if (percent < 0)
      led.disable();
    else
    {
      led.enable();      
      hsvCurrent = hsv_max - (hsv_max * percent);
      uint8_t hue = (hsvCurrent / 360.0 * 255.0);      
      led.setHSV(hue, 255, 255);    
    }
  } else {    
    led.disable();
  }
}

void run(bool onoff) {
  if (onoff != running)
  {
    running = onoff;
    if (onoff) {
      digitalWrite(FRIDGEPIN, LOW);
      digitalWrite(FANPIN, HIGH);
    } else {
      digitalWrite(FRIDGEPIN, HIGH);
      digitalWrite(FANPIN, LOW);
    }
    if (running)
      mqttClient.publish(MQTT_PUB_RUNNING, 0, true, "1");
    else
      mqttClient.publish(MQTT_PUB_RUNNING, 0, true, "0");
  }
}

void onTouchOnOff()
{
  touchOnOff = true;
}

void onTouchLed()
{
  touchLed = true;
}

void onSec1Timer()
{
  if (sleepTime >= millis())
    digitalWrite(LEDPPIN, !digitalRead(LEDPPIN));
  else
    digitalWrite(LEDPPIN, HIGH);
  updateTime();
}

void onSec10Timer()
{
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
    temp = 0;
  }
  else {
    temp = event.temperature;
  }
 // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
    humidity = 0;
  }
  else {
    humidity = event.relative_humidity;
  }

  if (digitalRead(TRAYPIN) && sleepTime < millis()) 
  {
    if (!running && humidity > dehumHumidityThresholdParam.value()) {
      run(true);
    } else if (running && humidity < 50)
    {
      run(false);
    }
  } else {
    run(false);
  }
  updateLed();
  publishUptime();
  mqttSendtemp();
}

void setup()
{
  rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);  
  rtc_wdt_set_time(RTC_WDT_STAGE0, 4000);
  // basic setup
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FANPIN, OUTPUT);
  pinMode(FRIDGEPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(LEDPPIN, OUTPUT);
  digitalWrite(FANPIN, LOW);
  digitalWrite(FRIDGEPIN, HIGH);
  digitalWrite(LEDPIN, HIGH);
  digitalWrite(LEDPPIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  

  pinMode(TRAYPIN, INPUT_PULLUP);

  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));


  // WiFi.onEvent(onWifiConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(onWifiDisconnect, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  // WiFi.setTxPower(WIFI_POWER_19_5dBm);

  if (!preferences.begin("wifi"))
  {
    Serial.println("Error opening NVS-Namespace");
    for (;;);  // leere Dauerschleife -> Ende
  }    
  iotWebConf.setupUpdateServer(
      [](const char *updatePath)
      { httpUpdater.setup(&server, updatePath); },
      [](const char *userName, char *password)
      { httpUpdater.updateCredentials(userName, password); });

  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttUserNameParam);
  mqttGroup.addItem(&mqttUserPasswordParam);
  iotWebConf.addParameterGroup(&mqttGroup);
  ntpGroup.addItem(&ntpServerParam);
  ntpGroup.addItem(&ntpTimezoneParam);
  iotWebConf.addParameterGroup(&ntpGroup);

  dehumGroup.addItem(&dehumHumidityThresholdParam);
  iotWebConf.addParameterGroup(&dehumGroup);

  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.setWifiConnectionCallback(&onWifiConnected);

  bool validConfig = iotWebConf.init();
  if (!validConfig)
  {
    Serial.println("Invalid config detected - restoring WiFi settings...");
    // much better handling than iotWebConf library to avoid lost wifi on configuration change    
    if (preferences.isKey("apPassword"))
      strncpy(iotWebConf.getApPasswordParameter()->valueBuffer, preferences.getString("apPassword").c_str(), iotWebConf.getApPasswordParameter()->getLength());
    else
      String("AP Password not found for restauration.");
    if (preferences.isKey("wifiSsid")) 
      strncpy(iotWebConf.getWifiSsidParameter()->valueBuffer, preferences.getString("wifiSsid").c_str(), iotWebConf.getWifiSsidParameter()->getLength());
    else
      String("WiFi SSID not found for restauration.");
    if (preferences.isKey("wifiPassword")) 
      strncpy(iotWebConf.getWifiPasswordParameter()->valueBuffer, preferences.getString("wifiPassword").c_str(), iotWebConf.getWifiPasswordParameter()->getLength());
    else
      String("WiFi Password not found for restauration.");
    iotWebConf.saveConfig();
    iotWebConf.resetWifiAuthInfo();
  }

  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/config", []
            { iotWebConf.handleConfig(); });
  server.onNotFound([]()
                    { iotWebConf.handleNotFound(); });
  Serial.println("Wifi manager ready.");

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onSubscribe(onMqttSubscribe);

  if (mqttUser != "")
    mqttClient.setCredentials(mqttUser, mqttPassword);
  mqttClient.setServer(mqttServer, MQTT_PORT);

  // configure the timezone
  configTime(0, 0, ntpServer);
  setTimezone(ntpTimezone);
  
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.println(WiFi.localIP());

  // sec10Timer.attach(10, onSec10Timer);
}

void loop()
{
  iotWebConf.doLoop();
  ArduinoOTA.handle();
  rtc_wdt_feed();

  if (timer10sMillis < millis())
  {
    timer10sMillis = millis() + 10000;
    onSec10Timer();
  }

  if (timer1sMillis < millis())
  {
    timer1sMillis = millis() + 1000;
    onSec1Timer();
  }

  if (needReset)
  {
    Serial.println("Rebooting in 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }

  if (touchMillis < millis())
  {
    if (touchRead(TOUCHONOFFPIN) > thresholdOnOff)
    {
      touchOnOff = true;
      touchMillis = millis() + 500;
    }
    else
    {
      touchOnOff = false;
      touchOnOffTrigger = true;
    }
  }
  if (touchMillis < millis())
  {  
    if (touchRead(TOUCHLEDPIN) > thresholdLed)
    {
      touchLed = true;
      touchMillis = millis() + 500;
    }
    else
    {
      touchLed = false;
      touchLedTrigger = true;
    }
  }

  if (touchOnOff && touchOnOffTrigger)
  {
    if (sleepTime > millis())
      sleepTime = 0;
    else
      sleepTime = millis() + 3600000;   // sleep 1 hour
    tone(BUZZERPIN, 1000, 500);
    touchOnOffTrigger = false;
  }
  if (touchLed && touchLedTrigger)
  {
    if (ledOnOff)
    {
      ledOnOff = false;
      digitalWrite(LEDPIN, LOW);
      tone(BUZZERPIN, 300, 500);
    }
    else
    {
      ledOnOff = true;
      digitalWrite(LEDPIN, HIGH);
      tone(BUZZERPIN, 500, 500);
    }        
    updateLed();    
    touchLedTrigger = false;
  }
}