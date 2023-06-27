#include <WiFiClientSecure.h>
#include <ArduinoHA.h>
#include <DHT.h>

#include "credentials.h"

#define ARDUINOHA_DEBUG

#define EX_ARDUINOHA_BINARY_SENSOR
#define EX_ARDUINOHA_BUTTON
#define EX_ARDUINOHA_CAMERA
#define EX_ARDUINOHA_COVER
#define EX_ARDUINOHA_DEVICE_TRACKER
#define EX_ARDUINOHA_DEVICE_TRIGGER
#define EX_ARDUINOHA_FAN
#define EX_ARDUINOHA_HVAC
#define EX_ARDUINOHA_LIGHT
#define EX_ARDUINOHA_LOCK
#define EX_ARDUINOHA_SCENE
#define EX_ARDUINOHA_SELECT
#define EX_ARDUINOHA_SENSOR
#define EX_ARDUINOHA_SWITCH
#define EX_ARDUINOHA_TAG_SCANNER

#define WL_MAC_ADDR_LENGTH 6

#define DHTTYPE DHT22
#define DHTPIN 33

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);

HASensorNumber temperatureSensor("meulduino_temperature_sensor");
HASensorNumber humiditySensor("meulduino_humidity_sensor");
HASensorNumber heatIndexSensor("meulduino_heat_index_sensor");

DHT dht(DHTPIN, DHTTYPE);

const char* wifiSsid = WIFI_SSID;
const char* wifiPassword = WIFI_PASSWORD;

const char* mqttBrokerAddress = MQTT_BROKER_HOST;
const uint8_t mqttBrokerPortTls = 1883;
const char* mqttBrokerUserUsername = MQTT_BROKER_USERNAME;
const char* mqttBrokerUserPassword = MQTT_BROKER_PASSWORD;

const int sampleRateInMillis = 10000;

const byte mac[WL_MAC_ADDR_LENGTH] = { 0x24, 0x6F, 0x28, 0x7A, 0xCC, 0x00 };

unsigned long lastUpdateAt = 0;

void onWiFiConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("WiFi connected.");
}

void onWiFiIpAssigned(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("IP address assigned.");

  setupDevice();
  setupSensors();
  setupMqtt();
}

void onWiFiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("WiFi disconnected.");

  // Since we're disconnected, let's clean up and reboot.
  mqtt.disconnect();
  digitalWrite(LED_BUILTIN, LOW);
  ESP.restart();
}

void onMqttConnected()
{
  Serial.println("Connected to MQTT broker.");

  digitalWrite(LED_BUILTIN, HIGH);
}

void setupWiFi()
{
  Serial.println("Setting up WiFi connection.");

  // Reset old connection info.
  WiFi.disconnect(true);
  delay(1000);
  
  // Register callbacks
  WiFi.onEvent(onWiFiConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(onWiFiIpAssigned, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(onWiFiDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // Initiate connection
  WiFi.begin(wifiSsid, wifiPassword);
}

void setupDevice()
{
  Serial.println("Setting up device.");

  device.setUniqueId(mac, sizeof(mac));
  device.setName("Meulduino Test");
  device.setSoftwareVersion("1.0.0");
  device.setManufacturer("Meulduino b.v.");
  device.setModel("ESP32");
}

void setupSensors()
{
  setupTemperatureSensor();
  setupHumiditySensor();
  setupHeatIndexSensor();
}

void setupTemperatureSensor()
{
  Serial.println("Setting up Temperature Sensor.");

  temperatureSensor.setAvailability(true);
  temperatureSensor.setUnitOfMeasurement("°C");
  temperatureSensor.setName("Meulduino Temperature Sensor");
  temperatureSensor.setIcon("mdi:thermometer");
}

void setupHumiditySensor()
{
  Serial.println("Setting up Humidity Sensor.");

  humiditySensor.setAvailability(true);
  humiditySensor.setUnitOfMeasurement("%");
  humiditySensor.setName("Meulduino Humidity Sensor");
  humiditySensor.setIcon("mdi:water-percent");

}

void setupHeatIndexSensor()
{
  Serial.println("Setting up Heat Index Sensor.");

  heatIndexSensor.setAvailability(true);
  heatIndexSensor.setUnitOfMeasurement("°C");
  heatIndexSensor.setName("Meulduino Heat Index Sensor");
  heatIndexSensor.setIcon("mdi:heat-wave");
}

void setupMqtt()
{
  Serial.println("Setting up MQTT.");

  mqtt.onConnected(onMqttConnected);
  mqtt.begin(mqttBrokerAddress, mqttBrokerUserUsername, mqttBrokerUserPassword);

  setupDht22();
}

void setupDht22()
{
  dht.begin();
  // sleep(2000);
}

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  setupWiFi();
}

void loop()
{
  mqtt.loop();

  if ((millis() - lastUpdateAt) > sampleRateInMillis) {
    lastUpdateAt = millis();

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    float heatIndex = dht.computeHeatIndex(temperature, humidity, false);

    bool readError = (isnan(humidity) || isnan(temperature) || isnan(heatIndex));
    bool zeroValue = (humidity == 0 || temperature == 0 || heatIndex == 0);
    
    if (readError || zeroValue) {
      return; // We don't want 0 values or read errors reported.
    }

    temperatureSensor.setValue(temperature);
    humiditySensor.setValue(humidity);
    heatIndexSensor.setValue(heatIndex);
  }
}