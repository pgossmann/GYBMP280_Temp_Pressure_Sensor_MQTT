// important parameters to change
<<<<<<< HEAD:GY21_Temp_Humidity_Sensor_MQTT.ino
String baseTopic = "/sensor/gy21/";
=======
String baseTopic = "/sensor/bmp280/";
>>>>>>> 0b2feabc85099e8e6142a5e6b6e58c715b413549:GYBMP280_Temp_Pressure_Sensor_MQTT.ino
// sleep interval 15 minutes = 900 seconds, 5 minutes = 300, 20 seconds for testing
unsigned long sleepInterval = 900e6;

// webserver includes
#include <WiFiClient.h>
#include <PubSubClient.h>

#include "Adafruit_BMP280.h"
// Load Wi-Fi library
#include <ESP8266WiFi.h>

// network credentials
#include "secrets.h"
// const char* mqtthost = "IPAddressOFRaspi";
// const char* ssid = "yourWifiSSID";
// const char* password = "password";

// variables for the sensor reading
float pressure;
float temp;
float volt;

#define SCL 5  // D1 ON NODEMCU
#define SDA 4  // D2 ON NODEMCU
#define BLUE_LED_PIN 2

// Connect VIN to 3-5VDC
// Connect GND to ground
// Connect SCL to I2C clock pin
// Connect SDA to I2C data pin
Adafruit_BMP280 sensor;
#define BMP280_I2C_ADDRESS 0x76

// MQTT
WiFiClient wifi;
PubSubClient mqttClient(wifi);
<<<<<<< HEAD:GY21_Temp_Humidity_Sensor_MQTT.ino
String mqttclientId = "sensor2";

=======
String mqttclientId = "sensorWemos";
>>>>>>> 0b2feabc85099e8e6142a5e6b6e58c715b413549:GYBMP280_Temp_Pressure_Sensor_MQTT.ino

// fast wifi
#include "wifi_helper.h"
WifiHelper wh;  // use wh(123) to set EEPROM offset

void setup() {
  Serial.begin(9600);
  Serial.println("GY-BMP280");
  // setup pin D1 (GPIO 5) as SCL (clock) and pin D2 (GPIO 4) as SDA (data)
  // Wire.begin(SDA, SCL);

  sensor.begin(BMP280_I2C_ADDRESS);
  sensor.wakeup();

  // turn on the light
  pinMode(BLUE_LED_PIN, OUTPUT);
  toggleLed(true);

  // connect wifi with fast mode = true
  bool isConnected = connectToWifi(true);
  if (!isConnected) {
    // let's try again in a few minutes
    goToSleep();
    return;
  }
  // connect to mqtt server (raspi pi running on mqtthost)
  mqttClient.setServer(mqtthost, 1883);

  // voltage
  pinMode(A0, INPUT);
}

void loop() {
  readSensor();
  readVoltage();
  writeSensorToSerial();
  publishSensorToMqtt();
  goToSleep();
}

void goToSleep() {
  // put sensor to sleep?
  sensor.sleep();
  toggleLed(false);
  delay(100);
  Serial.println("Going to sleep");
  ESP.deepSleep(sleepInterval);
  delay(100);
}

void readSensor() {
  temp = sensor.readTemperature();
  pressure = sensor.readPressure();
}

void readVoltage() {
  int nVoltageRaw = analogRead(A0);
  volt = (float)nVoltageRaw;
}

void writeSensorToSerial() {
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print("°C    ");
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");
  Serial.print("Voltage: ");
  Serial.println(volt);
}

void toggleLed(bool onOff) {
  if (onOff) {
    // turn led on
    digitalWrite(BLUE_LED_PIN, LOW);
  } else {
    // turn led off
    digitalWrite(BLUE_LED_PIN, HIGH);
  }
}

void publishSensorToMqtt() {
  bool connected = mqttClient.connected();
  if (!connected) {
    connected = connectToMqTT();
  }
  // can't connect? nothing to publish
  if (!connected) { return; }
  if (!isnan(temp)) {
    publish(baseTopic + "temp", String(temp));
  }
  if (!isnan(pressure)) {
    publish(baseTopic + "pressure", String(pressure));
  }
  if (!isnan(volt)) {
    publish(baseTopic + "voltage", String(volt));
  }
}
void publish(String topic, String payload) {
  Serial.println("published " + topic + ": " + payload);
  mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

bool connectToWifi(bool useFastMode) {
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  bool isConnected = false;
  if (useFastMode) {
    // fast connection mode, cached details
    isConnected = wh.connect(ssid, password);
    if (!isConnected) {
      Serial.println("Wifi connection failed");
    }
  } else {
    // classic connection
    bool timeout = false;
    int trys = 0;
    WiFi.begin(ssid, password);
    while (!timeout && WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      trys++;
      timeout = trys > 8;
    }
    isConnected = WiFi.status() == WL_CONNECTED;
  }
  if (isConnected) {
    // Print local IP address and start web server
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  return isConnected;
}

bool connectToMqTT() {
  // Loop until we're reconnected
  bool timeout = false;
  int trys = 0;
  while (!timeout && !mqttClient.connected()) {
    Serial.println("Attempting MQTT connection... #"+String(trys+1));
    // Create a random client ID
    mqttclientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(mqttclientId.c_str())) {
      Serial.println("connected with client id " + mqttclientId);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
      trys++;
      timeout = trys > 1;
    }
  }
  return mqttClient.connected();
}
