# 1 "/var/folders/q9/yq4yhzcx5jj1rh1dhhfztph80000gn/T/tmpw58eg1ob"
#include <Arduino.h>
# 1 "/Users/timolohmann/Documents/PlatformIO/Projects/Camper-tracker/src/camper-tracker.ino"

#include <Arduino.h>
#include <SoftWire.h>
#include "SSD1306.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define SEALEVELPRESSURE_HPA (1019)

Adafruit_BME280 bme;

#define PIN_SDA 4
#define PIN_SCL 15

SoftWire sw(PIN_SDA, PIN_SCL);
SSD1306 display(0x3C, PIN_SDA, PIN_SCL);

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

const char *ssid = "HATILAN";
const char *password = "1Qayxsw2";
const char *serverName = "http://rudi.timolohmann.de/logs";
unsigned long delayTime;
void setup();
void loop();
void printOnDisplay(String tmp, String hum, String lon, String lat, String alt, String speed);
void sendToServer(String tmp, String hum, String lon, String lat, String alt, String speed, String sat);
void printInSerial(String temperature, String humidity, String longitude, String latitude, String altitude, String speed);
#line 30 "/Users/timolohmann/Documents/PlatformIO/Projects/Camper-tracker/src/camper-tracker.ino"
void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  delay(50);
  digitalWrite(16, HIGH);
  delay(50);
  display.init();
  display.setFont(ArialMT_Plain_10);
  delay(50);
  display.drawString(0, 0, "Starting up ...");
  display.drawString(0, 12, "- and initializing.");
  display.display();
  delay(500);

  bool bme_status;
  bme_status = bme.begin(0x76);
  if (!bme_status)
  {
    display.clear();
    display.drawString(0, 0, "No valid BME280 found");
    display.drawString(0, 12, "Please check wiring!");
    display.display();
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  SerialGPS.begin(9600, SERIAL_8N1, 22, 23);
  delay(500);

  delayTime = 30000;

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X16,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5);
}

void loop()
{
  static int p0 = 0;

  while (SerialGPS.available() > 0)
  {
    gps.encode(SerialGPS.read());
  }

  bme.takeForcedMeasurement();
  String humidity = String(bme.readHumidity());
  String temperature = String(bme.readTemperature());
  String longitude = String(gps.location.lng(), 6);
  String latitude = String(gps.location.lat(), 6);
  String altitude = String(gps.altitude.meters());
  String speed = String(gps.speed.kmph());
  String satellites = String(gps.satellites.value());

  display.clear();
  printOnDisplay(temperature, humidity, longitude, latitude, altitude, speed);
  printInSerial(temperature, humidity, longitude, latitude, altitude, speed);
  sendToServer(temperature, humidity, longitude, latitude, altitude, speed, satellites);
  display.display();

  delay(delayTime);
}

void printOnDisplay(String tmp, String hum, String lon, String lat, String alt, String speed)
{
  String tempvar = String("Temp: ") + (tmp) + (" °C");
  display.drawString(0, 0, tempvar);

  String humvar = String("Humidity: ") + (hum) + (" %");
  display.drawString(0, 10, humvar);

  String longvar = String("Longitude: " + (lon));
  display.drawString(0, 20, longvar);

  String latvar = String("Latitude: " + (lat));
  display.drawString(0, 30, latvar);

  String speedvar = String("Speed: " + (speed) + " km/h");
  display.drawString(0, 40, speedvar);
}

void sendToServer(String tmp, String hum, String lon, String lat, String alt, String speed, String sat)
{
  HTTPClient http;
  http.begin(serverName);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String httpRequestData = "client=rudi&temperature=" + String(tmp) +
                           "&humidity= " + String(hum) +
                           "&longitude=" + String(lon) +
                           "&latitude=" + String(lat) +
                           "&speed=" + String(speed) +
                           "&altitude=" + String(alt) +
                           "&satellites=" + String(sat) +
                           "&key=123";

  int httpResponseCode = http.POST(httpRequestData);

  if (httpResponseCode > 0)
  {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
    display.drawString(0, 50, "*** Data sent ***");
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
    display.drawString(0, 50, "*** Error ***");
  }
  http.end();
}

void printInSerial(String temperature, String humidity, String longitude, String latitude, String altitude, String speed)
{
  Serial.print("LAT=");
  Serial.println(latitude);
  Serial.print("LONG=");
  Serial.println(longitude);
  Serial.print("ALT=");
  Serial.println(altitude);
  Serial.print("Sats=");
  Serial.println(gps.satellites.value());
  Serial.print("Speed: ");
  Serial.println(speed);
  Serial.print("Temp: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
}