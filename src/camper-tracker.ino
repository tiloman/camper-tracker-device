
#include <Arduino.h>
#include <SoftWire.h>
#include "SSD1306.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define PIN_SDA 4
#define PIN_SCL 15
SSD1306 display(0x3C, PIN_SDA, PIN_SCL);

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

struct GpsDataState_t
{
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double distMax = 0;
  double dist = 0;
  double altMax = -999999;
  double altMin = 999999;
  double spdMax = 0;
  double prevDist = 0;
};
GpsDataState_t gpsState = {};

// BME280
#define SEALEVELPRESSURE_HPA (1019) // Check Internet for MSL pressure at our place to calc your elevation mASL
#define TEMP_CORR (0)              // Manual correction of temp sensor (mine reads 2 degrees C too high)
#define ELEVATION (505)             // Enter your elevation mASL to calculate MSL pressure (QNH) at your place

Adafruit_BME280 bme; // I2C



SoftWire sw(PIN_SDA, PIN_SCL);

float SLpressure_hPa;

const char *ssid = "HATILAN";
const char *password = "1Qayxsw2";
const char *serverName = "http://rudi.timolohmann.de/logs";
unsigned long delayTime;

#define TASK_SERIAL_RATE 6000 // ms
uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
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
  bme_status = bme.begin(0x76); // address either 0x76 or 0x77
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

  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  delayTime = 5000;

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X16, // temperature
                  Adafruit_BME280::SAMPLING_X1,  // pressure
                  Adafruit_BME280::SAMPLING_X1,  // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5);
}

void loop()
{
  static int p0 = 0;

  gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();

  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;

  while (SerialGPS.available() > 0)
  {
    gps.encode(SerialGPS.read());
  }

  if (nextSerialTaskTs < millis())
  {
    printValues();
    printGPSSerial();
    nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
  }
}

void printValues()
{
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();

  display.clear(); // clear display buffer
  String tempvar = String("Temp: ") + (temperature) + (" °C");
  display.drawString(0, 0, tempvar);

  String humvar = String("Temp: ") + (humidity) + (" %");
  display.drawString(0, 10, humvar);

  String longvar = String("Long: " + (gps.location.lng(), 6));
  display.drawString(0, 20, longvar);
  Serial.println(longvar);

  Serial.println(String(temperature) + "/" + String(humidity));

  HTTPClient http;
  http.begin(serverName);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String httpRequestData = "client=rudi&temperature=" + String(temperature) + "&humidity= " + String(humidity) + "&longitude=" + String(gps.location.lng(), 6) + "&latitude=" + String(gps.location.lat(), 6) + "&key=123";

  int httpResponseCode = http.POST(httpRequestData);

  if (httpResponseCode > 0)
  {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
    display.drawString(0, 30, "Data sent");
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
    display.drawString(0, 30, "Error");
  }
  display.display();
  http.end();
}

void printGPSSerial()
{
  Serial.print("LAT=");
  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG=");
  Serial.println(gps.location.lng(), 6);
  Serial.print("ALT=");
  Serial.println(gps.altitude.meters());
  Serial.print("Sats=");
  Serial.println(gps.satellites.value());
  Serial.print("DST: ");
  Serial.println(gpsState.dist, 1);
}
