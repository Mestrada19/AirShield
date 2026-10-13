// Project: AirShield
// Mesa Center: UCI
// Team: Marcus Morales, Alejandro Castro, Mario Estrada, Angel Olivares
// Smart Home Hub

// Librarys 
#include <HardwareSerial.h>
#include <TFT_eSPI.h>
#include <TM1637Display.h>
#include "RTClib.h"
#include <Wire.h>
#include "DFRobot_AHT20.h"
#include "Adafruit_SGP30.h"
#include <WiFi.h>

// Pins
#define BUZZER_PIN    13  // Active Buzzer for alerts
#define MQ_PIN        34  // Methane Gas Sensor 
#define RXD2          16  // PM Sensor RX
#define TXD2          17  // PM Sensor TX
#define TM_CLK        5   // TM1637 Clock
#define TM_DIO        21  // TM1637 Data
#define I2C_SDA       25  // Primary I2C Data 
#define I2C_SCL       26  // Primary I2C Clock
#define I2C1_SDA      22  // Secondary I2C Data 
#define I2C1_SCL      32  // Secondary I2C Clock

// WiFi Credentials
const char* ssid = "SmartHomeHub";
const char* password = "LhsMesa";
WiFiServer server(80);    // Web server for dashboard + status

// Sensor Data Holders
float tempF = 72.0, humidity = 45.0, tvoc = 0, eco2 = 400, gas = 0;  // float to store numbers with decimal points
uint16_t pm10 = 0, pm25 = 0, pm100 = 0;
bool pm_ok = false, sgp_ok = false, sgp_valid = false;
bool airBad = false;      // true if any PM > 120 (for servo ESP)

// The Hardware Setup
RTC_DS3231 rtc;
DFRobot_AHT20 aht20;
Adafruit_SGP30 sgp;
HardwareSerial pms(1); // Serial port for PM sensor
TFT_eSPI tft = TFT_eSPI();
TM1637Display display(TM_CLK, TM_DIO);

// PM Sensor Data Structure
struct pms5003data {
  uint16_t pm10_env, pm25_env, pm100_env, checksum;
};
pms5003data data;
bool sensorConnected = false;
unsigned long lastSensorTime = 0;
const unsigned long timeout = 5000;

// Main loop
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize WiFi Access Point
  WiFi.softAP(ssid, password);
  server.begin();
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP()); // default 192.168.4.1 [web:21]
  
  // Initialize I2C Bus 1 (RTC/AHT20)
  Wire.begin(I2C_SDA, I2C_SCL);
  aht20.begin();
  
  // Initialize I2C Bus 2 (SGP30)
  Wire1.begin(I2C1_SDA, I2C1_SCL);
  sgp_ok = sgp.begin(&Wire1);
  
  // Start PM Sensor
  pms.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Display Setup
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  
  delay(10000); // Warmup for gas sensor accuracy
  Serial.println("SYSTEM READY");
}

void loop() {
  unsigned long now = millis();
  
  // Handle Web Client Requests
  WiFiClient client = server.available();
  if (client) handleWeb(client);
  
  // Periodic Sensor Reading
  readAHT20(now);
  if (sgp_ok) readSGP30(now);
  readPMS();
  gas = analogRead(MQ_PIN);
  
  delay(30);
}

// Sensor Functions

// Refresh Temp & Hum every 2 seconds 
void readAHT20(unsigned long now) {
  static unsigned long last = 0;
  if (now - last > 2000) {
    last = now;
    if (aht20.startMeasurementReady(true)) {
      tempF = (aht20.getTemperature_C() * 9/5) + 32; // convert tempature to fahrenheit
      humidity = aht20.getHumidity_RH();
    }
  }
}

// Update TVOCs and eCO2 every second
void readSGP30(unsigned long now) {
  static unsigned long last = 0;
  if (now - last > 1000) {
    last = now;
    uint32_t hum = getHumidity(tempF-32, humidity);
    sgp.setHumidity(hum); // Compensate for humidity
    
    if (sgp.IAQmeasure()) {
      tvoc = sgp.TVOC;
      eco2 = sgp.eCO2;
      sgp_valid = true;
      if (tvoc > 120) beep(3, 400); // Alert if TVOC is high
    } else {
      sgp_valid = false;
    }
  }
}

// Read PM Data
bool readPMS() {
  unsigned long now = millis();
  if (pms.available() && pms.peek() == 0x42) {
    uint8_t buf[32];
    if (pms.readBytes(buf, 32) == 32) {
      uint16_t sum = 0;
      for (int i = 0; i < 30; i++) sum += buf[i];
      
      data.pm10_env  = (buf[10] << 8) | buf[11];
      data.pm25_env  = (buf[12] << 8) | buf[13];
      data.pm100_env = (buf[14] << 8) | buf[15];
      data.checksum  = (buf[30] << 8) | buf[31];
      
      if (sum == data.checksum) {
        sensorConnected = true;
        lastSensorTime = now;
        pm_ok = true;
        pm10  = data.pm10_env;
        pm25  = data.pm25_env;
        pm100 = data.pm100_env;
        
        // Alert if PM2.5 is over 35
        if (pm25 > 35) beep(2, 500);
        
        // Set airBad flag for second ESP32 (any PM > 120)
        if (pm10 > 120 || pm25 > 120 || pm100 > 120) {
          airBad = true;
        } else {
          airBad = false;
        }
        
        showValues(); // Update Display
        return true;
      }
    }
  }
  
  // Handle Disconnection
  if (sensorConnected && (now - lastSensorTime > timeout)) {
    sensorConnected = false;
    pm_ok = false;
    showNotConnected();
  }
  return false;
}

// HTML Server

// Handle browser requests
void handleWeb(WiFiClient client) {
  String header;
  unsigned long t = millis();
  while (client.connected() && millis() - t < 2000) {
    if (client.available()) {
      header += (char)client.read();
      if (header.endsWith("\r\n\r\n")) {
        
        // Simple status endpoint for servo ESP32
        if (header.indexOf("GET /s") >= 0) {      // /s = status
          sendStatus(client);                     // send JSON with airBad
        } else {
          sendWebPage(client);                    // normal HTML dashboard
        }
        
        break;
      }
    }
  }
  client.stop();
}

// Send status JSON to second ESP32 (airBad or OK)
void sendStatus(WiFiClient c) {
  c.println("HTTP/1.1 200 OK");
  c.println("Content-Type: application/json");
  c.println("Cache-Control: no-cache");
  c.println("Connection: close");
  c.println();
  c.print("{\"air\":\"");
  c.print(airBad ? "BAD" : "OK");
  c.println("\"}");
}

// Generate the HTML Dashboard
void sendWebPage(WiFiClient c) {
  c.println("HTTP/1.1 200 OK\nContent-Type: text/html\nCache-Control: no-cache\n");
  c.println("<!DOCTYPE html><html><head>");
  c.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  c.println("<style>body{font-family:Arial;background:linear-gradient(135deg,#1e40af,#60a5fa);color:white;text-align:center;}");
  c.println(".container{max-width:500px;margin:auto;background:rgba(255,255,255,0.12);padding:30px;border-radius:20px;}");
  c.println(".sensor{margin:15px 0;padding:20px;background:rgba(255,255,255,0.2);border-radius:15px;} .value{font-size:28px;font-weight:bold;}");
  c.println(".circle{display:inline-block;width:18px;height:18px;border-radius:50%;margin-left:10px;}");
  c.println("</style></head><body><div class='container'><h1>AirShield Dashboard</h1>");
  
  // Sensor Data Rows
  c.print("<div class='sensor'>Temp: <span class='value'>" + String(tempF,1) + " F</span></div>");
  c.print("<div class='sensor'>Humidity: <span class='value'>" + String(humidity,1) + " %</span></div>");
  c.print("<div class='sensor'>PM2.5: <span class='value'>" + String(pm25) + " ug</span></div>");
  c.print("<div class='sensor'>eCO2: <span class='value'>" + String(eco2,0) + " ppm</span></div>");
  
  c.println("</div></body></html>");
}

// Color coding on HTML server based off safety levels
String getColor(float val, String type) {
  if (type == "temp") return val < 80 ? "limegreen" : "red";
  if (type == "pm") return val < 35 ? "limegreen" : "red";
  if (type == "gas") return val < 500 ? "limegreen" : "red";
  return "gray";
}

// Buzzer on/off
void beep(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(ms);
    digitalWrite(BUZZER_PIN, LOW);
    delay(ms/2);
  }
}

// Update the display
void showValues() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(130, 15); tft.setTextSize(2); tft.print("AirShield");
  
  tft.setTextSize(1);
  tft.setCursor(15, 100);  tft.print("PM2.5: "); tft.print(pm25);
  tft.setCursor(175, 100); tft.print("Gas: ");   tft.print(gas,0);
  tft.setCursor(355, 110); tft.setTextSize(2);  tft.print(tempF,0); tft.print("F");
}

void showNotConnected() {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(130, 160); tft.print("PM SENSOR ERROR");  // Display possible errors with PM sensor
}

// Humidity calculation for SGP30 
uint32_t getHumidity(float f, float rh) {
  float t = (f - 32) * 5/9;
  float abs = 216.7f * ((rh/100) * 6.112f * exp((17.62f * t)/(243.12f + t)) / (273.15f + t));
  return (uint32_t)(1000 * abs);
}
