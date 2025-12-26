/*
 * WEATHER STATION SLAVE - DHT11 via HC-05 Bluetooth
 * 
 * Hardware:
 * - Arduino Uno (Slave)
 * - HC-05 Bluetooth Module (configured as SLAVE)
 * - DHT11 Temperature & Humidity Sensor
 * 
 * Connections:
 * DHT11:
 *   VCC → 5V
 *   DATA → Pin 2
 *   GND → GND
 * 
 * HC-05 (with 2 diodes on RX recommended):
 *   VCC → 5V
 *   GND → GND
 *   TX → Pin 10
 *   RX → Pin 11 (through 2 diodes: Pin11→|>|→|>|→RX)
 * 
 * Protocol:
 * - Waits for "REQ" command from master
 * - Responds with 8 bytes: 4 bytes temp (float) + 4 bytes humidity (float)
 */

#include <DHT.h>
#include <SoftwareSerial.h>

// Pin Definitions
#define DHTPIN 2          // DHT11 data pin
#define DHTTYPE DHT11     // DHT sensor type
#define BT_RX 10          // Connect to HC-05 TX
#define BT_TX 11          // Connect to HC-05 RX (through 2 diodes!)

// Initialize sensors and communication
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial BTSerial(BT_RX, BT_TX);

// Sensor data
float currentTemperature = 0.0;
float currentHumidity = 0.0;
unsigned long lastRead = 0;
unsigned long lastSend = 0;

// Statistics
unsigned long requestCount = 0;
unsigned long errorCount = 0;

void setup() {
  // Initialize Serial for debugging (optional - can be removed)
  Serial.begin(9600);
  Serial.println(F("========================================"));
  Serial.println(F("  Weather Station Slave - Starting"));
  Serial.println(F("========================================"));
  
  // Initialize Bluetooth communication
  BTSerial.begin(9600);
  Serial.println(F("[BT] Bluetooth initialized at 9600 baud"));
  
  // Initialize DHT sensor
  dht.begin();
  Serial.println(F("[DHT] Sensor initialized"));
  
  // Wait for sensor to stabilize
  delay(2000);
  
  // Initial sensor read
  Serial.println(F("[INIT] Reading initial sensor data..."));
  readSensor();
  
  Serial.println(F("[READY] Weather station ready!"));
  Serial.println(F("========================================"));
  Serial.println();
}

void loop() {
  // Read sensor data every 2 seconds
  if (millis() - lastRead > 2000) {
    readSensor();
    lastRead = millis();
  }
  
  // Check for request from master
  if (BTSerial.available() >= 3) {
    char cmd[4] = {0};
    BTSerial.readBytes(cmd, 3);
    
    // Check if it's a request command
    if (strcmp(cmd, "REQ") == 0) {
      requestCount++;
      sendData();
      
      Serial.print(F("[TX] Data sent | Request #"));
      Serial.print(requestCount);
      Serial.print(F(" | Temp: "));
      Serial.print(currentTemperature, 1);
      Serial.print(F("°C | Hum: "));
      Serial.print(currentHumidity, 1);
      Serial.println(F("%"));
      
      lastSend = millis();
    }
    
    // Clear any extra bytes in buffer
    while(BTSerial.available()) {
      BTSerial.read();
    }
  }
  
  // Status heartbeat every 10 seconds (optional)
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 10000) {
    Serial.print(F("[STATUS] Running | Temp: "));
    Serial.print(currentTemperature, 1);
    Serial.print(F("°C | Hum: "));
    Serial.print(currentHumidity, 1);
    Serial.print(F("% | Requests: "));
    Serial.print(requestCount);
    Serial.print(F(" | Errors: "));
    Serial.println(errorCount);
    lastHeartbeat = millis();
  }
}

void readSensor() {
  // Read humidity (percentage)
  float h = dht.readHumidity();
  
  // Read temperature as Celsius
  float t = dht.readTemperature();
  
  // Check if readings failed
  if (isnan(h) || isnan(t)) {
    errorCount++;
    Serial.println(F("[ERROR] Failed to read from DHT sensor!"));
    Serial.print(F("  - Humidity: "));
    Serial.println(isnan(h) ? "NaN" : String(h));
    Serial.print(F("  - Temperature: "));
    Serial.println(isnan(t) ? "NaN" : String(t));
    
    // Keep previous valid values or set error codes
    if (currentTemperature == 0.0) {
      currentTemperature = -999.0;
      currentHumidity = -999.0;
    }
    // Otherwise keep last good reading
    
  } else {
    // Valid readings - update current values
    currentTemperature = t;
    currentHumidity = h;
    
    // Debug output
    Serial.print(F("[DHT] Read | Temp: "));
    Serial.print(currentTemperature, 1);
    Serial.print(F("°C | Humidity: "));
    Serial.print(currentHumidity, 1);
    Serial.println(F("%"));
  }
}

void sendData() {
  // Send temperature (4 bytes as float)
  byte tempBytes[4];
  memcpy(tempBytes, &currentTemperature, 4);
  BTSerial.write(tempBytes, 4);
  
  // Send humidity (4 bytes as float)
  byte humBytes[4];
  memcpy(humBytes, &currentHumidity, 4);
  BTSerial.write(humBytes, 4);
  
  // Total: 8 bytes sent
}

/*
 * TROUBLESHOOTING GUIDE:
 * 
 * 1. "Failed to read from DHT sensor!"
 *    - Check DHT11 wiring (VCC, DATA, GND)
 *    - Ensure DHT11 VCC connected to 5V (not 3.3V)
 *    - Try different DHT11 module (might be faulty)
 *    - Add 10kΩ pull-up resistor from DATA to VCC
 * 
 * 2. No serial output:
 *    - Check Serial Monitor baud rate (should be 9600)
 *    - Ensure USB cable is data cable (not power-only)
 * 
 * 3. No Bluetooth communication:
 *    - Check HC-05 wiring (VCC to 5V, not 3.3V!)
 *    - Verify HC-05 TX → Pin 10
 *    - Verify HC-05 RX ← Pin 11 (through diodes)
 *    - Check HC-05 LED (should blink slowly when paired)
 *    - Verify HC-05 configured as SLAVE (AT+ROLE=0)
 *    - Verify HC-05 baud rate is 9600 (AT+UART=9600,0,0)
 * 
 * 4. Master shows "Connecting BT...":
 *    - HC-05 modules not paired
 *    - Check both modules configured correctly
 *    - Ensure master ROLE=1, slave ROLE=0
 *    - Try power cycling both Arduinos
 * 
 * 5. Readings always -999:
 *    - DHT11 never successfully read
 *    - Check all DHT11 connections
 *    - DHT11 may need 2 second warm-up after power on
 *    - Try different DHT11 sensor
 * 
 * 6. Erratic/wrong readings:
 *    - Poor power supply (use good 5V source)
 *    - Long wires on DHT11 (keep under 20cm)
 *    - Electromagnetic interference
 *    - DHT11 too close to heat source
 */