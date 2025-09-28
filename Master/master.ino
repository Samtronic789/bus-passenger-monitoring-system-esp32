#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <TinyGPSPlus.h>

// ===== PIN CONFIGURATIONS =====
#define RS485_TX   4    // RS485 DI
#define RS485_RX   15   // RS485 RO
#define RS485_DE   27   // RS485 Driver Enable
#define RS485_RE   2    // RS485 Receiver Enable
#define A9G_RX_PIN 16   // ESP32 RX ← A9G TX (AT commands)
#define A9G_TX_PIN 17   // ESP32 TX → A9G RX (AT commands)
#define A9G_RST    21   // A9G Reset pin
#define LED_PIN    2    // Status LED
#define GPS_RX_PIN 34   // ESP32 RX ← A9G GPS_TX (NMEA data)
#define GPS_TX_PIN 35   // ESP32 TX → A9G GPS_RX (not used)
#define TRIG1      5    // Door sensor 1 trigger
#define ECHO1      18   // Door sensor 1 echo
#define TRIG2      25   // Door sensor 2 trigger
#define ECHO2      26   // Door sensor 2 echo
#define TRIG3      21   // Inner sensor 1 trigger
#define ECHO3      22   // Inner sensor 1 echo
#define TRIG4      23   // Inner sensor 2 trigger
#define ECHO4      19   // Inner sensor 2 echo

// ===== SERIAL PORTS =====
HardwareSerial RS485Serial(1); // RS485 communication
HardwareSerial A9G_Serial(2);  // A9G AT commands
HardwareSerial GPS_Serial(3);  // GPS NMEA data

// ===== BUS INFO =====
const int totalSeats = 42;
int masterCount = 0;
int slaveCount = 0;
String busID = "NB1234";
String routeID = "87";

// ===== GPS/Location =====
TinyGPSPlus gps;
float latitude = 0.0;
float longitude = 0.0;
float speedKmh = 0.0;
float prevLatitude = 0.0;
float prevLongitude = 0.0;
unsigned long prevGpsTime = 0;
const unsigned long gpsInterval = 20000; // 20 seconds

// ===== A9G MQTT CONFIG =====
const char* APN = "dialog";
const char* MQTT_BROKER = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
const uint16_t MQTT_KEEPALIVE = 120;
const size_t SERIAL_BUFFER_SIZE = 2048;

// ===== STATE MANAGEMENT =====
struct SystemState {
  bool a9g_initialized : 1;
  bool mqtt_connected : 1;
  bool gps_enabled : 1;
  String sta_mac;
  uint32_t mqtt_success_count;
  uint32_t mqtt_failure_count;
  bool count_changed : 1;
  bool gps_changed : 1;
  unsigned long last_gps_update;
} state = {0};

// ===== PASSENGER TRACKING =====
enum PassengerState { IDLE, DOOR_TRIGGERED, INNER_TRIGGERED };
PassengerState passengerState = IDLE;
unsigned long lastDetectTime = 0;
const int detectThreshold = 50; // cm
const unsigned long debounceDelay = 200; // ms
const unsigned long timeoutDelay = 2000; // ms

// ===== UTILITY FUNCTIONS =====
float haversineDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371.0; // Earth's radius in km
  float lat1Rad = radians(lat1);
  float lat2Rad = radians(lat2);
  float deltaLat = radians(lat2 - lat1);
  float deltaLon = radians(lon2 - lon1);
  float a = sin(deltaLat / 2) * sin(deltaLat / 2) +
            cos(lat1Rad) * cos(lat2Rad) * sin(deltaLon / 2) * sin(deltaLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c; // Distance in km
}

// ===== ULTRASONIC FUNCTIONS =====
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return (duration == 0) ? 999 : duration * 0.034 / 2; // cm
}

void setupUltrasonic() {
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);
  pinMode(TRIG4, OUTPUT); pinMode(ECHO4, INPUT);
  Serial.println("Ultrasonic sensors initialized: Door (TRIG1/ECHO1, TRIG2/ECHO2), Inner (TRIG3/ECHO3, TRIG4/ECHO4)");
}

bool readDoorSensors() {
  return getDistance(TRIG1, ECHO1) < detectThreshold || getDistance(TRIG2, ECHO2) < detectThreshold;
}

bool readInnerSensors() {
  return getDistance(TRIG3, ECHO3) < detectThreshold || getDistance(TRIG4, ECHO4) < detectThreshold;
}

void updatePassengerCount() {
  bool doorDetected = readDoorSensors();
  bool innerDetected = readInnerSensors();
  unsigned long now = millis();

  switch (passengerState) {
    case IDLE:
      if (doorDetected && now - lastDetectTime > debounceDelay) {
        passengerState = DOOR_TRIGGERED;
        lastDetectTime = now;
        Serial.println("[STATE] Door triggered → possible ENTRY");
      } else if (innerDetected && now - lastDetectTime > debounceDelay) {
        passengerState = INNER_TRIGGERED;
        lastDetectTime = now;
        Serial.println("[STATE] Inner triggered → possible EXIT");
      }
      break;

    case DOOR_TRIGGERED:
      if (innerDetected && now - lastDetectTime > debounceDelay) {
        masterCount++;
        if (masterCount + slaveCount > totalSeats) masterCount = totalSeats - slaveCount;
        Serial.println("[PASSENGER] ENTRY detected - Master count: " + String(masterCount));
        passengerState = IDLE;
        state.count_changed = true;
        delay(500);
      } else if (!doorDetected || now - lastDetectTime > timeoutDelay) {
        Serial.println("[STATE] Door trigger " + String(!doorDetected ? "false" : "timeout") + " - returning to IDLE");
        passengerState = IDLE;
      }
      break;

    case INNER_TRIGGERED:
      if (doorDetected && now - lastDetectTime > debounceDelay) {
        if (masterCount > 0) {
          masterCount--;
          Serial.println("[PASSENGER] EXIT detected - Master count: " + String(masterCount));
        } else {
          Serial.println("[PASSENGER] EXIT detected but master count already at 0");
        }
        passengerState = IDLE;
        state.count_changed = true;
        delay(500);
      } else if (!innerDetected || now - lastDetectTime > timeoutDelay) {
        Serial.println("[STATE] Inner trigger " + String(!innerDetected ? "false" : "timeout") + " - returning to IDLE");
        passengerState = IDLE;
      }
      break;
  }
}

// ===== RS485 FUNCTIONS =====
void rs485Transmit() {
  digitalWrite(RS485_DE, HIGH);
  digitalWrite(RS485_RE, HIGH);
}

void rs485Receive() {
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);
}

void readSlaveResponse() {
  if (RS485Serial.available()) {
    String msg = RS485Serial.readStringUntil('\n');
    msg.trim();
    if (msg.length() > 0) {
      Serial.println("[RS485] Received from slave: " + msg);
      if (msg.startsWith("{") && msg.endsWith("}")) {
        int startIndex = msg.indexOf("\"passenger_count\":");
        if (startIndex != -1) {
          startIndex += 18;
          int endIndex = msg.indexOf("}", startIndex);
          if (endIndex != -1) {
            String countStr = msg.substring(startIndex, endIndex);
            countStr.trim();
            int newSlaveCount = countStr.toInt();
            if (newSlaveCount >= 0 && newSlaveCount != slaveCount) {
              Serial.println("[RS485] Slave count changed: " + String(slaveCount) + " → " + String(newSlaveCount));
              slaveCount = newSlaveCount;
              state.count_changed = true;
            }
          } else {
            Serial.println("[RS485] Error: Invalid JSON format - missing closing brace");
          }
        } else {
          Serial.println("[RS485] Error: Invalid JSON format - missing passenger_count key");
        }
      } else {
        Serial.println("[RS485] Error: Received non-JSON message");
      }
    }
  }
}

// ===== A9G FUNCTIONS =====
String getStaMacString() {
  WiFi.mode(WIFI_STA);
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

bool waitForResponse(const String& expected, uint32_t timeout_ms) {
  unsigned long start = millis();
  String buf;
  buf.reserve(512);
  while (A9G_Serial.available()) A9G_Serial.read();
  while (millis() - start < timeout_ms) {
    while (A9G_Serial.available()) {
      char c = A9G_Serial.read();
      buf += c;
      if (buf.indexOf(expected) != -1) return true;
      if (buf.indexOf("ERROR") != -1) return false;
      if (buf.length() > 1024) buf.remove(0, 512);
    }
    delay(10);
  }
  return false;
}

bool sendAT(const String& cmd, const String& expect = "OK", uint32_t timeout_ms = 5000) {
  Serial.println(">> " + cmd);
  A9G_Serial.println(cmd);
  return waitForResponse(expect, timeout_ms);
}

void a9gPowerOnSequence() {
  pinMode(A9G_RST, OUTPUT);
  digitalWrite(A9G_RST, LOW);
  delay(1100);
  pinMode(A9G_RST, INPUT_PULLUP);
  delay(3000);
}

void a9gResetPulse() {
  pinMode(A9G_RST, OUTPUT);
  digitalWrite(A9G_RST, LOW);
  delay(150);
  digitalWrite(A9G_RST, HIGH);
  delay(3000);
}

bool initializeA9G_MQTT() {
  delay(8000);
  a9gPowerOnSequence();
  a9gResetPulse();
  for (int i = 0; i < 5; ++i) {
    if (sendAT("AT")) break;
    if (i == 4) return false;
    delay(1000);
  }
  sendAT("ATI");
  sendAT("AT+CMEE=2");
  sendAT("AT+CSQ");
  sendAT("AT+CREG?");
  if (!sendAT("AT+GPS=1", "OK", 5000)) {
    Serial.println("[!] GPS power ON failed, continuing without GPS");
    state.gps_enabled = false;
  } else {
    Serial.println("[✓] GPS powered ON");
    state.gps_enabled = true;
  }
  sendAT("AT+CGACT=0,1", "OK", 10000);
  sendAT("AT+CGACT=0,2", "OK", 5000);
  sendAT("AT+CGACT=0,3", "OK", 5000);
  delay(2000);
  if (!sendAT("AT+CGATT=1", "OK", 15000)) return false;
  if (!sendAT("AT+CGDCONT=1,\"IP\",\"" + String(APN) + "\"")) return false;
  if (!sendAT("AT+CGACT=1,1", "OK", 20000)) return false;
  String clientId = "a9g_mqtt_test_" + state.sta_mac;
  clientId.replace(":", "");
  String mqttCmd = "AT+MQTTCONN=\"" + String(MQTT_BROKER) + "\"," + String(MQTT_PORT) +
                   ",\"" + clientId + "\"," + String(MQTT_KEEPALIVE) + ",0";
  if (!sendAT(mqttCmd, "OK", 20000)) return false;
  state.mqtt_connected = true;
  return true;
}

bool publishMqttMessage(const String& topic, const String& message) {
  if (!state.a9g_initialized || !state.mqtt_connected) {
    Serial.println("[MQTT] Not initialized or connected");
    return false;
  }
  Serial.println("[MQTT] Publishing to topic: " + topic);
  Serial.println("[MQTT] Payload: " + message);
  String escaped = message;
  escaped.replace("\"", "\\\"");
  String pubCmd = "AT+MQTTPUB=\"" + topic + "\",\"" + escaped + "\",0,0,0";
  A9G_Serial.println(pubCmd);
  if (waitForResponse("OK", 10000)) {
    Serial.println("[MQTT] Publish successful");
    state.mqtt_success_count++;
    return true;
  }
  Serial.println("[MQTT] Standard format failed, trying alternative...");
  pubCmd = "AT+MQTTPUB=\"" + topic + "\",\"" + escaped + "\"";
  A9G_Serial.println(pubCmd);
  if (waitForResponse("OK", 10000)) {
    Serial.println("[MQTT] Publish successful (alternative format)");
    state.mqtt_success_count++;
    return true;
  }
  Serial.println("[MQTT] Publish failed");
  state.mqtt_failure_count++;
  return false;
}

void publishBusData() {
  int totalPassengers = masterCount + slaveCount;
  int availableSeats = totalSeats - totalPassengers;
  String payload = "{\"id\":\"" + busID + "\",\"route_id\":\"" + routeID + "\",\"lat\":" +
                   String(latitude, 6) + ",\"lon\":" + String(longitude, 6) +
                   ",\"seats\":" + String(availableSeats) + ",\"speed_kmh\":" + String(speedKmh, 1) + "}";
  String topic = "busservice/" + routeID + "/" + busID;
  if (publishMqttMessage(topic, payload)) {
    Serial.println("[BUS DATA] Published - Available seats: " + String(availableSeats) +
                  " (Master: " + String(masterCount) + ", Slave: " + String(slaveCount) + ")");
  }
}

void updateGPSData() {
  while (GPS_Serial.available()) gps.encode(GPS_Serial.read());
  static unsigned long lastGpsCheck = 0;
  if (millis() - lastGpsCheck >= gpsInterval) {
    lastGpsCheck = millis();
    if (gps.location.isValid()) {
      float newLat = gps.location.lat();
      float newLng = gps.location.lng();
      if (newLat != 0.0 && newLng != 0.0) {
        if (prevLatitude != 0.0 && prevLongitude != 0.0 && prevGpsTime != 0) {
          float distanceKm = haversineDistance(prevLatitude, prevLongitude, newLat, newLng);
          float timeHours = (lastGpsCheck - prevGpsTime) / 3600000.0; // Convert ms to hours
          speedKmh = (timeHours > 0) ? distanceKm / timeHours : 0.0;
          Serial.println("[GPS] Calculated speed: " + String(speedKmh, 1) + " km/h (Distance: " + String(distanceKm, 6) + " km)");
        }
        latitude = newLat;
        longitude = newLng;
        prevLatitude = newLat;
        prevLongitude = newLng;
        prevGpsTime = lastGpsCheck;
        state.last_gps_update = millis();
        state.gps_changed = true;
        Serial.print("[GPS] Lat: "); Serial.print(newLat, 6);
        Serial.print("  Lng: "); Serial.print(newLng, 6);
        if (gps.satellites.isValid()) Serial.print("  Sats: " + String(gps.satellites.value()));
        Serial.println();
      }
    } else {
      Serial.println("[GPS] No fix yet... Satellites: " + String(gps.satellites.isValid() ? gps.satellites.value() : 0));
      speedKmh = 0.0;
      state.gps_changed = true;
    }
  }
}

// ===== SERIAL COMMANDS =====
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  cmd.toUpperCase();
  if (cmd == "HELP") {
    Serial.println("\n=== Bus Tracking Commands ===");
    Serial.println("HELP      - Show this help");
    Serial.println("STATUS    - Show bus status");
    Serial.println("STATS     - Show MQTT statistics");
    Serial.println("SENSORS   - Show sensor readings");
    Serial.println("SLAVE     - Show slave count");
    Serial.println("GPS <lat> <lon> <speed> - Set GPS data (manual override)");
    Serial.println("RESET     - Reset passenger counts");
    Serial.println("RECONNECT - Reconnect MQTT");
    Serial.println("RESTART   - Restart ESP32");
  } else if (cmd == "STATUS") {
    Serial.println("\n=== Bus System Status ===");
    Serial.println("A9G Initialized: " + String(state.a9g_initialized ? "YES" : "NO"));
    Serial.println("MQTT Connected: " + String(state.mqtt_connected ? "YES" : "NO"));
    Serial.println("GPS Enabled: " + String(state.gps_enabled ? "YES" : "NO"));
    Serial.println("Bus ID: " + busID);
    Serial.println("Route ID: " + routeID);
    Serial.println("MAC Address: " + state.sta_mac);
    Serial.println("Master Count: " + String(masterCount));
    Serial.println("Slave Count: " + String(slaveCount));
    Serial.println("Total Passengers: " + String(masterCount + slaveCount));
    Serial.println("Available Seats: " + String(totalSeats - masterCount - slaveCount));
    Serial.println("Current Location: " + String(latitude, 6) + ", " + String(longitude, 6));
    Serial.println("Speed: " + String(speedKmh, 1) + " km/h");
    Serial.println("Tracker State: " + String(passengerState == IDLE ? "IDLE" :
                                             passengerState == DOOR_TRIGGERED ? "DOOR_TRIGGERED" : "INNER_TRIGGERED"));
  } else if (cmd == "STATS") {
    Serial.println("\n=== MQTT Statistics ===");
    Serial.println("Successful: " + String(state.mqtt_success_count));
    Serial.println("Failed: " + String(state.mqtt_failure_count));
    if (state.mqtt_success_count + state.mqtt_failure_count > 0) {
      float success_rate = (float)state.mqtt_success_count / (state.mqtt_success_count + state.mqtt_failure_count) * 100;
      Serial.println("Success Rate: " + String(success_rate, 1) + "%");
    }
  } else if (cmd == "SENSORS") {
    Serial.println("\n=== Sensor Readings ===");
    long d1 = getDistance(TRIG1, ECHO1);
    long d2 = getDistance(TRIG2, ECHO2);
    long d3 = getDistance(TRIG3, ECHO3);
    long d4 = getDistance(TRIG4, ECHO4);
    Serial.println("Door Sensor 1: " + String(d1) + "cm" + (d1 < detectThreshold ? " [TRIGGERED]" : ""));
    Serial.println("Door Sensor 2: " + String(d2) + "cm" + (d2 < detectThreshold ? " [TRIGGERED]" : ""));
    Serial.println("Inner Sensor 1: " + String(d3) + "cm" + (d3 < detectThreshold ? " [TRIGGERED]" : ""));
    Serial.println("Inner Sensor 2: " + String(d4) + "cm" + (d4 < detectThreshold ? " [TRIGGERED]" : ""));
    Serial.println("Current State: " + String(passengerState == IDLE ? "IDLE" :
                                             passengerState == DOOR_TRIGGERED ? "DOOR_TRIGGERED" : "INNER_TRIGGERED"));
  } else if (cmd == "SLAVE") {
    Serial.println("\n=== Slave Information ===");
    Serial.println("Current Slave Count: " + String(slaveCount));
    Serial.println("Communication: Automatic (slave sends when count changes)");
  } else if (cmd.startsWith("GPS ")) {
    String params = cmd.substring(4);
    int space1 = params.indexOf(' ');
    int space2 = params.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > space1) {
      latitude = params.substring(0, space1).toFloat();
      longitude = params.substring(space1 + 1, space2).toFloat();
      speedKmh = params.substring(space2 + 1).toFloat();
      prevLatitude = latitude;
      prevLongitude = longitude;
      prevGpsTime = millis();
      Serial.println("GPS manually updated: " + String(latitude, 6) + ", " + String(longitude, 6) +
                    " @ " + String(speedKmh, 1) + " km/h");
      state.gps_changed = true;
    } else {
      Serial.println("Usage: GPS <latitude> <longitude> <speed_kmh>");
    }
  } else if (cmd == "RESET") {
    masterCount = 0;
    slaveCount = 0;
    passengerState = IDLE;
    state.count_changed = true;
    Serial.println("Passenger counts and tracker state reset");
  } else if (cmd == "RECONNECT") {
    state.mqtt_connected = false;
    sendAT("AT+MQTTDISC", "OK", 5000);
    delay(2000);
    String clientId = "a9g_mqtt_test_" + state.sta_mac;
    clientId.replace(":", "");
    String mqttCmd = "AT+MQTTCONN=\"" + String(MQTT_BROKER) + "\"," + String(MQTT_PORT) +
                     ",\"" + clientId + "\"," + String(MQTT_KEEPALIVE) + ",0";
    if (sendAT(mqttCmd, "OK", 15000)) {
      state.mqtt_connected = true;
      Serial.println("MQTT reconnected successfully");
    } else {
      Serial.println("MQTT reconnection failed");
    }
  } else if (cmd == "RESTART") {
    Serial.println("Restarting in 3 seconds...");
    delay(3000);
    ESP.restart();
  } else {
    Serial.println("Unknown command. Type HELP for available commands.");
  }
}

// ===== SETUP AND LOOP =====
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 Bus Tracking System ===");
  Serial.println("Type HELP for commands\n");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  RS485Serial.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);
  rs485Receive();

  A9G_Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
  A9G_Serial.begin(115200, SERIAL_8N1, A9G_RX_PIN, A9G_TX_PIN);

  GPS_Serial.setRxBufferSize(1024);
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS UART initialized: RX=" + String(GPS_RX_PIN) + ", TX=" + String(GPS_TX_PIN));

  setupUltrasonic();

  state.sta_mac = getStaMacString();
  Serial.println("System Info:");
  Serial.println("Bus: " + busID + " | Route: " + routeID);
  Serial.println("MAC: " + state.sta_mac);
  Serial.println("MQTT Topic: busservice/" + routeID + "/" + busID);

  Serial.println("\n=== Initializing A9G MQTT and GPS ===");
  bool initSuccess = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.println("Attempt " + String(attempt) + "/3...");
    if (initializeA9G_MQTT()) {
      state.a9g_initialized = true;
      initSuccess = true;
      Serial.println("A9G MQTT initialized successfully!");
      break;
    }
    Serial.println("Attempt " + String(attempt) + " failed");
    if (attempt < 3) {
      Serial.println("Retrying in 10 seconds...");
      delay(10000);
    }
  }

  if (!initSuccess) {
    Serial.println("\nA9G MQTT INITIALIZATION FAILED");
    while (true) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
      handleSerialCommands();
    }
  }

  digitalWrite(LED_PIN, LOW);
  Serial.println("\nBUS TRACKING SYSTEM READY!");
  Serial.println("Publishes MQTT on passenger count changes, GPS updates, or every 30 seconds if no GPS fix");
  if (state.gps_enabled) Serial.println("GPS enabled - using dedicated UART. Go outside for best results.");
  state.count_changed = true; // Initial publish
}

void loop() {
  handleSerialCommands();
  updatePassengerCount();
  readSlaveResponse();
  if (state.gps_enabled) updateGPSData();

  static unsigned long lastPublish = 0;
  if (state.a9g_initialized && state.mqtt_connected &&
      (state.count_changed || state.gps_changed || (millis() - lastPublish > 30000 && !gps.location.isValid()))) {
    Serial.println("[SENDER] Publishing bus data (count, GPS, or no-fix timeout)");
    state.count_changed = false;
    state.gps_changed = false;
    lastPublish = millis();
    publishBusData();
  }
  delay(50);
}
