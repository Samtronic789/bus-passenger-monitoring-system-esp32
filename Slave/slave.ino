#include <HardwareSerial.h>

// RS485 config
#define RS485_TX   4    // DI
#define RS485_RX   15   // RO
#define RS485_DE   27   // DE+RE
HardwareSerial RS485Serial(1);

// Ultrasonic pins
#define TRIG1 5
#define ECHO1 18
#define TRIG2 25
#define ECHO2 26
#define TRIG3 21
#define ECHO3 22
#define TRIG4 23
#define ECHO4 19

const int totalSeats = 42;
int passengerCount = 0;
int lastSentCount = -1;

// Detection threshold (cm)
const int detectThreshold = 50;

// For sequence tracking
enum State { IDLE, DOOR_TRIGGERED, INNER_TRIGGERED };
State state = IDLE;

unsigned long lastDetectTime = 0;

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return 999; // Timeout - return large distance
  return duration * 0.034 / 2;  // cm
}

bool detected(long distance) {
  return (distance < detectThreshold);
}

void rs485Transmit() { digitalWrite(RS485_DE, HIGH); }
void rs485Receive()  { digitalWrite(RS485_DE, LOW);  }

void sendPassengerCount() {
  if (passengerCount != lastSentCount) {
    String msg = "{\"passenger_count\":" + String(passengerCount) + "}";
    RS485Serial.flush(); // Clear buffer
    rs485Transmit();
    RS485Serial.println(msg);
    delay(50); // Ensure reliable transmission
    rs485Receive();
    lastSentCount = passengerCount;
    Serial.println("Slave sent: " + msg);
  }
}

void setup() {
  Serial.begin(115200);
  RS485Serial.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE, OUTPUT);
  rs485Receive();

  // Setup ultrasonic pins
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);
  pinMode(TRIG4, OUTPUT); pinMode(ECHO4, INPUT);

  Serial.println("SLAVE ready...");
}

void loop() {
  // Read distances
  long d1 = getDistance(TRIG1, ECHO1);
  long d2 = getDistance(TRIG2, ECHO2);
  long d3 = getDistance(TRIG3, ECHO3);
  long d4 = getDistance(TRIG4, ECHO4);

  bool doorDetected = detected(d1) || detected(d2);
  bool innerDetected = detected(d3) || detected(d4);

  unsigned long now = millis();
  bool countChanged = false;

  switch (state) {
    case IDLE:
      if (doorDetected) {
        state = DOOR_TRIGGERED;
        lastDetectTime = now;
        Serial.println("Door triggered → possible ENTRY");
      } else if (innerDetected) {
        state = INNER_TRIGGERED;
        lastDetectTime = now;
        Serial.println("Inner triggered → possible EXIT");
      }
      break;

    case DOOR_TRIGGERED:
      if (innerDetected) {
        passengerCount++;
        if (passengerCount > totalSeats) passengerCount = totalSeats;
        Serial.println("Passenger ENTERED → Count = " + String(passengerCount));
        state = IDLE;
        countChanged = true;
        delay(500); // Prevent double-counting
      } else if (!doorDetected) {
        Serial.println("False door trigger - returning to IDLE");
        state = IDLE;
      }
      break;

    case INNER_TRIGGERED:
      if (doorDetected) {
        passengerCount--;
        if (passengerCount < 0) passengerCount = 0;
        Serial.println("Passenger EXITED → Count = " + String(passengerCount));
        state = IDLE;
        countChanged = true;
        delay(500); // Prevent double-counting
      } else if (!innerDetected) {
        Serial.println("False inner trigger - returning to IDLE");
        state = IDLE;
      }
      break;
  }

  // Send passenger count via RS485 only if changed
  if (countChanged) {
    sendPassengerCount();
  }

  delay(50); // Match master's loop responsiveness
}
