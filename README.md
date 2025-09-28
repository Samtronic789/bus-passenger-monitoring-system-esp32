🚌 Bus Passenger Monitoring System

A real-time IoT-based passenger monitoring system for buses, built using ESP32, ultrasonic sensors, RS485 communication, A9G GPS module, and MQTT.

This system helps monitor the number of passengers getting in and out of a bus, along with location tracking, and publishes the data to a cloud MQTT broker for further analysis or integration into smart transportation dashboards.

🔧 System Overview

Total Sensors: 8 Ultrasonic sensors

Modules:

Master Node:

4 Ultrasonic Sensors

ESP32 Dev Board

RS485 TTL Module (to communicate with Slave)

A9G Module (GPS + GSM/GPRS)

Handles MQTT publishing

Slave Node:

4 Ultrasonic Sensors

ESP32 Dev Board

RS485 TTL Module (to send data to Master)

Communication Flow:

Slave counts passengers (4 sensors) → sends data via RS485 → Master

Master counts passengers (4 sensors) + aggregates Slave data → attaches GPS → publishes to MQTT topic
