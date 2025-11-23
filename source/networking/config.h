#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "Hello"
#define WIFI_PASSWORD "yosy8997"

// MQTT-SN Gateway Configuration
#define GATEWAY_IP_1 172
#define GATEWAY_IP_2 20
#define GATEWAY_IP_3 10
#define GATEWAY_IP_4 3
#define GATEWAY_PORT 1883

// Client Configuration
#define CLIENT_ID "UltrasonicBot"
#define TOPIC_NAME "pico/ultrasonic/data"

// Sensor Configuration
#define TRIG_PIN 16
#define ECHO_PIN 17
#define SERVO_PIN 1

// Barcode Reader Configuration - ADD THESE LINES
#define BARCODE_IR_PIN 7
#define BARCODE_ENABLED 1

// Threshold Configuration
#define SAFE_STOP_CM 60.0f
#define OCCUPIED_CM 70.0f
#define CLEAR_THRESHOLD_CM 80.0f

// Servo Configuration
#define STEP_DEG 4
#define SETTLE_MS 220

// Kalman Filter Configuration
#define KF_Q 0.05f
#define KF_R 5.0f

// Navigation Configuration
#define PREFER_RIGHT_ON_TIE 1

#endif