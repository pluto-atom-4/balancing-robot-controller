#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// Pin Definitions for ESP32-C3
#define I2C_SDA 8
#define I2C_SCL 9
#define SERVO_PIN 10

Adafruit_MPU6050 mpu;
Servo myServo;

void setup() {
    Serial.begin(115200);
    delay(1000); // no blocking wait: boots without a USB host

    // Initialize I2C with ESP32-C3 specific pins
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 connected successfully.");

    // Initialize Servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myServo.setPeriodHertz(50); // Standard 50hz servo
    myServo.attach(SERVO_PIN, 500, 2400); // SG90 min/max pulses
}

void loop() {
    // Read gyroscope
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Adafruit driver reports gyro in rad/s; convert Z axis to degrees per second
    float rotSpeedZ = gyro.gyro.z * RAD_TO_DEG;

    Serial.print("Rotation Speed Z: ");
    Serial.print(rotSpeedZ);
    Serial.print(" deg/s -> Direction: ");

    if (rotSpeedZ > 5.0) {
        Serial.println("Counter-Clockwise (CCW)");
    } else if (rotSpeedZ < -5.0) {
        Serial.println("Clockwise (CW)");
    } else {
        Serial.println("Stationary");
    }

    // Map rotation speed range (-150 to +150 deg/s) to Servo angle (0 to 180 degrees)
    // Center speed (0 deg/s) maps to 90 degrees (neutral position)
    int servoAngle = map(constrain(rotSpeedZ, -150, 150), -150, 150, 0, 180);

    myServo.write(servoAngle);

    delay(50); // 20 Hz sampling rate
}
