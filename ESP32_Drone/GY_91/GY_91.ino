#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>

// Create instances for MPU6050 (IMU) and BMP280 (barometric sensor)
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1);
  }
  Serial.println("MPU6050 initialized successfully!");

  // Initialize BMP280
  if (!bmp.begin(0x76)) { // Default I2C address for BMP280
    Serial.println("Failed to find BMP280 sensor!");
    while (1);
  }
  Serial.println("BMP280 initialized successfully!");
}

void loop() {
  // Read accelerometer and gyroscope data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print accelerometer data
  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2, ");
  Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s^2, ");
  Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s^2");

  // Print gyroscope data
  Serial.print("Gyro X: "); Serial.print(g.gyro.x); Serial.print(" rad/s, ");
  Serial.print("Y: "); Serial.print(g.gyro.y); Serial.print(" rad/s, ");
  Serial.print("Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");

  // Read and print barometric pressure
  Serial.print("Pressure: "); Serial.print(bmp.readPressure()); Serial.println(" Pa");

  // Delay for readability
  delay(1000);
}
