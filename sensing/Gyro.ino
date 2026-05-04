#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_VL6180X.h"
#include <math.h>

Adafruit_VL6180X vl = Adafruit_VL6180X();
Adafruit_MPU6050 mpu;

// Filter variables
float anglePitch = 0; // The final calculated Pitch angle
float angleRoll = 0;  // The final calculated Roll angle
float angleYaw = 0;   // The final calculated Yaw angle (will drift)

// Gyro bias values (determined during calibration)
float gyroX_bias = 0;
float gyroY_bias = 0;
float gyroZ_bias = 0;

// Timer variable to calculate the time difference (dt)
unsigned long timer;

// Complementary filter coefficient (alpha)
// A high value (e.g., 0.98) trusts the gyro more.
// A low value (e.g., 0.90) trusts the accelerometer more.
const float alpha = 0.98;

void setup(void) 
{
  Serial.begin(115200);
  while (!Serial) 
  {
    delay(10); // wait for serial port to connect
  }

  // Initialize the MPU6050
  if (!mpu.begin())
   {
    Serial.println("Failed to find MPU6050 chip");
    while (1) 
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // Set gyro range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // CORRECTED: Was MPU6050_RANGE_500_DPS
  // Set filter bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // --- Gyro Calibration ---
  // The gyro will have a small error (bias) even when still.
  // We need to measure this error and subtract it later.
  Serial.println("Calibrating gyro. Keep the sensor still!");
  Serial.println("Please wait, this will take 5-10 seconds...");

  int numReadings = 500;
  for (int i = 0; i < numReadings; i++) 
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Sum up the readings
    gyroX_bias += g.gyro.x;
    gyroY_bias += g.gyro.y;
    gyroZ_bias += g.gyro.z;
    delay(10); // Wait 10ms
  }
  
  // Average the readings to get the bias
  gyroX_bias /= numReadings;
  gyroY_bias /= numReadings;
  gyroZ_bias /= numReadings;

  Serial.println("Calibration complete!");
  Serial.print("Gyro X Bias (rad/s): "); Serial.println(gyroX_bias);
  Serial.print("Gyro Y Bias (rad/s): "); Serial.println(gyroY_bias);
  Serial.print("Gyro Z Bias (rad/s): "); Serial.println(gyroZ_bias);
  Serial.println("------------------------------------");

  // --- Initial Angle Calculation ---
  // Set the initial angle of the filter to the angle
  // calculated by the accelerometer. This prevents a "jump"
  // at the beginning.
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // The Adafruit library returns gyro data in rad/s
  // and accelerometer data in m/s^2
  
  // Calculate initial Roll (rotation around X-axis)
  angleRoll = atan2(a.acceleration.y, a.acceleration.z) * (180.0 / M_PI);
  
  // Calculate initial Pitch (rotation around Y-axis)
  // We use the sqrt of (y^2 + z^2) to be more robust
  anglePitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * (180.0 / M_PI);
  
  // We assume Yaw starts at 0
  angleYaw = 0;
  
  // Start the timer
  timer = millis();

  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
}

void loop() {
  // --- 1. Calculate Delta-Time (dt) ---
  // Find the time elapsed since the last loop in seconds
  unsigned long currentMillis = millis();
  float dt = (currentMillis - timer) / 1000.0;
  timer = currentMillis; // Reset the timer

  // --- 2. Get Raw Sensor Data ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- 3. Calculate Angle from Accelerometer ---
  // These values are in degrees
  float angleRollAccel = atan2(a.acceleration.y, a.acceleration.z) * (180.0 / M_PI); // CORRECTED: Was a.acceleration.m
  float anglePitchAccel = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * (180.0 / M_PI);

  // --- 4. Get Gyro Rates (and remove bias) ---
  // The gyro data is in rad/s. We subtract the bias and
  // convert to degrees/s.
  float gyroX_deg = (g.gyro.x - gyroX_bias) * (180.0 / M_PI);
  float gyroY_deg = (g.gyro.y - gyroY_bias) * (180.0 / M_PI);
  float gyroZ_deg = (g.gyro.z - gyroZ_bias) * (180.0 / M_PI);

  // --- 5. Apply the Complementary Filter ---
  
  // Pitch (rotation around Y-axis)
  // Gyro part: Integrate the gyro rate to get the change in angle
  // Accelerometer part: The raw angle from the accelerometer
  anglePitch = alpha * (anglePitch + gyroY_deg * dt) + (1.0 - alpha) * (anglePitchAccel);

  // Roll (rotation around X-axis)
  // Gyro part: Integrate the gyro rate to get the change in angle
  // Accelerometer part: The raw angle from the accelerometer
  angleRoll = alpha * (angleRoll + gyroX_deg * dt) + (1.0 - alpha) * (angleRollAccel);

  // Yaw (rotation around Z-axis)
  // NOTE: This is based *only* on the gyro and WILL DRIFT over time.
  // There is no accelerometer data to correct it.
  angleYaw = angleYaw + gyroZ_deg * dt;


  // --- 6. Print the Results ---
  Serial.print("Roll: ");
  Serial.print(angleRoll, 2); // Print with 2 decimal places

  Serial.print("\t Pitch: ");
  Serial.print(anglePitch, 2);

  Serial.print("\t Yaw: ");
  Serial.print(angleYaw, 2);
  Serial.println();
  
  // A small delay so we don't flood the serial monitor
  // The filter itself works better with no delay,
  // but for printing, this is helpful.
  delay(10); 

  float lux = vl.readLux(VL6180X_ALS_GAIN_5);

  Serial.print("Lux: "); Serial.println(lux);
  
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  Serial.print("Range: "); Serial.println(range);
}
