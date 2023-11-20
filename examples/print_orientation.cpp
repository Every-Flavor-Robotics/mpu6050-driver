#include <Arduino.h>

#include "Wire.h"
#include "sensorgo_mpu6050.h"

SensorGoMPU6050 mpu;

// Function to print at a maximum frequency
void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(3000);

  //   Configure Wire on pins 38, 47
  Wire1.begin(38, 47);

  //   Begin IMU on Wire
  if (!mpu.begin(&Wire1))
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      //   Restart the board
      Serial.println("Restarting to try again...");
      ESP.restart();
    }
  }

  mpu.calibrate();
  //   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //   mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  //   mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  //   mpu.setHighPassFilter(MPU6050_HIGHPASS_DISABLE);
}

void loop()
{
  if (mpu.data_ready())
  {
    // Get pitch
    float pitch = mpu.get_pitch();

    freq_println("Pitch: " + String(pitch), 10);
  }
}