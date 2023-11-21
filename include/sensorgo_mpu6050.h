// Guards
#ifndef MPU6050_H_
#define MPU6050_H_

#include "MPU6050_6Axis_MotionApps612.h"

#define _PI_2 1.57079632679489661923f

class SensorGoMPU6050
{
 public:
  SensorGoMPU6050();
  //   ~MPU6050();

  bool begin(TwoWire *wire = &Wire);
  bool begin(uint8_t addr, TwoWire *wire = &Wire);

  bool calibrate();

  bool data_ready();

  float get_pitch();
  float get_roll();
  float get_yaw();

  //   void set_accelerometer_range(mpu6050_accel_range accel_range);
  //   void set_gyro_range(mpu6050_gyro_range gyro_range);
  //   void set_filter_bandwidth(mpu6050_bandwidth bandwidth);
  //   void set_high_pass_filter(mpu6050_high_pass_filter filter);
  //   void get_event(sensors_event_t *accel, sensors_event_t *gyro,
  //                  sensors_event_t *temp);
  //   void get_gyro_event(sensors_event_t *gyro);
  //   void get_accel_event(sensors_event_t *accel);
  //   void get_temp_event(sensors_event_t *temp);
  //   void get_orientation(sensors_vec_t *orientation, sensors_event_t *accel,
  //                        sensors_event_t *mag);
  //   void get_sensor(sensor_t *accel, sensor_t *gyro, sensor_t *temp);

  //   void set_sleep_enabled(bool enabled);
  //   bool get_sleep_enabled(void);

  //   void set_clock_source(uint8_t source);
  //   uint8_t get_clock_source(void);

  //   void set_full_scale_gyro_range(uint8_t range);
  //   uint8_t get_full_scale_gyro_range(void);

  //   void set_full_scale_accel_range(uint8_t range);
  //   uint8_t get_full_scale_accel_range(void);

  //   void set_sleep_enabled(bool enabled);
  //   bool get_sleep_enabled(void);

  //   void set_temp_sensor_enabled(bool enabled);
  //   bool get_temp_sensor_enabled(void);

 private:
  MPU6050 mpu;
  uint8_t dev_addr;
  // FIFO storage buffer
  uint8_t fifo_buffer[64];
  TwoWire *_wire;
  bool imu_ready;
  uint8_t interrupt_pin;  // expected DMP packet size (default is 42 bytes)
  uint16_t packet_size;
  Quaternion q;  // [w, x, y, z]         quaternion container

  void configure_default_settings();

  //   mpu6050_accel_range _accel_range;
  //   mpu6050_gyro_range _gyro_range;
  //   mpu6050_bandwidth _bandwidth;
  //   mpu6050_high_pass_filter _high_pass_filter;
};

#endif  // MPU6050_H_