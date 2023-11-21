// SensorGo header
#include "sensorgo_mpu6050.h"

int sign(float x) { return (x > 0) - (x < 0); }

SensorGoMPU6050::SensorGoMPU6050() {}

bool SensorGoMPU6050::begin(TwoWire *wire)
{
  return begin(MPU6050_DEFAULT_ADDRESS, wire);
}

bool SensorGoMPU6050::begin(uint8_t addr, TwoWire *wire)
{
  //   Wire1.begin(38, 47);
  //   Wire1.setClock(400000);
  _wire = wire;
  dev_addr = addr;
  //   Begin I2C
  _wire->begin();
  // Set I2C clock to 400kHz
  _wire->setClock(400000);

  mpu = MPU6050(dev_addr, _wire);

  //  Check if device is connected
  _wire->beginTransmission(dev_addr);
  if (_wire->endTransmission() != 0)
  {
    Serial.println("Unable to find MPU6050 chip");
    return false;
  }

  mpu.initialize();
  //   Set default IMU settings
  configure_default_settings();

  //   Next, configure DMP
  bool dmp_success = mpu.dmpInitialize();

  if (!dmp_success)
  {
    Serial.println("Failed to initialize DMP");
    return false;
  }

  return true;
}

bool SensorGoMPU6050::calibrate()
{
  mpu.CalibrateGyro(6);
  //   TODO re-enable accel calibration
  //   mpu.CalibrateAccel(6);

  Serial.println("Calibration parameters: ");
  mpu.PrintActiveOffsets();

  // Enable DMP
  mpu.setDMPEnabled(true);

  //   Serial.println("")

  delay(2000);
  //   Update sensor fusion gains
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0x20);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);

  // get expected DMP packet size for later comparison
  packet_size = mpu.dmpGetFIFOPacketSize();

  imu_ready = true;

  return true;
}

void SensorGoMPU6050::configure_default_settings()
{
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setSleepEnabled(false);

  //   TODO: Decide what all settings should be
  //   mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  //   mpu.setTempSensorEnabled(true);
}

bool SensorGoMPU6050::data_ready()
{
  return imu_ready && mpu.dmpGetCurrentFIFOPacket(fifo_buffer);
}

float SensorGoMPU6050::get_pitch()
{
  // static variable used for debouncing
  static float pitch;
  Quaternion q;  // [w, x, y, z]         quaternion container

  // read the package
  mpu.dmpGetQuaternion(&q, fifo_buffer);

  // calculate the angle of the robot
  float pitch_new =
      -_PI_2 + atan2(q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
                     2 * (q.y * q.z + q.w * q.x));

  // a bit of debouncing
  if (abs(pitch_new - pitch) > 0.1)
    pitch += sign(pitch_new - pitch) * 0.01;
  else
    pitch = pitch_new;

  return pitch;
}