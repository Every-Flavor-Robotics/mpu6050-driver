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
  uint8_t dev_status = mpu.dmpInitialize();
  if (dev_status == 0)
  {
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(dev_status);
    Serial.println(F(")"));

    return false;
  }

  return true;
}

bool SensorGoMPU6050::calibrate()
{
  mpu.CalibrateGyro(6);
  //   TODO re-enable accel calibration
  // mpu.CalibrateAccel(6);

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
  bool ready = imu_ready && mpu.dmpGetCurrentFIFOPacket(fifo_buffer);

  if (ready)
  {
    // read the package
    mpu.dmpGetQuaternion(&q, fifo_buffer);
  }

  return ready;
}

float SensorGoMPU6050::get_pitch()
{
  float sinp = 2 * (q.w * q.y - q.z * q.x);
  return asin(sinp);
}

float SensorGoMPU6050::get_roll()
{
  float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  return atan2(sinr_cosp, cosr_cosp);
}

float SensorGoMPU6050::get_yaw()
{
  float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return atan2(siny_cosp, cosy_cosp);
}