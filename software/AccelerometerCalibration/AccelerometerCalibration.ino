#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <EEPROM.h>

LSM9DS1 imu;

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  digitalWrite(13, HIGH);
  while (!imu.begin);
  digitalWrite(13, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

}
