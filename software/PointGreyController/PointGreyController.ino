#include <Stepper.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <SparkFunLSM9DS1.h>
#include <SD.h>
#include "config.h"

LSM9DS1 imu;
Stepper azimuth(NUM_STEPS, 3, 4, 5, 6);

char dims[] = {'x', 'y', 'z'};

float gyroOffset[3];

float relativeAngles[3] = {0, 0, 0};
int lastTimes[3];
float lastGyros[3];

bool firstLoop = true;
bool updateAzimuth = false;

void setup() {
  // Setup LED and azimuth stepper speed
  pinMode(LED, OUTPUT);
  azimuth.setSpeed(STEPPER_RPM);
  // Set up IMU
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  digitalWrite(LED, HIGH);
  while (!imu.begin()); // keep the light on if the imu can't be found
  digitalWrite(LED, LOW);
  // Setup interrupts
  MsTimer2::set(MOTOR_UPDATE_RATE, updateMotorTrigger);
  MsTimer2::start();
}

void loop() {
  if (firstLoop) {
    for (int i = 0; i < 3; i++) {
      lastGyros[i] = readGyro(dims[i]) / 1000;
      lastTimes[i] = millis();
    }
    firstLoop = false;
  }
  else {
    for (int i = 0; i < 3; i++) {
      float currentGyro = readGyro(dims[i]) / 1000;
      float currentTime = millis();
      relativeAngles[i] = (currentTime - lastTimes[i]) * (currentGyro / 1000 - lastGyros[i]) / 2;
      lastTimes[i] = currentTime;
      lastGyros[i] = currentGyro;
    }

    if (updateAzimuth) {
      // Only correcting in azimuth
      float degreesCorrected = rotateCameraAzimuth(-relativeAngles[2]);
      relativeAngles[2] += degreesCorrected;
      updateAzimuth = false;
    }
  }
}

void updateMotorTrigger() {
  updateAzimuth = true;
}

float readGyro(char dim) {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  switch (dim) {
    case 'x':
      return imu.calcGyro(imu.gx) - gyroOffset[0];
      break;
    case 'y':
      return imu.calcGyro(imu.gy) - gyroOffset[1];
      break;
    case 'z':
      return imu.calcGyro(imu.gz) - gyroOffset[2];
      break;
    default:
      return 0;
      break;
  } 
}

void zeroIMU(int samples) {
  float gyroSum;
  for (int i = 0; i < 3; i++) {
    gyroSum = 0;
    for (int i = 0; i < samples; i++) {
      gyroSum += readGyro(dims[i]);
    }
    gyroOffset[i] = gyroSum / samples;
  }
}

float rotateCameraAzimuth(float deg) {
  int steps = deg / STEP_SIZE * GEAR_RATIO;
  azimuth.step(steps);
  return steps * STEP_SIZE / GEAR_RATIO; // degrees actually rotated
}

