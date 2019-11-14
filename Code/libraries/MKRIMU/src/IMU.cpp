/*
  This file is part of the MKRIMU library.
  Copyright (c) 2018 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "IMU.h"

// from Table 3-14 in the datasheet
#define ACCELEROMETER_SAMPLE_RATE     100
#define ACCELEROMETER_PERIOD_MS       (1000 / ACCELEROMETER_SAMPLE_RATE)
#define GYROSCOPE_SAMPLE_RATE         100
#define GYROSCOPE_PERIOD_MS           (1000 / ACCELEROMETER_SAMPLE_RATE)
#define MAGNETOMETER_SAMPLE_RATE      20
#define MAGNETOMETER_PERIOD_MS        (1000 / MAGNETOMETER_SAMPLE_RATE)
#define EULER_ANGLES_SAMPLE_RATE      100
#define EULER_ANGLES_PERIOD_MS        (1000 / EULER_ANGLES_SAMPLE_RATE)

#define BNNO055_ADDRESS                0x28

#define BNNO055_CHIP_ID_REG            0x00
#define BNNO055_ACC_DATA_X_LSB_REG     0x08
#define BNNO055_MAG_DATA_X_LSB_REG     0x0e
#define BNNO055_GYR_DATA_X_LSB_REG     0x14
#define BNNO055_EUL_DATA_X_LSB_REG     0x1a
#define BNNO055_PAGE_ID_REG            0x07
#define BNNO055_TEMP_REG               0x34
#define BNNO055_SYS_STATUS_REG         0x39
#define BNNO055_UNIT_SEL_REG           0x3b
#define BNNO055_OPR_MODE_REG           0x3d
#define BNNO055_SYS_TRIGGER_REG        0x3f
#define BNNO055_AXIS_MAP_CONFIG_REG    0x41
#define BNNO055_AXIS_MAP_SIGN_REG      0x42

IMUClass::IMUClass(TwoWire& wire, int irqPin) : 
  _wire(&wire),
  _irqPin(irqPin)
{
}

IMUClass::~IMUClass()
{
}

int IMUClass::begin()
{
  _lastAccelerationReadMillis  = 0;
  _lastGyroscopeReadMillis     = 0;
  _lastMagneticFieldReadMillis = 0;
  _lastEulerAnglesReadMillis   = 0;

  _wire->begin();

  if (readRegister(BNNO055_CHIP_ID_REG) != 0xa0) {
    end();

    return 0;
  }

  // enter config mode
  writeRegister(BNNO055_OPR_MODE_REG, 0x00);
  delay(19);

  // select page id 0
  writeRegister(BNNO055_PAGE_ID_REG, 0x00);

  // enable external clock
  writeRegister(BNNO055_SYS_TRIGGER_REG, 0x80);

  // set acceleration unit to mG's, and fusion data output mode to Android
  writeRegister(BNNO055_UNIT_SEL_REG, 0x81);

  // set X = X, Y = Y, Z = Z
  writeRegister(BNNO055_AXIS_MAP_CONFIG_REG, 0x24);

  // invert X and Y axis signs
  writeRegister(BNNO055_AXIS_MAP_SIGN_REG, 0x06);

  // enter NDOF mode
  writeRegister(BNNO055_OPR_MODE_REG, 0x0c);
  delay(7);

  // wait for sensor fusion algorithm to be running
  do {
    delay(100);
  } while (readRegister(BNNO055_SYS_STATUS_REG) != 0x05);

  return 1;
}

void IMUClass::end()
{
  // try to set config mode
  writeRegister(BNNO055_OPR_MODE_REG, 0x00);

  _wire->end();
}

int IMUClass::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  _lastAccelerationReadMillis = millis();

  if (!readRegisters(BNNO055_ACC_DATA_X_LSB_REG, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  // convert mg to G's
  x = data[0] / 1000.0;
  y = data[1] / 1000.0;
  z = data[2] / 1000.0;

  return 1;
}

int IMUClass::accelerationAvailable()
{
  unsigned long now = millis();

  if (abs((long)now - (long)_lastAccelerationReadMillis) < ACCELEROMETER_PERIOD_MS) {
    return 0;
  }

  return 1;
}

float IMUClass::accelerationSampleRate()
{
  return ACCELEROMETER_SAMPLE_RATE; // fixed in fusion mode
}

int IMUClass::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  _lastGyroscopeReadMillis = millis();

  if (!readRegisters(BNNO055_GYR_DATA_X_LSB_REG, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  // convert degrees per second
  x = data[0] / 16.0;
  y = data[1] / 16.0;
  z = data[2] / 16.0;

  return 1;
}

int IMUClass::gyroscopeAvailable()
{
  unsigned long now = millis();

  if (abs((long)now - (long)_lastGyroscopeReadMillis) < GYROSCOPE_PERIOD_MS) {
    return 0;
  }

  return 1;
}

float IMUClass::gyroscopeSampleRate()
{
  return GYROSCOPE_SAMPLE_RATE; // fixed in fusion mode
}

int IMUClass::readMagneticField(float& x, float& y, float& z)
{
  int16_t data[3];

  _lastMagneticFieldReadMillis = millis();

  if (!readRegisters(BNNO055_MAG_DATA_X_LSB_REG, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  // convert uT
  x = data[0] / 16.0;
  y = data[1] / 16.0;
  z = data[2] / 16.0;

  return 1;
}

int IMUClass::magneticFieldAvailable()
{
  unsigned long now = millis();

  if (abs((long)now - (long)_lastMagneticFieldReadMillis) < MAGNETOMETER_PERIOD_MS) {
    return 0;
  }

  return 1;
}

float IMUClass::magneticFieldSampleRate()
{
  return MAGNETOMETER_SAMPLE_RATE; // fixed in fusion mode
}

int IMUClass::readEulerAngles(float& heading, float& roll, float& pitch)
{
  int16_t data[3];

  _lastEulerAnglesReadMillis = millis();

  if (!readRegisters(BNNO055_EUL_DATA_X_LSB_REG, (uint8_t*)data, sizeof(data))) {
    heading = NAN;
    roll = NAN;
    pitch = NAN;

    return 0;
  }

  // convert degrees
  heading = data[0] / 16.0;
  roll = data[1] / 16.0;
  pitch = data[2] / 16.0;

  return 1;
}

int IMUClass::eulerAnglesAvailable()
{
  unsigned long now = millis();

  if (abs((long)now - (long)_lastEulerAnglesReadMillis) < EULER_ANGLES_PERIOD_MS) {
    return 0;
  }

  return 1;
}

float IMUClass::eulerAnglesSampleRate()
{
  return EULER_ANGLES_SAMPLE_RATE; // fixed in fusion mode
}

float IMUClass::readTemperature()
{
  int temp = readRegister(BNNO055_TEMP_REG);

  if (temp == -1) {
    return NAN;
  }

  return (int8_t)temp;
}

int IMUClass::readRegister(uint8_t address)
{
  _wire->beginTransmission(BNNO055_ADDRESS);
  _wire->write(address);
  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(BNNO055_ADDRESS, 1) != 1) {
    return -1;
  }

  return _wire->read();
}

int IMUClass::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  _wire->beginTransmission(BNNO055_ADDRESS);
  _wire->write(address);
  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(BNNO055_ADDRESS, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }

  return 1;
}

int IMUClass::writeRegister(uint8_t address, uint8_t value)
{
  _wire->beginTransmission(BNNO055_ADDRESS);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}

IMUClass IMU(Wire, 0);
