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

#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include <Wire.h>

#include "IMU.h"

class IMUClass {
public:
  IMUClass(TwoWire& wire, int irqPin);
  virtual ~IMUClass();

  int begin();
  void end();

  // Accelerometer
  virtual int readAcceleration(float& x, float& y, float& z); // Results are in G (earth gravity).
  virtual int accelerationAvailable(); // Number of samples in the FIFO.
  virtual float accelerationSampleRate(); // Sampling rate of the sensor.
  
  // Gyroscope
  virtual int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
  virtual int gyroscopeAvailable(); // Number of samples in the FIFO.
  virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.

  // Magnetometer  
  virtual int readMagneticField(float& x, float& y, float& z); // Results are in uT (micro Tesla).
  virtual int magneticFieldAvailable(); // Number of samples in the FIFO.
  virtual float magneticFieldSampleRate(); // Sampling rate of the sensor.

  // Euler angles
  int readEulerAngles(float& heading, float& roll, float& pitch); // Results are in degrees
  int eulerAnglesAvailable(); // Number of samples in the FIFO.
  float eulerAnglesSampleRate(); // Sampling rate of the sensor.

  virtual float readTemperature(); // Result are in degrees Celcius

private:
  int readRegister(uint8_t address);
  int readRegisters(uint8_t address, uint8_t* data, size_t length);
  int writeRegister(uint8_t address, uint8_t value);

private:
  TwoWire* _wire;
  int _irqPin;

  unsigned long _lastAccelerationReadMillis;
  unsigned long _lastGyroscopeReadMillis;
  unsigned long _lastMagneticFieldReadMillis;
  unsigned long _lastEulerAnglesReadMillis;
};

extern IMUClass IMU;

#endif
