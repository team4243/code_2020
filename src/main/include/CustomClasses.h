#pragma once

#ifndef _H_
#define _H_

#include "frc/Joystick.h"

class Commands
{
public:
  static double GetDrive_ForwardReverse();
  static double GetDrive_Strafe();
  static double GetDrive_Rotate();
  static bool GetDrive_ZeroNavX();
};

class DriveTrain
{
public:
  void Init();
  void Drive();

private:
  void WriteTalonConfigs();
  double DeadBand(double);
};

class HangMech
{
public:
  void Init();
  void Hang_PercentOutput();
  void Hang_Position();

  void ProcessSensorData();
  void ResetSensor();
};

class TeensyGyro
{
public:
  void Reset();
  void ProcessSerialData();
  int GetAngleMeasurement();
};

#endif