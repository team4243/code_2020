#pragma once

#ifndef _H_
#define _H_

#include "frc/Joystick.h"

#define PORT_JOYSTICK_DRIVER_ONE (0)
#define PORT_JOYSTICK_DRIVER_TWO (1)

static frc::Joystick driver_one{PORT_JOYSTICK_DRIVER_ONE};
static frc::Joystick driver_two{PORT_JOYSTICK_DRIVER_TWO};
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