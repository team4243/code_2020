#pragma once

#ifndef _H_
#define _H_

#include "frc/Joystick.h"
#include "ButtonMap.h"

#define PORT_JOYSTICK_DRIVER_ONE (0)
#define PORT_JOYSTICK_DRIVER_TWO (1)

#define GYRO_ZERO_BUTTON (A_BUTTON)

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

private:
  void WriteTalonConfigs();
  bool button_pressed = false;
  double position;

  double speedCurrent = 0;
  double errorLast = 0;

  double proportional = 0;
  double derivative = 0;
};

class TeensyGyro
{
public:
  void Reset();
  void ProcessSerialData();
  int GetAngleMeasurement();
};

#endif