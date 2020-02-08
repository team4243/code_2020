#pragma once

#ifndef _H_
#define _H_

#include "frc/Joystick.h"
#include "ButtonMap.h"
#include <vector>
#include "ctre/Phoenix.h"
#include "frc/Talon.h"

#define PORT_JOYSTICK_DRIVER_ONE (0)
#define PORT_JOYSTICK_DRIVER_TWO (1)

#define INITIAL_VECTOR_SIZE (300)

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
  double max_stator_current = -9999999.0;
  double min_stator_current = 9999999.0;
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
  std::vector<double> leftStatorCurrent = std::vector<double>(INITIAL_VECTOR_SIZE);
  std::vector<double> rightStatorCurrent = std::vector<double>(INITIAL_VECTOR_SIZE);
  std::vector<double> smoothedLeftStatorCurrent = std::vector<double>(INITIAL_VECTOR_SIZE);
  std::vector<double> smoothedRightStatorCurrent = std::vector<double>(INITIAL_VECTOR_SIZE);

  void WriteTalonConfigs();
  bool button_pressed = false;
  double position;

  double speedCurrent = 0;
  double errorLast = 0;

  double proportional = 0;
  double derivative = 0;

  void logStatorCurrents(WPI_TalonSRX &motor, std::vector<double> &rawStator, std::vector<double> &smoothedStator);
  bool spikeDetected();
};

class TeensyGyro
{
public:
  void Reset();
  void ProcessSerialData();
  int GetAngleMeasurement();
};

class Utility
{ //Sending report of the current to file
  public:
  static int WriteToFile(std::iostream fh, std::string value) {fh << value << std::endl};
};

#endif