#pragma once

#ifndef _H_
#define _H_

#include "frc/Joystick.h"
#include "ButtonMap.h"
#include "ctre/Phoenix.h"

#include <vector>
#include "string.h"

#include "CIEColor.h"
#include "ColorMatch.h"
#include "ColorSensorV3.h"

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
  void Init();
  void Drive();

private:
  double max_stator_current = -1000;
  double min_stator_current = 1000;

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

class ControlPanel
{
public:
  void Init();
  void DoTheThing();
};

class ColorSensorInterface
{
public:
  ColorSensorInterface();
  ~ColorSensorInterface();
  std::string GetColorFromSensor(double confidence);
  bool ColorMatchesColorFromFMS();

private:
  std::shared_ptr<rev::ColorSensorV3> colorSensor;

  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
  rev::ColorMatch m_colorMatcher;

  std::string colorFromFMS;
  std::string getColorFromFMS();
};

class TeensyGyro
{
public:
  void Reset();
  void ProcessSerialData();
  int GetAngleMeasurement();
};

class Utility
{
public:
  //Sending report of the current to file
  static int WriteToFile(std::iostream fh, std::string value)
  {
    fh << value << std::endl;
    return 0;
  }
};

#endif