#pragma once

#ifndef _H_
#define _H_

#include "frc/Joystick.h"
#include "ButtonMap.h"
#include "ctre/Phoenix.h"

#include <vector>
#include <stdio.h>
#include <string.h>

#include "CIEColor.h"
#include "ColorMatch.h"
#include "ColorSensorV3.h"
#include "cscore_oo.h"

/* COMMAND BUTTON MAPPING */
#define GYRO_ZERO_BUTTON (A_BUTTON)

/* JOYSTICKS CONFIGURATION */
#define PORT_JOYSTICK_DRIVER_ONE (0)
#define PORT_JOYSTICK_DRIVER_TWO (1)

#define INITIAL_VECTOR_SIZE (300)

/* STATIC JOYSTICK INSTANTIATION */
static frc::Joystick drive_train_controller{PORT_JOYSTICK_DRIVER_ONE};
static frc::Joystick payload_controller{PORT_JOYSTICK_DRIVER_TWO};

/****************************************** DRIVE TRAIN ******************************************/
class DriveTrain
{
public:
  bool is_fast = (false);   //High gear for faster Driving 

  void Init();
  void Drive();

private:
  double max_stator_current = -1000;
  double min_stator_current = 1000;

  void writeTalonConfigs();
};

/****************************************** HANG MECH ******************************************/
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

  bool button_pressed = false;
  double position;

  double speedCurrent = 0;
  double errorLast = 0;

  double proportional = 0;
  double derivative = 0;

  void writeTalonConfigs();
  void logStatorCurrents(WPI_TalonSRX *, std::vector<double> *, std::vector<double> *);
  bool spikeDetected();
};

/****************************************** DRIVER CAMERAS ******************************************/
class DriverCameras
{
public:
  cs::UsbCamera camera1;
  cs::UsbCamera camera2;
  cs::VideoSink server;

  void Init();
};
/****************************************** CONTROL PANEL ******************************************/
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
  std::string GetColorFromSensor(double);
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

/****************************************** TEENSY GYRO ******************************************/
class TeensyGyro
{
public:
  void Reset();
  void ProcessSerialData();
  int GetAngleMeasurement();
};

/****************************************** UTILITIES ******************************************/
class Utils
{
public:
  static int WriteToFile(std::iostream, std::string);
  static double DeadBand(double, double);
  static double Constrain(double, double, double);
};

#endif