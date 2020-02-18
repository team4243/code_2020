#pragma once

#ifndef _H_
#define _H_

#include "frc/Joystick.h"
#include "ButtonMap.h"
#include "ctre/Phoenix.h"
#include "frc/DigitalInput.h"

#include <vector>
#include <stdio.h>
#include <string.h>

#include "CIEColor.h"
#include "ColorMatch.h"
#include "ColorSensorV3.h"
#include "cscore_oo.h"

/* COMMAND BUTTON MAPPING -- DRIVE TRAIN */
#define DRIVE_GYRO_ZERO_BUTTON (A_BUTTON)
#define TOGGLE_SLOW_SPEED_BUTTON (BACK_BUTTON)
#define TOGGLE_FIELD_MODE_BUTTON (START_BUTTON)

/* COMMAND BUTTON MAPPING -- HANG MECH */
#define TOGGLE_AUTO_MODE_BUTTON_1 (LEFT_BUMPER)
#define TOGGLE_AUTO_MODE_BUTTON_2 (RIGHT_BUMPER)
#define HANG_GYRO_ZERO_BUTTON (A_BUTTON)

/* JOYSTICKS CONFIGURATION */
#define PORT_JOYSTICK_DRIVER_ONE (0)
#define PORT_JOYSTICK_DRIVER_TWO (1)

#define INITIAL_VECTOR_SIZE (300)

/* STATIC JOYSTICK INSTANTIATION */
static frc::Joystick driver_one{PORT_JOYSTICK_DRIVER_ONE};
static frc::Joystick driver_two{PORT_JOYSTICK_DRIVER_TWO};

/****************************************** DRIVE TRAIN ******************************************/
class DriveTrain
{
public:
  void Init();
  void Drive();

private:
  bool useSlowSpeed = false;
  bool useFieldMode = false;
  double gyroYaw = 0;

  void commandChecks();
  void writeTalonConfigs();
};

/****************************************** HANG MECH ******************************************/
class HangMech
{
public:
  void Init();
  void Hang();

private:
  bool useAutoHang = false;

  void commandChecks();
  void writeTalonConfigs();
};

class LiftArm
{
public:
  void Init();
  void UpdateEncoder();
  void UpdateMotorCurrent();
  void ManualHang(double);
  void UpdatePosition(double positionChange);

  WPI_TalonSRX *Lift_Leader;
  WPI_TalonSRX *Lift_Follower;

  frc::DigitalInput *Limit_High;
  frc::DigitalInput *Limit_Low;

  bool max_reached = false;
  bool min_reached = false;

  double current_position = 0;
  double encoder_value = 0;

  double motor_current = 0;
  double min_motor_current = 1000;
  double max_motor_current = -1000;
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
  static void Reset();
  static void ProcessSerialData();
  static int GetAngleMeasurement();
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