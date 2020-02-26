#pragma once

#ifndef _H_
#define _H_

#include "frc/Joystick.h"
#include "ButtonMap.h"
#include "ctre/Phoenix.h"
#include "frc/DigitalInput.h"
#include "frc/AnalogInput.h"
#include "frc/SerialPort.h"

// #include <vector>
// #include <stdio.h>
// #include <string.h>

// #include "CIEColor.h"
// #include "ColorMatch.h"
// #include "ColorSensorV3.h"

#include "cscore_oo.h"

/* COMMAND BUTTON MAPPING -- DRIVE TRAIN */
#define DRIVE_GYRO_ZERO_BUTTON (RIGHT_BUMPER)
#define TOGGLE_SLOW_SPEED_BUTTON (RIGHT_TRIGGER)
#define TOGGLE_FIELD_MODE_BUTTON (LEFT_BUMPER)
#define JOYSTICK_X_AXIS (LEFT_WHEEL_X)
#define JOYSTICK_Y_AXIS (LEFT_WHEEL_Y)
#define JOYSTICK_Z_AXIS (RIGHT_WHEEL_X)

/* SHARED BUTTON MAPPING (FOR DRIVER TWO) */
#define TOGGLE_AUTO_MODE_BUTTON (LEFT_BUMPER)
#define STOP_BUTTON_1 (LEFT_BUMPER)
#define STOP_BUTTON_2 (B_BUTTON)

/* COMMAND BUTTON MAPPING -- HANG MECH */
#define TOGGLE_HANG_MECH_AUTO (START_BUTTON)
#define MANUAL_HANG_LEFT_AXIS (LEFT_WHEEL_Y)
#define MANUAL_HANG_RIGHT_AXIS (RIGHT_WHEEL_Y)
#define HANG_GYRO_ZERO_BUTTON_1 (RIGHT_BUMPER)
#define HANG_GYRO_ZERO_BUTTON_2 (Y_BUTTON)

/* COMMAND BUTTON MAPPING -- CONTROL PANEL */
#define TOGGLE_CONTROL_PANEL_AUTO (BACK_BUTTON)
#define TOGGLE_SPIN_THRICE (X_BUTTON)
#define TOGGLE_SPIN_TO_COLOUR (A_BUTTON)
#define CONTROL_PANEL_TURN_AXIS (RIGHT_TRIGGER)

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

  void AutoInit();
  void AutoDrive();

  void Stop();

private:
  void allPrints();
  void commandChecks();
  void writeTalonConfigs();

  bool useSlowSpeed = false;
  bool pressedLastFrame_slowSpeed = false;

  bool useFieldMode = false;
  bool pressedLastFrame_fieldMode = false;

  double gyroYaw = 0;

  // Adds counter during AutoPeriodic to make robot drive forward for 3 seconds
  int m_autoCtr = 0;
};

/****************************************** HANG MECH ******************************************/
class HangMech
{
public:
  void Init();
  void Hang();

private:
  void hangPosition();
  void hangPercentOutput();

  void commandChecks();
  void allPrints();
  void writeTalonConfigs();

  bool useAutoHang = false;
  bool pressedLastFrame_autoHang = false;

  double speedCurrent = 0;
  double errorLast = 0;
};

class LiftArm
{
public:
  void Init();
  void UpdateEncoder();
  void UpdateMotorCurrent();

  void ManualHangPosition(double);
  void ManualHangPercentOutput(double);

  void UpdatePosition(double);
  void UpdateSpeed(double);

  WPI_TalonSRX *Lift_Leader;
  WPI_TalonSRX *Lift_Follower;

  frc::AnalogInput *Limit_High;
  frc::DigitalInput *Limit_Low;

  bool max_reached = false;
  bool min_reached = false;

  double current_position = 0;
  double encoder_value = 0;

  double motor_current = 0;
  double min_motor_current = 0;
  double max_motor_current = 0;

private:
  void getLimits();
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
  void Turn();

private:
  // std::string first_colour = "";
  // std::string previous_colour = "";
  // std::string current_mode = "";

  int confidence_count = 0;
  int num_colour_changed = 0;

  bool isManual = true;
  bool pressedLastFrame_isManual = false;

  bool isTurningThrice = false;
  // bool pressedLastFrame_isTurningThrice = false;

  bool isTurningToColour = false;
  // bool pressedLastFrame_isTurningToColour = false;

  void stopMotor();
  void manualTurn();
  void turnThreeTimes();
  void turnToColour();
  void countTurns();
  void commandChecks();
  void writeTalonConfigs();
};

// class ColorSensorInterface
// {
// public:
//   // ColorSensorInterface();
//   // ~ColorSensorInterface();
//   // std::string GetColorFromSensor(double);
//   // bool ColorMatchesColorFromFMS();

// private:
//   // std::shared_ptr<rev::ColorSensorV3> colorSensor;

//   // static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
//   // static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
//   // static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
//   // static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
//   // rev::ColorMatch m_colorMatcher;

//   // std::string colorFromFMS;
//   // std::string getColorFromFMS();
// };

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
  // static int WriteToFile(std::iostream, std::string);
  static double DeadBand(double, double);
  static double Constrain(double, double, double);
};

#endif