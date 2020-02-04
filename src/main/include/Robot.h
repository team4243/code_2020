#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "DriveTrain.h"
#include "frc/Joystick.h"

#define PORT_JOYSTICK_DRIVER_ONE (0)
#define PORT_JOYSTICK_DRIVER_TWO (1)

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

private:
  frc::Joystick *driver_one;
  frc::Joystick *driver_two;
  DriveTrain driveTrain;
};
