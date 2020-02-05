#pragma once

#include <frc/TimedRobot.h>

<<<<<<< HEAD
class Robot : public frc::TimedRobot
{
public:
=======

class Robot : public frc::TimedRobot {
 public:
>>>>>>> DriveTrain_Dev
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
};