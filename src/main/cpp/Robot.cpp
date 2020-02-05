#include "Robot.h"

#include "CustomClasses.h"

#include "frc/Joystick.h"

#define ENABLE_DRIVE_TRAIN (true)
#define ENABLE_HANG_MECH (false)

DriveTrain driveTrain;
HangMech hangMech;

/******************** ROBOT INIT ********************/
void Robot::RobotInit()
{
    if (ENABLE_DRIVE_TRAIN)
        driveTrain.Init();

    if (ENABLE_HANG_MECH)
        hangMech.Init();
}

/******************** ROBOT PERIODIC ********************/
void Robot::RobotPeriodic() {}

/******************** AUTONOMOUS INIT ********************/
void Robot::AutonomousInit() {}

/******************** AUTONOMOUS PERIODIC ********************/
void Robot::AutonomousPeriodic() {}

/******************** TELEOP INIT ********************/
void Robot::TeleopInit() {}

/******************** TELEOP PERIODIC ********************/
void Robot::TeleopPeriodic()
{
    if (ENABLE_DRIVE_TRAIN)
        driveTrain.Drive();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif