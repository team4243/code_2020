#include "Robot.h"

#include "DriveTrain.h"

#define ENABLE_DRIVE_TRAIN (true)

DriveTrain driveTrain;

/******************** ROBOT INIT ********************/
void Robot::RobotInit()
{
    driveTrain.Init();
    // control panel init
    // hang mech init
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