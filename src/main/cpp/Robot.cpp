#include "Robot.h"
#include "DriveTrain.h"

#include "frc/Joystick.h"

#define ENABLE_DRIVE_TRAIN (true)

/******************** ROBOT INIT ********************/
void Robot::RobotInit()
{
    driver_one = new frc::Joystick(PORT_JOYSTICK_DRIVER_ONE);
    driver_two = new frc::Joystick(PORT_JOYSTICK_DRIVER_TWO);

    if (ENABLE_DRIVE_TRAIN)
        driveTrain.Init(driver_one);

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