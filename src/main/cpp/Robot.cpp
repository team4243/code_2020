#include "Robot.h"
#include "CustomClasses.h"

#define ENABLE_DRIVE_TRAIN (true)
#define ENABLE_HANG_MECH (false)
#define ENABLE_CONTROL_PANEL (false)
#define ENABLE_DRIVER_CAMERAS (true)

DriveTrain driveTrain;
HangMech hangMech;
ControlPanel controlPanel;
DriverCameras driverCameras;

/******************** ROBOT INIT ********************/
void Robot::RobotInit()
{
    if (ENABLE_DRIVE_TRAIN)
        driveTrain.Init();

    if (ENABLE_HANG_MECH)
        hangMech.Init();

    if (ENABLE_CONTROL_PANEL)
        controlPanel.Init();

      if (ENABLE_DRIVER_CAMERAS)
        driverCameras.Init();
}
/******************** ROBOT PERIODIC ********************/
void Robot::RobotPeriodic()
{
    if (ENABLE_HANG_MECH)
        hangMech.ProcessSensorData();
}

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
    
    if (ENABLE_HANG_MECH)
        hangMech.Hang_PercentOutput(); // hangMech.Hang_Position();

    if (ENABLE_CONTROL_PANEL)
        controlPanel.DoTheThing();    
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif