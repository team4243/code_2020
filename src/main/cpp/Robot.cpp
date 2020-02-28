#include "Robot.h"
#include "CustomClasses.h"

#include "frc/smartdashboard/SmartDashboard.h"

/* SYSTEM ENABLING */
#define ENABLE_DRIVE_TRAIN (true)
#define ENABLE_HANG_MECH (true)
#define ENABLE_CONTROL_PANEL (true)
#define ENABLE_DRIVER_CAMERAS (true)

/* CUSTOM CLASS INSTANTIATION */
DriveTrain driveTrain;
HangMech hangMech;
ControlPanel controlPanel;
DriverCameras driverCameras;
TeensyGyro teensyGyro;

/******************** ROBOT INIT ********************/
void Robot::RobotInit()
{
    teensyGyro.Reset();

    if (ENABLE_DRIVER_CAMERAS)
        driverCameras.Init();
}

/******************** ROBOT PERIODIC ********************/
void Robot::RobotPeriodic()
{
    if (ENABLE_HANG_MECH)
        teensyGyro.ProcessSerialData();
}

/******************** AUTONOMOUS INIT ********************/
//Initialization is carried out in Robot Init -Programming

void Robot::AutonomousInit()
{
    if (ENABLE_DRIVE_TRAIN)
    {
        driveTrain.AutoInit();
        driveTrain.Stop();
    }

    if (ENABLE_HANG_MECH)
        hangMech.Init();

    if (ENABLE_CONTROL_PANEL)
        controlPanel.Init();
}

/******************** AUTONOMOUS PERIODIC ********************/
//Have Robot drive (x)ft based off of autonomous scoring

void Robot::AutonomousPeriodic()
{
    if (ENABLE_DRIVE_TRAIN)
        driveTrain.AutoDrive();
}

/******************** TELEOP INIT ********************/
//Initialization is carried out in Robot Init -Programming

void Robot::TeleopInit()
{
    teensyGyro.Reset();

    if (ENABLE_DRIVE_TRAIN)
    {
        driveTrain.Init();
        driveTrain.Stop();
    }

    if (ENABLE_HANG_MECH)
        hangMech.Init();

    if (ENABLE_CONTROL_PANEL)
        controlPanel.Init();
}

/******************** TELEOP PERIODIC ********************/
void Robot::TeleopPeriodic()
{
    if (ENABLE_DRIVE_TRAIN)
        driveTrain.Drive();

    if (ENABLE_HANG_MECH)
        hangMech.Hang((double)teensyGyro.GetAngleMeasurement());

    if (ENABLE_CONTROL_PANEL)
        controlPanel.Turn();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif