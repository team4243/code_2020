#include "Robot.h"
#include "CustomClasses.h"

#include "frc/smartdashboard/SmartDashboard.h"

/* SYSTEM ENABLING */
#define ENABLE_DRIVE_TRAIN (false)
#define ENABLE_HANG_MECH (true)
#define ENABLE_CONTROL_PANEL (false)
#define ENABLE_DRIVER_CAMERAS (false)

/* CUSTOM CLASS INSTANTIATION */
DriveTrain driveTrain;
HangMech hangMech;
ControlPanel controlPanel;
DriverCameras driverCameras;

/******************** ROBOT INIT ********************/
void Robot::RobotInit()
{
    if (ENABLE_DRIVER_CAMERAS)
        driverCameras.Init();
}

/******************** ROBOT PERIODIC ********************/
void Robot::RobotPeriodic()
{
    if (ENABLE_HANG_MECH)
    {
        TeensyGyro::ProcessSerialData();

        // Get gyro angle
        double angle = TeensyGyro::GetAngleMeasurement(); // degrees

        // Print gyro angle
        frc::SmartDashboard::PutNumber("Hang Gyro:", angle);
    }
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
        hangMech.Hang();

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