#include "Robot.h"

#include "CustomClasses.h"

#include "frc/Joystick.h"
#include "string.h"
#include "DriveTrain.h"
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ColorSensorInterface.h>


#define ENABLE_DRIVE_TRAIN (true)
#define ENABLE_HANG_MECH (false)

frc::Joystick joystick_TEST{PORT_JOYSTICK_PLAYER_TWO};
WPI_TalonSRX CPMotor{56};

DriveTrain driveTrain;
HangMech hangMech;

ColorSensorInterface colorSensorInterface;

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

    frc::SmartDashboard::PutString("OUT OF DRIVE", "YES");

    if (joystick_TEST.GetRawButton(B_Button))
    {
        CPMotor.Set(.1);
        // bool cpButtonPressed = CPMotor.HasAnyFault();
        frc::SmartDashboard::PutNumber("Motor", 0.1);
        // frc::SmartDashboard::PutBoolean("Button Pressed", cpButtonPressed);
        std::cout << "MOTOR....MOVE PLEASE" << std::endl;
    }

    else
    {
        CPMotor.Set(0);
        frc::SmartDashboard::PutNumber("Motor", 0);
    }

    if (joystick_TEST.GetRawButton(X_Button))
    {
        std::string foundcolor = colorSensorInterface.GetColorFromSensor(.70);
        frc::SmartDashboard::PutString("Color Sense", foundcolor);
    }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif