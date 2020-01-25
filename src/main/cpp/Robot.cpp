#include "Robot.h"

#include "frc/Joystick.h"
#include "frc/AnalogGyro.h"
#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"

#define CHANNEL_TALON_LF (0)
#define CHANNEL_TALON_LR (1)
#define CHANNEL_TALON_RF (2)
#define CHANNEL_TALON_RR (3)

#define PORT_JOYSTICK_DRIVE (0)
#define JOYSTICK_DEADBAND (0.10)

#define PORT_GYRO (0)
#define GRO_KV_PER_DEGREE_SECOND (0.0128)

WPI_TalonSRX leftFrontDriveMotor{CHANNEL_TALON_LF};
WPI_TalonSRX leftRearDriveMotor{CHANNEL_TALON_LR};
WPI_TalonSRX rightFrontDriveMotor{CHANNEL_TALON_RF};
WPI_TalonSRX rightRearDriveMotor{CHANNEL_TALON_RR};

frc::MecanumDrive mecanumDrive{leftFrontDriveMotor, leftRearDriveMotor, rightFrontDriveMotor, rightRearDriveMotor};

frc::Joystick joystick{PORT_JOYSTICK_DRIVE};

frc::AnalogGyro gyro{PORT_GYRO};

double DeadBand(double axisValue)
{
    if (axisValue < -JOYSTICK_DEADBAND)
        return axisValue;
    else if (axisValue > JOYSTICK_DEADBAND)
        return axisValue;
    else
        return 0;
}

void Robot::RobotInit()
{
    leftFrontDriveMotor.SetInverted(true);
    leftRearDriveMotor.SetInverted(true);

    gyro.SetSensitivity(GRO_KV_PER_DEGREE_SECOND);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
    double joystick_X = DeadBand(joystick.GetX());
    double joystick_Y = DeadBand(joystick.GetY());
    double joystick_Z = DeadBand(joystick.GetZ());
    double angle = gyro.GetAngle();

    mecanumDrive.DriveCartesian(joystick_X, joystick_Y, joystick_Z, angle);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif