#include "Robot.h"

#include "frc/Joystick.h"
#include "frc/AnalogGyro.h"
#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"

/* MOTOR CONTROLLERS CONFIGURATION */
#define CHANNEL_TALON_LF_LEADER (0)
#define CHANNEL_TALON_LR_LEADER (1)
#define CHANNEL_TALON_RF_LEADER (2)
#define CHANNEL_TALON_RR_LEADER (3)

#define CHANNEL_TALON_LF_FOLLOWER (0)
#define CHANNEL_TALON_LR_FOLLOWER (1)
#define CHANNEL_TALON_RF_FOLLOWER (2)
#define CHANNEL_TALON_RR_FOLLOWER (3)

/* JOYSTICKS CONFIGURATION */
#define PORT_JOYSTICK_DRIVE (0)
#define JOYSTICK_DEADBAND (0.10)

/* JOYSTICK BUTTON MAPPING*/
#define BUTTON_CROSS (5)
#define AXIS_R3_X (2)

/* GYRO CONFIGURATION */
#define PORT_GYRO (0)
#define GRO_KV_PER_DEGREE_SECOND (0.0128)

/* MOTOR CONTROLLER OBJECT INSTANTIATION */
WPI_TalonSRX leftFrontDriveMotor_Leader{CHANNEL_TALON_LF_LEADER};
WPI_TalonSRX leftRearDriveMotor_Leader{CHANNEL_TALON_LR_LEADER};
WPI_TalonSRX rightFrontDriveMotor_Leader{CHANNEL_TALON_RF_LEADER};
WPI_TalonSRX rightRearDriveMotor_Leader{CHANNEL_TALON_RR_LEADER};

WPI_TalonSRX leftFrontDriveMotor_Follower{CHANNEL_TALON_LF_FOLLOWER};
WPI_TalonSRX leftRearDriveMotor_Follower{CHANNEL_TALON_LR_FOLLOWER};
WPI_TalonSRX rightFrontDriveMotor_Follower{CHANNEL_TALON_RF_FOLLOWER};
WPI_TalonSRX rightRearDriveMotor_Follower{CHANNEL_TALON_RR_FOLLOWER};

/* MECANUM DRIVE OBJECT INSTANTIATION */
frc::MecanumDrive mecanumDrive{leftFrontDriveMotor_Leader,
                               leftRearDriveMotor_Leader,
                               rightFrontDriveMotor_Leader,
                               rightRearDriveMotor_Leader};

/* JOYSTICK OBJECTS INSTANTIATION */
frc::Joystick joystick{PORT_JOYSTICK_DRIVE};

/* ANALOG GYRO OBJECT INSTANTIATION */
frc::AnalogGyro gyro{PORT_GYRO};

/* DEADBAND FUNCTION */
/* Takes joystick axis (-1 to 1) and returns 0 if within the deadband */
double DeadBand(double axisValue)
{
    if (axisValue < -JOYSTICK_DEADBAND)
        return axisValue;
    else if (axisValue > JOYSTICK_DEADBAND)
        return axisValue;
    else
        return 0;
}

/******************** ROBOT INIT ********************/
void Robot::RobotInit()
{
    // Set inverted for all LEFT side motor controllers
    leftFrontDriveMotor_Leader.SetInverted(true);
    leftRearDriveMotor_Leader.SetInverted(true);

    leftFrontDriveMotor_Follower.SetInverted(true);
    leftRearDriveMotor_Follower.SetInverted(true);

    // Set followers to follow their perspective leaders
    leftFrontDriveMotor_Follower.Follow(leftFrontDriveMotor_Leader);
    leftRearDriveMotor_Follower.Follow(leftRearDriveMotor_Leader);
    rightFrontDriveMotor_Follower.Follow(rightFrontDriveMotor_Leader);
    rightRearDriveMotor_Follower.Follow(rightRearDriveMotor_Leader);

    // Set the GYRO sensitivity in kV/(deg*s)
    gyro.SetSensitivity(GRO_KV_PER_DEGREE_SECOND);
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
    if (joystick.GetRawButton(BUTTON_CROSS))
        gyro.Reset();

    double joystick_X = DeadBand(joystick.GetX());
    double joystick_Y = DeadBand(-joystick.GetY());
    double joystick_Z = DeadBand(joystick.GetRawAxis(AXIS_R3_X));
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