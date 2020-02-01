#include "DriveTrain.h"

#include "frc/Joystick.h"
#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"

#include "frc/DriverStation.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "AHRS.h"

#include "ButtonMap.h"

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
frc::Joystick joystick{PORT_JOYSTICK_DRIVER_ONE};

/* NAVX OBJECT INSTANTIATION */
AHRS *navX_gyro;

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

void DriveTrain::Init()
{
    navX_gyro = new AHRS(SPI::Port::kMXP);
    navX_gyro->ZeroYaw();

    if (WRITE_TALON_CONFIGURATIONS)
        WriteTalonConfigs();

    // Set followers to follow their perspective leaders
    leftFrontDriveMotor_Follower.Follow(leftFrontDriveMotor_Leader);
    leftRearDriveMotor_Follower.Follow(leftRearDriveMotor_Leader);
    rightFrontDriveMotor_Follower.Follow(rightFrontDriveMotor_Leader);
    rightRearDriveMotor_Follower.Follow(rightRearDriveMotor_Leader);
}

void DriveTrain::Drive()
{
    double joystick_X = DeadBand(joystick.GetRawAxis(Left_Wheel_X)) * SPEED_SCALAR;
    double joystick_Y = DeadBand(-joystick.GetRawAxis(Left_Wheel_Y)) * SPEED_SCALAR;
    double joystick_Z = DeadBand(joystick.GetRawAxis(Right_Wheel_X)) * SPEED_SCALAR;

    //mecanumDrive.DriveCartesian(joystick_X, joystick_Y, joystick_Z);
    mecanumDrive.DriveCartesian(joystick_X, joystick_Y, joystick_Z, (double)navX_gyro->GetYaw());

    SmartDashboard::PutNumber("The Robot Yawn", navX_gyro->GetYaw());

    if (joystick.GetRawButton(A_Button))
        navX_gyro->ZeroYaw();
}

void DriveTrain::WriteTalonConfigs()
{
    leftFrontDriveMotor_Leader.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    leftFrontDriveMotor_Leader.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    leftFrontDriveMotor_Leader.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    leftFrontDriveMotor_Leader.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    leftFrontDriveMotor_Leader.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    leftFrontDriveMotor_Leader.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    leftFrontDriveMotor_Follower.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    leftFrontDriveMotor_Follower.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    leftFrontDriveMotor_Follower.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    leftFrontDriveMotor_Follower.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    leftFrontDriveMotor_Follower.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    leftFrontDriveMotor_Follower.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    leftRearDriveMotor_Leader.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    leftRearDriveMotor_Leader.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    leftRearDriveMotor_Leader.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    leftRearDriveMotor_Leader.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    leftRearDriveMotor_Leader.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    leftRearDriveMotor_Leader.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    leftRearDriveMotor_Follower.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    leftRearDriveMotor_Follower.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    leftRearDriveMotor_Follower.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    leftRearDriveMotor_Follower.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    leftRearDriveMotor_Follower.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    leftRearDriveMotor_Follower.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    rightFrontDriveMotor_Leader.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    rightFrontDriveMotor_Leader.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    rightFrontDriveMotor_Leader.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    rightFrontDriveMotor_Leader.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    rightFrontDriveMotor_Leader.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    rightFrontDriveMotor_Leader.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    rightFrontDriveMotor_Follower.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    rightFrontDriveMotor_Follower.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    rightFrontDriveMotor_Follower.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    rightFrontDriveMotor_Follower.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    rightFrontDriveMotor_Follower.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    rightFrontDriveMotor_Follower.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    rightRearDriveMotor_Leader.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    rightRearDriveMotor_Leader.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    rightRearDriveMotor_Leader.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    rightRearDriveMotor_Leader.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    rightRearDriveMotor_Leader.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    rightRearDriveMotor_Leader.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    rightRearDriveMotor_Follower.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    rightRearDriveMotor_Follower.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    rightRearDriveMotor_Follower.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    rightRearDriveMotor_Follower.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    rightRearDriveMotor_Follower.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    rightRearDriveMotor_Follower.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);
}