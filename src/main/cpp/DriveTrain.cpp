#include "CustomClasses.h"

#include "frc/drive/MecanumDrive.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include "ctre/Phoenix.h"
#include "AHRS.h"

/* TUNING VARIABLES */
#define SPEED_SCALAR (0.25)
#define WRITE_TALON_CONFIGURATIONS (false)
#define USE_FIELD_MODE (false)
#define DRIVE_JOYSTICK_DEADBAND (0.10)

/* MOTOR CONTROLLERS CAN DEVICE NUMBERS */
#define CHANNEL_TALON_LF_LEADER (53)
#define CHANNEL_TALON_LR_LEADER (60)
#define CHANNEL_TALON_RF_LEADER (51)
#define CHANNEL_TALON_RR_LEADER (50)

#define CHANNEL_TALON_LF_FOLLOWER (62)
#define CHANNEL_TALON_LR_FOLLOWER (1)
#define CHANNEL_TALON_RF_FOLLOWER (2)
#define CHANNEL_TALON_RR_FOLLOWER (59)

/* TALON SRX CONFIGURATION */
#define MECANUM_DRIVE_PEAK_OUTPUT_FWD (0.35)   // (0 --> 1)
#define MECANUM_DRIVE_PEAK_OUTPUT_REV (-0.35)  // (-1 --> 0)
#define MECANUM_DRIVE_PROPORTIONAL_CTRL (0.01) // PID "P Gain"
#define MECANUM_DRIVE_DERIVATIVE_CTRL (0.001)  // PID "D Gain"
#define MECANUM_DRIVE_FEED_FWD_CTRL (0)        // PID "F Gain"
#define MECANUM_DRIVE_RAMP_TIME (0)            // Seconds for (0 --> 1) Speed
#define MECANUM_DRIVE_SLOT_IDX (0)             // Control profile (0 or 1)

/* MOTOR CONTROLLER INSTANTIATION */
WPI_TalonSRX leftFront_Leader{CHANNEL_TALON_LF_LEADER};
WPI_TalonSRX leftRear_Leader{CHANNEL_TALON_LR_LEADER};
WPI_TalonSRX rightFront_Leader{CHANNEL_TALON_RF_LEADER};
WPI_TalonSRX rightRear_Leader{CHANNEL_TALON_RR_LEADER};

WPI_TalonSRX leftFront_Follower{CHANNEL_TALON_LF_FOLLOWER};
WPI_TalonSRX leftRear_Follower{CHANNEL_TALON_LR_FOLLOWER};
WPI_TalonSRX rightFront_Follower{CHANNEL_TALON_RF_FOLLOWER};
WPI_TalonSRX rightRear_Follower{CHANNEL_TALON_RR_FOLLOWER};

/* MECANUM DRIVE INSTANTIATION */
frc::MecanumDrive mecanumDrive{leftFront_Leader, leftRear_Leader, rightFront_Leader, rightRear_Leader};

/* NAVX INSTANTIATION */
AHRS navX_gyro{SPI::Port::kMXP};

void DriveTrain::Init()
{
    navX_gyro.ZeroYaw();

    if (WRITE_TALON_CONFIGURATIONS)
        writeTalonConfigs();

    // Set followers to follow their perspective leaders
    leftFront_Follower.Follow(leftFront_Leader);
    leftRear_Follower.Follow(leftRear_Leader);
    rightFront_Follower.Follow(rightFront_Leader);
    rightRear_Follower.Follow(rightRear_Leader);
}

void DriveTrain::Drive()
{
    // Get joystick values
    double joystick_X = driver_one.GetRawAxis(LEFT_WHEEL_X);
    double joystick_Y = -driver_one.GetRawAxis(LEFT_WHEEL_Y);
    double joystick_Z = driver_one.GetRawAxis(RIGHT_WHEEL_X);

    // Deadband
    joystick_X = Utils::DeadBand(joystick_X, DRIVE_JOYSTICK_DEADBAND);
    joystick_Y = Utils::DeadBand(joystick_Y, DRIVE_JOYSTICK_DEADBAND);
    joystick_Z = Utils::DeadBand(joystick_Z, DRIVE_JOYSTICK_DEADBAND);

    // Scale
    joystick_X *= SPEED_SCALAR;
    joystick_Y *= SPEED_SCALAR;
    joystick_Z *= SPEED_SCALAR;

    // Constrain
    joystick_X = Utils::Constrain(joystick_X, -1, 1);
    joystick_Y = Utils::Constrain(joystick_Y, -1, 1);
    joystick_Z = Utils::Constrain(joystick_Z, -1, 1);

    // Get sensor measurement
    double gyroYaw = (double)navX_gyro.GetYaw();

    SmartDashboard::PutNumber("The Robot Yawn", gyroYaw);

    // Field mode uses the GYRO YAW as an input
    if (USE_FIELD_MODE)
        mecanumDrive.DriveCartesian(joystick_X, joystick_Y, joystick_Z, gyroYaw);
    else
        mecanumDrive.DriveCartesian(joystick_X, joystick_Y, joystick_Z);

    if (driver_one.GetRawButton(GYRO_ZERO_BUTTON))
    {
        navX_gyro.ZeroYaw();
        this->min_stator_current = 0;
        this->max_stator_current = 0;
    }

    double statCurrent = leftFront_Leader.GetStatorCurrent();

    if (statCurrent < this->min_stator_current)
        this->min_stator_current = statCurrent;

    if (statCurrent > this->max_stator_current)
        this->max_stator_current = statCurrent;

    SmartDashboard::PutNumber("LF Current", statCurrent);
    SmartDashboard::PutNumber("Max Current", this->max_stator_current);
    SmartDashboard::PutNumber("Min Current", this->min_stator_current);
}

void DriveTrain::writeTalonConfigs()
{
    leftFront_Leader.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    leftFront_Leader.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    leftFront_Leader.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    leftFront_Leader.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    leftFront_Leader.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    leftFront_Leader.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    leftRear_Leader.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    leftRear_Leader.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    leftRear_Leader.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    leftRear_Leader.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    leftRear_Leader.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    leftRear_Leader.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    rightFront_Leader.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    rightFront_Leader.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    rightFront_Leader.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    rightFront_Leader.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    rightFront_Leader.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    rightFront_Leader.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);

    rightRear_Leader.ConfigPeakOutputForward(MECANUM_DRIVE_PEAK_OUTPUT_FWD);
    rightRear_Leader.ConfigPeakOutputReverse(MECANUM_DRIVE_PEAK_OUTPUT_REV);
    rightRear_Leader.ConfigClosedloopRamp(MECANUM_DRIVE_RAMP_TIME);
    rightRear_Leader.Config_kP(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_PROPORTIONAL_CTRL);
    rightRear_Leader.Config_kD(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_DERIVATIVE_CTRL);
    rightRear_Leader.Config_kF(MECANUM_DRIVE_SLOT_IDX, MECANUM_DRIVE_FEED_FWD_CTRL);
}