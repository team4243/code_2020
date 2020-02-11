#include "CustomClasses.h"

#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "AHRS.h"

/* TUNING VARIABLES */
#define SPEED_SCALAR (0.25)
#define WRITE_TALON_CONFIGURATIONS (false)

#define USE_FIELD_MODE (false)

/* MOTOR CONTROLLERS CAN DEVICE NUMBERS */
#define CHANNEL_TALON_LF_LEADER (53)
#define CHANNEL_TALON_LR_LEADER (60)
#define CHANNEL_TALON_RF_LEADER (51)
#define CHANNEL_TALON_RR_LEADER (50)

#define CHANNEL_TALON_LF_FOLLOWER (62)
#define CHANNEL_TALON_LR_FOLLOWER (1)
#define CHANNEL_TALON_RF_FOLLOWER (2)
#define CHANNEL_TALON_RR_FOLLOWER (59)

/* JOYSTICKS CONFIGURATION */
#define JOYSTICK_DEADBAND (0.10)

/* TALON SRX CONFIGURATION */
#define MECANUM_DRIVE_PEAK_OUTPUT_FWD (0.35) // Maximum output speed 0->1
#define MECANUM_DRIVE_PEAK_OUTPUT_REV (-0.35)
#define MECANUM_DRIVE_PROPORTIONAL_CTRL (0.01)
#define MECANUM_DRIVE_DERIVATIVE_CTRL (0.001)
#define MECANUM_DRIVE_FEED_FWD_CTRL (0)
#define MECANUM_DRIVE_RAMP_TIME (0) // Seconds to get from neutral to full speed (peak output)
#define MECANUM_DRIVE_SLOT_IDX (0)  // Which motor control profile to save the configuration to, 0 and 1 available

/* MOTOR CONTROLLER OBJECT INSTANTIATION */
WPI_TalonSRX leftFront_Leader{CHANNEL_TALON_LF_LEADER};
WPI_TalonSRX leftRear_Leader{CHANNEL_TALON_LR_LEADER};
WPI_TalonSRX rightFront_Leader{CHANNEL_TALON_RF_LEADER};
WPI_TalonSRX rightRear_Leader{CHANNEL_TALON_RR_LEADER};

WPI_TalonSRX leftFront_Follower{CHANNEL_TALON_LF_FOLLOWER};
WPI_TalonSRX leftRear_Follower{CHANNEL_TALON_LR_FOLLOWER};
WPI_TalonSRX rightFront_Follower{CHANNEL_TALON_RF_FOLLOWER};
WPI_TalonSRX rightRear_Follower{CHANNEL_TALON_RR_FOLLOWER};


/* MECANUM DRIVE OBJECT INSTANTIATION */
frc::MecanumDrive mecanumDrive{leftFront_Leader,
                               leftRear_Leader,
                               rightFront_Leader,
                               rightRear_Leader};

/* NAVX OBJECT INSTANTIATION */
AHRS navX_gyro{SPI::Port::kMXP};

void DriveTrain::Init()
{
    navX_gyro.ZeroYaw();

    if (WRITE_TALON_CONFIGURATIONS)
        WriteTalonConfigs();

    // Set followers to follow their perspective leaders
    leftFront_Follower.Follow(leftFront_Leader);
    leftRear_Follower.Follow(leftRear_Leader);
    rightFront_Follower.Follow(rightFront_Leader);
    rightRear_Follower.Follow(rightRear_Leader);
}

void DriveTrain::Drive()
{
    // Deadband and scale the joystick input axes
    double joystick_X = DeadBand(driver_one.GetRawAxis(LEFT_WHEEL_X)) * SPEED_SCALAR;
    double joystick_Y = DeadBand(-driver_one.GetRawAxis(LEFT_WHEEL_Y)) * SPEED_SCALAR;
    double joystick_Z = DeadBand(driver_one.GetRawAxis(RIGHT_WHEEL_X)) * SPEED_SCALAR;

    double gyroYaw = (double)navX_gyro.GetYaw();
    
    SmartDashboard::PutNumber("The Robot Yawn", gyroYaw);

    // Field mode uses the GYRO YAW as an input
    if (USE_FIELD_MODE)
        mecanumDrive.DriveCartesian(joystick_X, joystick_Y, joystick_Z, gyroYaw);
    else
        mecanumDrive.DriveCartesian(joystick_X, joystick_Y, joystick_Z);

    if (driver_one.GetRawButton(A_BUTTON))
    {
        navX_gyro.ZeroYaw();
        this->min_stator_current = 0;
        this->max_stator_current = 0;
    }

    double statCurrent = leftFront_Leader.GetStatorCurrent();
    if(statCurrent < this->min_stator_current) this->min_stator_current = statCurrent;
    if(statCurrent > this->max_stator_current) this->max_stator_current = statCurrent;
    SmartDashboard::PutNumber("LF Current", statCurrent);
    SmartDashboard::PutNumber("Max Current", this->max_stator_current);
    SmartDashboard::PutNumber("Min Current", this->min_stator_current);
}

void DriveTrain::WriteTalonConfigs()
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

double DriveTrain::DeadBand(double axisValue)
{
    /* Takes joystick axis (-1 to 1) and returns 0 if within the deadband */
    if (axisValue < -JOYSTICK_DEADBAND)
        return axisValue;
    else if (axisValue > JOYSTICK_DEADBAND)
        return axisValue;
    else
        return 0;
}