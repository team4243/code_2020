#include "CustomClasses.h"

#include "frc/drive/MecanumDrive.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include "ctre/Phoenix.h"
#include "AHRS.h"

/* TUNING VARIABLES */
#define LOW_SPEED_SCALAR (0.05)
#define HIGH_SPEED_SCALAR (0.25)

#define WRITE_TALON_CONFIGURATIONS (false)
#define DRIVE_JOYSTICK_DEADBAND (0.10)

/*AUTO PERIODIC DRIVE */
#define AUTO_MOVEMENT_DURATION (3.0)
#define LOOP_FREQUENCY (50)
#define AUTO_MOVEMENT_ITERATIONS (AUTO_MOVEMENT_DURATION * LOOP_FREQUENCY)
#define AUTO_MOVE (0.4)
#define AUTO_TWIRL (0.0)

/* MOTOR CONTROLLERS CAN DEVICE NUMBERS */
#define DRIVE_REAR_RIGHT_LEADER (12)
#define DRIVE_REAR_RIGHT_FOLLOWER (59)
#define DRIVE_REAR_LEFT_LEADER (51)
#define DRIVE_REAR_LEFT_FOLLOWER (50)

#define DRIVE_FRONT_RIGHT_LEADER (35)
#define DRIVE_FRONT_RIGHT_FOLLOWER (53)
#define DRIVE_FRONT_LEFT_LEADER (54)
#define DRIVE_FRONT_LEFT_FOLLOWER (56)

/* TALON SRX CONFIGURATION */
#define DRIVE_PEAK_OUTPUT_FWD (0.35)   // (0 --> 1)
#define DRIVE_PEAK_OUTPUT_REV (-0.35)  // (-1 --> 0)
#define DRIVE_PROPORTIONAL_CTRL (0.01) // PID "P Gain"
#define DRIVE_DERIVATIVE_CTRL (0.001)  // PID "D Gain"
#define DRIVE_FEED_FWD_CTRL (0)        // PID "F Gain"
#define DRIVE_RAMP_TIME (0)            // Seconds for (0 --> 1) Speed
#define DRIVE_SLOT_IDX (0)             // Control profile (0 or 1)

/* MOTOR CONTROLLER INSTANTIATION */
WPI_TalonSRX leftFront_Leader{DRIVE_FRONT_LEFT_LEADER};
WPI_TalonSRX leftRear_Leader{DRIVE_REAR_LEFT_LEADER};
WPI_TalonSRX rightFront_Leader{DRIVE_FRONT_RIGHT_LEADER};
WPI_TalonSRX rightRear_Leader{DRIVE_REAR_RIGHT_LEADER};

WPI_TalonSRX leftFront_Follower{DRIVE_FRONT_LEFT_FOLLOWER};
WPI_TalonSRX leftRear_Follower{DRIVE_REAR_LEFT_FOLLOWER};
WPI_TalonSRX rightFront_Follower{DRIVE_FRONT_RIGHT_FOLLOWER};
WPI_TalonSRX rightRear_Follower{DRIVE_REAR_RIGHT_FOLLOWER};

/* MECANUM DRIVE INSTANTIATION */
frc::MecanumDrive mecanumDrive{leftFront_Leader, leftRear_Leader, rightFront_Leader, rightRear_Leader};

/* NAVX GYRO INSTANTIATION */
AHRS navX_gyro{SPI::Port::kMXP};

void DriveTrain::AutoInit() { m_autoCtr = 0; }

void DriveTrain::AutoDrive()
{
    if (++m_autoCtr <= AUTO_MOVEMENT_ITERATIONS)
    {
        mecanumDrive.DriveCartesian(AUTO_MOVE, 0.0, 0.0);
    }
    else
    {
        mecanumDrive.DriveCartesian(0.0, 0.0, AUTO_TWIRL);
    }
}

void DriveTrain::Init()
{
    // Write Talon configuration if SET
    if (WRITE_TALON_CONFIGURATIONS)
        writeTalonConfigs();

    // Initially zero the gyro
    navX_gyro.ZeroYaw();

    // Set follower motor controllers
    leftFront_Follower.Follow(leftFront_Leader);
    leftRear_Follower.Follow(leftRear_Leader);
    rightFront_Follower.Follow(rightFront_Leader);
    rightRear_Follower.Follow(rightRear_Leader);
}

void DriveTrain::Drive()
{
    // Check for driver commands
    commandChecks();

    // Get joystick values
    double joystick_X = driver_one.GetRawAxis(JOYSTICK_X_AXIS);
    double joystick_Y = -driver_one.GetRawAxis(JOYSTICK_Y_AXIS);
    double joystick_Z = driver_one.GetRawAxis(JOYSTICK_Z_AXIS);

    // Deadband (set values within +- deadband to zero)
    joystick_X = Utils::DeadBand(joystick_X, DRIVE_JOYSTICK_DEADBAND);
    joystick_Y = Utils::DeadBand(joystick_Y, DRIVE_JOYSTICK_DEADBAND);
    joystick_Z = Utils::DeadBand(joystick_Z, DRIVE_JOYSTICK_DEADBAND);

    // Scale the input for desired speed
    if (useSlowSpeed)
    {
        joystick_X *= LOW_SPEED_SCALAR;
        joystick_Y *= LOW_SPEED_SCALAR;
        joystick_Z *= LOW_SPEED_SCALAR;
    }
    else
    {
        joystick_X *= HIGH_SPEED_SCALAR;
        joystick_Y *= HIGH_SPEED_SCALAR;
        joystick_Z *= HIGH_SPEED_SCALAR;
    }

    // Constrain the input to be between -1 and 1
    joystick_X = Utils::Constrain(joystick_X, -1, 1);
    joystick_Y = Utils::Constrain(joystick_Y, -1, 1);
    joystick_Z = Utils::Constrain(joystick_Z, -1, 1);

    // Field mode uses the GYRO YAW as an input
    if (useFieldMode)
        mecanumDrive.DriveCartesian(joystick_Y, joystick_X, joystick_Z, gyroYaw);
    else
        mecanumDrive.DriveCartesian(joystick_Y, joystick_X, joystick_Z);
}

void DriveTrain::Stop()
{
    mecanumDrive.DriveCartesian(0.0, 0.0, 0.0);
}

void DriveTrain::commandChecks()
{
    // Check for FIELD MODE toggle and update dashboard
    if (driver_one.GetRawButton(TOGGLE_FIELD_MODE_BUTTON))
    {
        // Command button debounce
        if (!pressedLastFrame_fieldMode)
        {
            pressedLastFrame_fieldMode = true;

            // Toggle the mode
            useFieldMode = !useFieldMode;

            // Print the mode
            if (useFieldMode)
                frc::SmartDashboard::PutString("Drive Mode:", "FIELD MODE");
            else
                frc::SmartDashboard::PutString("Drive Mode:", "BODY MODE");
        }
    }
    else
        pressedLastFrame_fieldMode = false;

    // Check for SLOW SPEED toggle and update dashboard
    if (driver_one.GetRawButton(TOGGLE_SLOW_SPEED_BUTTON))
    {
        // Command button debounce
        if (!pressedLastFrame_slowSpeed)
        {
            pressedLastFrame_slowSpeed = true;

            // Toggle the mode
            useSlowSpeed = !useSlowSpeed;

            // Print the mode
            if (useSlowSpeed)
                frc::SmartDashboard::PutString("Drive Speed:", "SLOW");
            else
                frc::SmartDashboard::PutString("Drive Speed:", "FAST");
        }
    }
    else
        pressedLastFrame_slowSpeed = false;

    // Gyro functions for FIELD MODE
    if (useFieldMode)
    {
        // Check for command to ZERO the GYRO
        if (driver_one.GetRawButton(DRIVE_GYRO_ZERO_BUTTON))
            navX_gyro.ZeroYaw();

        // Get gyro measurement
        gyroYaw = (double)navX_gyro.GetYaw();

        // Print gyro measurement
        frc::SmartDashboard::PutNumber("The Robot Yawn:", gyroYaw);
    }
}

void DriveTrain::writeTalonConfigs()
{
    leftFront_Leader.ConfigPeakOutputForward(DRIVE_PEAK_OUTPUT_FWD);
    leftFront_Leader.ConfigPeakOutputReverse(DRIVE_PEAK_OUTPUT_REV);
    leftFront_Leader.ConfigClosedloopRamp(DRIVE_RAMP_TIME);
    leftFront_Leader.Config_kP(DRIVE_SLOT_IDX, DRIVE_PROPORTIONAL_CTRL);
    leftFront_Leader.Config_kD(DRIVE_SLOT_IDX, DRIVE_DERIVATIVE_CTRL);
    leftFront_Leader.Config_kF(DRIVE_SLOT_IDX, DRIVE_FEED_FWD_CTRL);

    leftRear_Leader.ConfigPeakOutputForward(DRIVE_PEAK_OUTPUT_FWD);
    leftRear_Leader.ConfigPeakOutputReverse(DRIVE_PEAK_OUTPUT_REV);
    leftRear_Leader.ConfigClosedloopRamp(DRIVE_RAMP_TIME);
    leftRear_Leader.Config_kP(DRIVE_SLOT_IDX, DRIVE_PROPORTIONAL_CTRL);
    leftRear_Leader.Config_kD(DRIVE_SLOT_IDX, DRIVE_DERIVATIVE_CTRL);
    leftRear_Leader.Config_kF(DRIVE_SLOT_IDX, DRIVE_FEED_FWD_CTRL);

    rightFront_Leader.ConfigPeakOutputForward(DRIVE_PEAK_OUTPUT_FWD);
    rightFront_Leader.ConfigPeakOutputReverse(DRIVE_PEAK_OUTPUT_REV);
    rightFront_Leader.ConfigClosedloopRamp(DRIVE_RAMP_TIME);
    rightFront_Leader.Config_kP(DRIVE_SLOT_IDX, DRIVE_PROPORTIONAL_CTRL);
    rightFront_Leader.Config_kD(DRIVE_SLOT_IDX, DRIVE_DERIVATIVE_CTRL);
    rightFront_Leader.Config_kF(DRIVE_SLOT_IDX, DRIVE_FEED_FWD_CTRL);

    rightRear_Leader.ConfigPeakOutputForward(DRIVE_PEAK_OUTPUT_FWD);
    rightRear_Leader.ConfigPeakOutputReverse(DRIVE_PEAK_OUTPUT_REV);
    rightRear_Leader.ConfigClosedloopRamp(DRIVE_RAMP_TIME);
    rightRear_Leader.Config_kP(DRIVE_SLOT_IDX, DRIVE_PROPORTIONAL_CTRL);
    rightRear_Leader.Config_kD(DRIVE_SLOT_IDX, DRIVE_DERIVATIVE_CTRL);
    rightRear_Leader.Config_kF(DRIVE_SLOT_IDX, DRIVE_FEED_FWD_CTRL);
}