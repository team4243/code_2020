#include "CustomClasses.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/Phoenix.h"

#include "frc/DigitalInput.h"

/* TUNING VARIABLES */
#define USE_POSITION_CONTROL (false)
#define WRITE_TALON_CONFIGURATIONS (true)
#define AUTO_HANG_SPEED (0)

/* AUTO HANG VARIABLES */
#define LOOPS_PER_SECOND (50)
#define HEIGHT_PER_DEGREE (0.2967) // Arm distance (17 inch) * tan(10)

// 0.5 inches height per lead screw revolution * gear ratio
// Gear ratios: 1, 0.8, 0.55
#define HEIGHT_PER_REVOLUTION (0.5 * 1)

/* DIO CHANNEL NUMBERS (0 - 9)*/
#define LEFT_HIGH_DIO_CHANNEL_NUM (3) //not installed
#define LEFT_LOW_DIO_CHANNEL_NUM (0)
#define RIGHT_HIGH_DIO_CHANNEL_NUM (2) // not installed
#define RIGHT_LOW_DIO_CHANNEL_NUM (1)

/* MOTOR DEFINITIONS*/
#define LIFT_LEFT_LEADER (16)
#define LIFT_LEFT_FOLLOWER (25)

// TO TEST JUST ONE: SET DEVICE ID'S SAME

#define LIFT_RIGHT_LEADER (62)
#define LIFT_RIGHT_FOLLOWER (60)

/* TALON CONFIGURATION */
#define HANG_PEAK_OUTPUT_FWD (0.5)
#define HANG_PEAK_OUTPUT_REV (-0.5)
#define HANG_PROPORTIONAL_CTRL (0.25)
#define HANG_DERIVATIVE_CTRL (10)
#define HANG_FEED_FWD_CTRL (0.0)
#define HANG_RAMP_TIME (0)
#define HANG_SLOT_IDX (0)

/* PERCENT OUTPUT PID CONTROL TUNE */
#define PROPORTIONAL_CONTROL (0.01)
#define DERIVATIVE_CONTROL (0.001)
#define FEED_FORWARD_CONTROL (0)
#define SPEED_SCALAR (0.5)

/* LIFT ARM OBJECTS */
LiftArm LeftArm;
LiftArm RightArm;

void HangMech::Init()
{
    useAutoHang = false;
    pressedLastFrame_autoHang = false;

    speedCurrent = 0;
    errorLast = 0;

    LeftArm.Lift_Leader = new WPI_TalonSRX(LIFT_LEFT_LEADER);
    LeftArm.Lift_Follower = new WPI_TalonSRX(LIFT_LEFT_FOLLOWER);
    RightArm.Lift_Leader = new WPI_TalonSRX(LIFT_RIGHT_LEADER);
    RightArm.Lift_Follower = new WPI_TalonSRX(LIFT_RIGHT_FOLLOWER);

    LeftArm.Limit_High = new frc::DigitalInput(LEFT_HIGH_DIO_CHANNEL_NUM);
    LeftArm.Limit_Low = new frc::DigitalInput(LEFT_LOW_DIO_CHANNEL_NUM);
    RightArm.Limit_High = new frc::DigitalInput(RIGHT_HIGH_DIO_CHANNEL_NUM);
    RightArm.Limit_Low = new frc::DigitalInput(RIGHT_LOW_DIO_CHANNEL_NUM);

    LeftArm.Init();
    RightArm.Init();

    if (WRITE_TALON_CONFIGURATIONS)
        writeTalonConfigs();
}

void HangMech::Hang()
{
    // Check for driver commands
    commandChecks();

    // Update sensor readings
    LeftArm.UpdateEncoder();
    RightArm.UpdateEncoder();

    LeftArm.UpdateMotorCurrent();
    RightArm.UpdateMotorCurrent();

    if (useAutoHang) // AUTO
    {
        if (USE_POSITION_CONTROL)
            hangPosition();
        else
            hangPercentOutput();
    }
    else // MANUAL
    {
        if (USE_POSITION_CONTROL)
        {
            LeftArm.ManualHangPosition(driver_two.GetRawAxis(MANUAL_HANG_LEFT_AXIS));
            RightArm.ManualHangPosition(driver_two.GetRawAxis(MANUAL_HANG_RIGHT_AXIS));
        }
        else
        {
            LeftArm.ManualHangPercentOutput(driver_two.GetRawAxis(MANUAL_HANG_LEFT_AXIS));
            RightArm.ManualHangPercentOutput(driver_two.GetRawAxis(MANUAL_HANG_RIGHT_AXIS));
        }
    }

    allPrints();
}

void HangMech::hangPosition()
{
    // Get gyro angle
    double angle = TeensyGyro::GetAngleMeasurement(); // degrees

    // Computation stuff, ya know... math
    double speed = AUTO_HANG_SPEED * angle; // degrees per second

    double changed_position = (speed / LOOPS_PER_SECOND); // degrees per loop

    double degree_per_revolution = HEIGHT_PER_REVOLUTION / HEIGHT_PER_DEGREE;

    changed_position /= degree_per_revolution; // revolutions per loop

    LeftArm.UpdatePosition(changed_position);
    RightArm.UpdatePosition(-changed_position);

    // Print positions
    frc::SmartDashboard::PutNumber("LEFT Position:", LeftArm.current_position);
    frc::SmartDashboard::PutNumber("RIGHT Position:", RightArm.current_position);
}

void HangMech::hangPercentOutput()
{
    // Get the sensor measurement
    double angle = TeensyGyro::GetAngleMeasurement();

    // Determine the error
    double errorCurrent = angle;

    // Determine change in error
    double errorChange = errorLast - errorCurrent;

    // Compute correction
    double speedChange = PROPORTIONAL_CONTROL * errorCurrent + DERIVATIVE_CONTROL * errorChange;

    // Determine new set speed
    double speedNew = speedCurrent + speedChange;

    // Contrain
    speedNew = Utils::Constrain(speedNew, -1, 1);

    speedNew *= SPEED_SCALAR;

    LeftArm.UpdateSpeed(speedNew);
    RightArm.UpdateSpeed(-speedNew);

    errorLast = errorCurrent;
    speedCurrent = speedNew;

    // Print gyro angle
    frc::SmartDashboard::PutNumber("Hang Gyro:", angle);

    // Print custom PID values
    frc::SmartDashboard::PutNumber("Current Speed:", speedCurrent);
}

void HangMech::commandChecks()
{
    bool autoPressed_1 = driver_two.GetRawButton(TOGGLE_AUTO_MODE_BUTTON);
    bool autoPressed_2 = driver_two.GetRawButton(TOGGLE_HANG_MECH_AUTO);

    // Check for HANG MODE toggle and update dashboard
    if (autoPressed_1 && autoPressed_2)
    {
        // Command button debounce
        if (!pressedLastFrame_autoHang)
        {
            pressedLastFrame_autoHang = true;

            // Toggle the mode
            useAutoHang = !useAutoHang;
        }
    }
    else
        pressedLastFrame_autoHang = false;
}

void HangMech::allPrints()
{
    // Print limit switch triggers
    frc::SmartDashboard::PutString("LEFT Limit High:", (LeftArm.max_reached ? "TRIGGERED" : "--"));
    frc::SmartDashboard::PutString("LEFT Limit Low:", (LeftArm.min_reached ? "TRIGGERED" : "--"));

    frc::SmartDashboard::PutString("RIGHT Limit High:", (RightArm.max_reached ? "TRIGGERED" : "--"));
    frc::SmartDashboard::PutString("RIGHT Limit Low:", (RightArm.min_reached ? "TRIGGERED" : "--"));

    // Print encoder values
    frc::SmartDashboard::PutNumber("LEFT Encoder:", LeftArm.encoder_value);
    frc::SmartDashboard::PutNumber("RIGHT Encoder:", RightArm.encoder_value);

    // Print stator current, min & max currents
    frc::SmartDashboard::PutNumber("LEFT Current:", LeftArm.motor_current);
    frc::SmartDashboard::PutNumber("LEFT MIN Current:", LeftArm.min_motor_current);
    frc::SmartDashboard::PutNumber("LEFT MAX Current:", LeftArm.max_motor_current);

    frc::SmartDashboard::PutNumber("RIGHT Current:", RightArm.motor_current);
    frc::SmartDashboard::PutNumber("RIGHT MIN Current:", RightArm.min_motor_current);
    frc::SmartDashboard::PutNumber("RIGHT MAX Current:", RightArm.max_motor_current);

    // Print the mode
    frc::SmartDashboard::PutString("Hang Mode:", (useAutoHang ? "AUTO" : "MANUAL"));
}

void HangMech::writeTalonConfigs()
{
    LeftArm.Lift_Leader->ConfigPeakOutputForward(HANG_PEAK_OUTPUT_FWD);
    LeftArm.Lift_Leader->ConfigPeakOutputReverse(HANG_PEAK_OUTPUT_REV);
    LeftArm.Lift_Leader->ConfigClosedloopRamp(HANG_RAMP_TIME);
    LeftArm.Lift_Leader->Config_kP(HANG_SLOT_IDX, HANG_PROPORTIONAL_CTRL);
    LeftArm.Lift_Leader->Config_kD(HANG_SLOT_IDX, HANG_DERIVATIVE_CTRL);
    LeftArm.Lift_Leader->Config_kF(HANG_SLOT_IDX, HANG_FEED_FWD_CTRL);

    RightArm.Lift_Leader->ConfigPeakOutputForward(HANG_PEAK_OUTPUT_FWD);
    RightArm.Lift_Leader->ConfigPeakOutputReverse(HANG_PEAK_OUTPUT_REV);
    RightArm.Lift_Leader->ConfigClosedloopRamp(HANG_RAMP_TIME);
    RightArm.Lift_Leader->Config_kP(HANG_SLOT_IDX, HANG_PROPORTIONAL_CTRL);
    RightArm.Lift_Leader->Config_kD(HANG_SLOT_IDX, HANG_DERIVATIVE_CTRL);
    RightArm.Lift_Leader->Config_kF(HANG_SLOT_IDX, HANG_FEED_FWD_CTRL);
}