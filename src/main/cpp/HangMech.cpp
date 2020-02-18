#include "CustomClasses.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/Phoenix.h"

#include "frc/DigitalInput.h"

/* TUNING VARIABLES */
#define WRITE_TALON_CONFIGURATIONS (true)
#define AUTO_HANG_SPEED (0.01)

/* AUTO HANG VARIABLES */
#define LOOPS_PER_SECOND (50)
#define HEIGHT_PER_DEGREE (0.2967) // Arm distance (17 inch) * tan(10)

// 0.5 inches height per lead screw revolution * gear ratio
// Gear ratios: 1, 0.8, 0.55
#define HEIGHT_PER_REVOLUTION (0.5 * 1)

/* DIO CHANNEL NUMBERS (0 - 9)*/
#define LEFT_HIGH_DIO_CHANNEL_NUM (0)
#define LEFT_LOW_DIO_CHANNEL_NUM (1)
#define RIGHT_HIGH_DIO_CHANNEL_NUM (2)
#define RIGHT_LOW_DIO_CHANNEL_NUM (3)

/* MOTOR DEFINITIONS*/
#define RIGHT_PAYLOAD_LIFT_LEADER_DEVICENUMBER (2)
#define RIGHT_PAYLOAD_LIFT_FOLLOWER_DEVICENUMBER (58)

#define LEFT_PAYLOAD_LIFT_LEADER_DEVICENUMBER (0)
#define LEFT_PAYLOAD_LIFT_FOLLOWER_DEVICENUMBER (0)

/* TALON CONFIGURATION */
#define HANG_PEAK_OUTPUT_FWD (0.5)
#define HANG_PEAK_OUTPUT_REV (-0.5)
#define HANG_PROPORTIONAL_CTRL (0.25)
#define HANG_DERIVATIVE_CTRL (10)
#define HANG_FEED_FWD_CTRL (0.0)
#define HANG_RAMP_TIME (0)
#define HANG_SLOT_IDX (0)

/* LIFT ARM OBJECTS */
LiftArm LeftArm;
LiftArm RightArm;

void HangMech::Init()
{
    LeftArm.Lift_Leader = new WPI_TalonSRX(LEFT_PAYLOAD_LIFT_LEADER_DEVICENUMBER);
    LeftArm.Lift_Follower = new WPI_TalonSRX(LEFT_PAYLOAD_LIFT_FOLLOWER_DEVICENUMBER);

    LeftArm.Limit_High = new frc::DigitalInput(LEFT_HIGH_DIO_CHANNEL_NUM);
    LeftArm.Limit_Low = new frc::DigitalInput(LEFT_LOW_DIO_CHANNEL_NUM);

    if (WRITE_TALON_CONFIGURATIONS)
        writeTalonConfigs();

    TeensyGyro::Reset();
}

void HangMech::Hang()
{
    // Check for driver commands
    commandChecks();

    // Update sensor readings
    LeftArm.UpdateEncoder();
    //RightArm.UpdateEncoder();

    LeftArm.UpdateMotorCurrent();
    //RightArm.UpdateMotorCurrent();

    // AUTO
    if (useAutoHang)
    {
        // Get gyro angle
        double angle = TeensyGyro::GetAngleMeasurement(); // degrees

        // Computation stuff, ya know... math
        double speed = AUTO_HANG_SPEED * angle; // degrees per second

        double changed_position = (speed / LOOPS_PER_SECOND); // degrees per loop

        double degree_per_revolution = HEIGHT_PER_REVOLUTION / HEIGHT_PER_DEGREE;

        changed_position /= degree_per_revolution; // revolutions per loop

        LeftArm.UpdatePosition(-changed_position);
        //RightArm.UpdatePosition(changed_position);
    }

    // MANUAL
    else
    {
        LeftArm.ManualHang(-driver_two.GetRawAxis(MANUAL_HANG_LEFT_AXIS));
        //RightArm.ManualHang(-driver_two.GetRawAxis(MANUAL_HANG_RIGHT_AXIS));
    }

    // Print positions
    frc::SmartDashboard::PutNumber("LEFT Position:", LeftArm.current_position);
    //frc::SmartDashboard::PutNumber("RIGHT Position:", RightArm.current_position);

    // Print limit switch triggers
    frc::SmartDashboard::PutString("LEFT Limit High:", (LeftArm.max_reached ? "TRIGGERED" : ""));
    frc::SmartDashboard::PutString("LEFT Limit Low:", (LeftArm.min_reached ? "TRIGGERED" : ""));

    //frc::SmartDashboard::PutString("RIGHT Limit High:", (RightArm.max_reached ? "TRIGGERED" : ""));
    //frc::SmartDashboard::PutString("RIGHT Limit Low:", (RightArm.min_reached ? "TRIGGERED" : ""));

    // Print encoder values
    frc::SmartDashboard::PutNumber("LEFT Encoder:", LeftArm.encoder_value);
    //frc::SmartDashboard::PutNumber("RIGHT Encoder:", RightArm.encoder_value);

    // Print stator current, min & max currents
    frc::SmartDashboard::PutNumber("LEFT Current:", LeftArm.motor_current);
    //frc::SmartDashboard::PutNumber("RIGHT Current:", RightArm.motor_current);
}

void HangMech::commandChecks()
{
    bool autoPressed_1 = driver_two.GetRawButton(TOGGLE_AUTO_MODE_BUTTON_1);
    bool autoPressed_2 = driver_two.GetRawButton(TOGGLE_AUTO_MODE_BUTTON_2);

    // Check for HANG MODE toggle and update dashboard
    if (autoPressed_1 && autoPressed_2)
    {
        // Command button debounce
        if (!pressedLastFrame_autoHang)
        {
            pressedLastFrame_autoHang = true;

            // Toggle the mode
            useAutoHang = !useAutoHang;

            // Print the mode
            if (useAutoHang)
                frc::SmartDashboard::PutString("Hang Mode:", "AUTO");
            else
                frc::SmartDashboard::PutString("Hang Mode:", "MANUAL");
        }
    }
    else
        pressedLastFrame_autoHang = false;
}

void HangMech::writeTalonConfigs()
{
    LeftArm.Lift_Leader->ConfigPeakOutputForward(HANG_PEAK_OUTPUT_FWD);
    LeftArm.Lift_Leader->ConfigPeakOutputReverse(HANG_PEAK_OUTPUT_REV);
    LeftArm.Lift_Leader->ConfigClosedloopRamp(HANG_RAMP_TIME);
    LeftArm.Lift_Leader->Config_kP(HANG_SLOT_IDX, HANG_PROPORTIONAL_CTRL);
    LeftArm.Lift_Leader->Config_kD(HANG_SLOT_IDX, HANG_DERIVATIVE_CTRL);
    LeftArm.Lift_Leader->Config_kF(HANG_SLOT_IDX, HANG_FEED_FWD_CTRL);

    // RightArm.Lift_Leader->ConfigPeakOutputForward(HANG_PEAK_OUTPUT_FWD);
    // RightArm.Lift_Leader->ConfigPeakOutputReverse(HANG_PEAK_OUTPUT_REV);
    // RightArm.Lift_Leader->ConfigClosedloopRamp(HANG_RAMP_TIME);
    // RightArm.Lift_Leader->Config_kP(HANG_SLOT_IDX, HANG_PROPORTIONAL_CTRL);
    // RightArm.Lift_Leader->Config_kD(HANG_SLOT_IDX, HANG_DERIVATIVE_CTRL);
    // RightArm.Lift_Leader->Config_kF(HANG_SLOT_IDX, HANG_FEED_FWD_CTRL);
}