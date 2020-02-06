#include "CustomClasses.h"

#include "ctre/Phoenix.h"
#include "frc/Talon.h"

#include "frc/smartdashboard/SmartDashboard.h"

/* PID CONTROL */
#define PROPORTIONAL_CONTROL (0.0022)
#define DERIVATIVE_CONTROL (0.012)
#define ANGLE_DESIRED (0)
#define SPEED_SCALAR (0.5)
#define USE_JOYSTICK (false)

/* MOTOR DEFINITIONS*/
#define PAYLOAD_LIFT_LEADER_DEVICENUMBER (53)
#define PAYLOAD_LIFT_FOLLOWER_DEVICENUMBER (54)

#define MANUAL_INCREMENT (100)
#define WRITE_TALON_CONFIGURATIONS (true)

/* TALON CONFIGURATION */
#define TALON_PEAK_OUTPUT_FWD (0.5)
#define TALON_PEAK_OUTPUT_REV (-0.5)
#define TALON_PROPORTIONAL_CTRL (0.25)
#define TALON_DERIVATIVE_CTRL (10)
#define TALON_FEED_FWD_CTRL (0.0)
#define TALON_RAMP_TIME (0)
#define TALON_SLOT_IDX (0)

WPI_TalonSRX Payload_Lift_Leader{PAYLOAD_LIFT_LEADER_DEVICENUMBER};
WPI_TalonSRX Payload_Lift_Follower{PAYLOAD_LIFT_LEADER_DEVICENUMBER};

TeensyGyro teensyGyro;

void HangMech::Init()
{
    ResetSensor();

    if (WRITE_TALON_CONFIGURATIONS)
        WriteTalonConfigs();

    Payload_Lift_Follower.Follow(Payload_Lift_Leader);
}

void HangMech::Hang_PercentOutput()
{
    // was used for trying to change control variables while Robot is running, is currently not enabled
    if (USE_JOYSTICK)
    {
        if (driver_one.GetRawButton(Y_BUTTON))
            proportional += 0.0001;
        if (driver_one.GetRawButton(X_BUTTON))
            proportional -= 0.0001;
        if (driver_one.GetRawButton(B_BUTTON))
            derivative += 0.0001;
        if (driver_one.GetRawButton(A_BUTTON))
            derivative -= 0.0001;
    }
    else 
    {
        proportional = PROPORTIONAL_CONTROL;
        derivative = DERIVATIVE_CONTROL;
    }

    double angleActual = teensyGyro.GetAngleMeasurement();

    double errorCurrent = ANGLE_DESIRED - angleActual;
    double errorChange = errorLast - errorCurrent;

    double speedChanged = PROPORTIONAL_CONTROL * errorCurrent + DERIVATIVE_CONTROL * errorChange;
    double speedNew = speedCurrent + speedChanged;

    if (speedNew > 1)
        speedNew = 1;
    else if (speedNew < -1)
        speedNew = -1;

    speedNew *= SPEED_SCALAR;

    Payload_Lift_Leader.Set(ControlMode::PercentOutput, speedNew);

    errorLast = errorCurrent;
    speedCurrent = speedNew;

    frc::SmartDashboard::PutNumber("Angle:", angleActual);
    frc::SmartDashboard::PutNumber("Speed:", speedCurrent);
    frc::SmartDashboard::PutNumber("Proportional:", proportional);
    frc::SmartDashboard::PutNumber("Derivative:", derivative);
}

void HangMech::Hang_Position()
{
    position = 0;

    Payload_Lift_Leader.Set(ControlMode::Position, position);

    double encoder = Payload_Lift_Leader.GetSensorCollection().GetQuadraturePosition();

    frc::SmartDashboard::PutNumber("Encoder:", encoder);
    frc::SmartDashboard::PutNumber("Position:", position);
}

void HangMech::ResetSensor()
{
    teensyGyro.Reset();
}

void HangMech::ProcessSensorData()
{
    teensyGyro.ProcessSerialData();
}

void HangMech::WriteTalonConfigs()
{
    Payload_Lift_Leader.ConfigPeakOutputForward(TALON_PEAK_OUTPUT_FWD);
    Payload_Lift_Leader.ConfigPeakOutputReverse(TALON_PEAK_OUTPUT_REV);
    Payload_Lift_Leader.ConfigClosedloopRamp(TALON_RAMP_TIME);
    Payload_Lift_Leader.Config_kP(TALON_SLOT_IDX, TALON_PROPORTIONAL_CTRL);
    Payload_Lift_Leader.Config_kD(TALON_SLOT_IDX, TALON_DERIVATIVE_CTRL);
    Payload_Lift_Leader.Config_kF(TALON_SLOT_IDX, TALON_FEED_FWD_CTRL);
}