#include "CustomClasses.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/Phoenix.h"

/* TUNING VARIABLES */
#define WRITE_TALON_CONFIGURATIONS (true)

/* PID CONTROL */
#define PROPORTIONAL_CONTROL (0.0022)
#define DERIVATIVE_CONTROL (0.012)
#define ANGLE_DESIRED (0)
#define SPEED_SCALAR (0.5)
#define USE_JOYSTICK (false)
#define MANUAL_INCREMENT (100)

/* MOTOR DEFINITIONS*/
#define RIGHT_PAYLOAD_LIFT_LEADER_DEVICENUMBER (53)
#define RIGHT_PAYLOAD_LIFT_FOLLOWER_DEVICENUMBER (54)

#define LEFT_PAYLOAD_LIFT_LEADER_DEVICENUMBER (0)
#define LEFT_PAYLOAD_LIFT_FOLLOWER_DEVICENUMBER (0)

/* TALON CONFIGURATION */
#define TALON_PEAK_OUTPUT_FWD (0.5)
#define TALON_PEAK_OUTPUT_REV (-0.5)
#define TALON_PROPORTIONAL_CTRL (0.25)
#define TALON_DERIVATIVE_CTRL (10)
#define TALON_FEED_FWD_CTRL (0.0)
#define TALON_RAMP_TIME (0)
#define TALON_SLOT_IDX (0)

/* MOTOR CONTROLLER INSTANTIATION */
WPI_TalonSRX Right_Payload_Lift_Leader{RIGHT_PAYLOAD_LIFT_LEADER_DEVICENUMBER};
WPI_TalonSRX Right_Payload_Lift_Follower{RIGHT_PAYLOAD_LIFT_FOLLOWER_DEVICENUMBER};

WPI_TalonSRX Left_Payload_Lift_Leader{LEFT_PAYLOAD_LIFT_LEADER_DEVICENUMBER};
WPI_TalonSRX Left_Payload_Lift_Follower{LEFT_PAYLOAD_LIFT_FOLLOWER_DEVICENUMBER};

/* HANG GYRO INSTANTIATION */
TeensyGyro teensyGyro;

void HangMech::Init()
{
    ResetSensor();

    if (WRITE_TALON_CONFIGURATIONS)
        writeTalonConfigs();

    Right_Payload_Lift_Follower.Follow(Right_Payload_Lift_Leader);
}

void HangMech::Hang_PercentOutput()
{
    // PID Tuning
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

    // Get the sensor measurement
    double angleActual = teensyGyro.GetAngleMeasurement();

    // Determine the error
    double errorCurrent = ANGLE_DESIRED - angleActual;

    // Determine change in error
    double errorChange = errorLast - errorCurrent;

    // Compute correction
    double speedChange = PROPORTIONAL_CONTROL * errorCurrent + DERIVATIVE_CONTROL * errorChange;

    // Determine new set speed
    double speedNew = speedCurrent + speedChange;

    // Contrain
    speedNew = Utils::Constrain(speedNew, -1, 1);

    speedNew *= SPEED_SCALAR;

    Right_Payload_Lift_Leader.Set(ControlMode::PercentOutput, speedNew);

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

    Right_Payload_Lift_Leader.Set(ControlMode::Position, position);

    double encoder = Right_Payload_Lift_Leader.GetSensorCollection().GetQuadraturePosition();

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

void HangMech::logStatorCurrents(WPI_TalonSRX *motor, std::vector<double> *rawStator, std::vector<double> *smoothedStator)
{
}

bool HangMech::spikeDetected() { return true; }

void HangMech::writeTalonConfigs()
{
    Right_Payload_Lift_Leader.ConfigPeakOutputForward(TALON_PEAK_OUTPUT_FWD);
    Right_Payload_Lift_Leader.ConfigPeakOutputReverse(TALON_PEAK_OUTPUT_REV);
    Right_Payload_Lift_Leader.ConfigClosedloopRamp(TALON_RAMP_TIME);
    Right_Payload_Lift_Leader.Config_kP(TALON_SLOT_IDX, TALON_PROPORTIONAL_CTRL);
    Right_Payload_Lift_Leader.Config_kD(TALON_SLOT_IDX, TALON_DERIVATIVE_CTRL);
    Right_Payload_Lift_Leader.Config_kF(TALON_SLOT_IDX, TALON_FEED_FWD_CTRL);

    Left_Payload_Lift_Leader.ConfigPeakOutputForward(TALON_PEAK_OUTPUT_FWD);
    Left_Payload_Lift_Leader.ConfigPeakOutputReverse(TALON_PEAK_OUTPUT_REV);
    Left_Payload_Lift_Leader.ConfigClosedloopRamp(TALON_RAMP_TIME);
    Left_Payload_Lift_Leader.Config_kP(TALON_SLOT_IDX, TALON_PROPORTIONAL_CTRL);
    Left_Payload_Lift_Leader.Config_kD(TALON_SLOT_IDX, TALON_DERIVATIVE_CTRL);
    Left_Payload_Lift_Leader.Config_kF(TALON_SLOT_IDX, TALON_FEED_FWD_CTRL);
}