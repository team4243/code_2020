#include "CustomClasses.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/Phoenix.h"

#include "frc/DigitalInput.h"
#include "frc/AnalogInput.h"

/* TUNING VARIABLES */
#define HANG_JOYSTICK_DEADBAND (0.15)
#define MANUAL_HANG_SPEED (1)

#define COUNTS_PER_REVOLUTION (4096)
#define MAX_ENCODER_VALUE (123456789)

void LiftArm::Init()
{
    current_position = 0;
    motor_current = 0;
    min_motor_current = 0;
    max_motor_current = 0;

    // Configure encoders
    Lift_Leader->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    Lift_Leader->SetSensorPhase(false);
    Lift_Leader->SetSelectedSensorPosition(0, 0, 10);

    // Set follower motor controller
    Lift_Follower->Follow(*Lift_Leader);
}

void LiftArm::ManualHangPosition(double joystickInput)
{
    getLimits();

    // Deadband
    joystickInput = Utils::DeadBand(joystickInput, HANG_JOYSTICK_DEADBAND);

    // Scale
    joystickInput *= MANUAL_HANG_SPEED;

    // Update position
    UpdatePosition(joystickInput * COUNTS_PER_REVOLUTION);
}

void LiftArm::ManualHangPercentOutput(double joystickInput)
{
    getLimits();

    // Deadband
    joystickInput = Utils::DeadBand(joystickInput, HANG_JOYSTICK_DEADBAND);

    // Update position
    UpdateSpeed(joystickInput);
}

void LiftArm::UpdatePosition(double positionChange)
{
    getLimits();

    // Reject new positions ABOVE MAX if reached
    if (!max_reached && positionChange > 0)
    {
        current_position += positionChange;
        Lift_Leader->Set(ControlMode::Position, current_position);
    }

    // Reject new positions BELOW MIN if reached
    else if (!min_reached && positionChange < 0)
    {
        current_position += positionChange;
        Lift_Leader->Set(ControlMode::Position, current_position);
    }

    // Stop!!
    else if (max_reached || min_reached)
        Lift_Leader->Set(ControlMode::PercentOutput, 0);
}

void LiftArm::UpdateSpeed(double newSpeed)
{
    getLimits();

    // robot arms up is neg. and down is pos.
    if (!min_reached && newSpeed > 0)
        Lift_Leader->Set(ControlMode::PercentOutput, newSpeed);

    else if (!max_reached && newSpeed < 0)
        Lift_Leader->Set(ControlMode::PercentOutput, newSpeed);

    else
        Lift_Leader->Set(ControlMode::PercentOutput, 0);
}

void LiftArm::UpdateEncoder()
{
    encoder_value = Lift_Leader->GetSensorCollection().GetQuadraturePosition();
}

void LiftArm::UpdateMotorCurrent()
{
    // Get stator current
    motor_current = Lift_Leader->GetStatorCurrent();

    // Set MIN if lower
    if (motor_current < min_motor_current)
        min_motor_current = motor_current;

    // Set MAX if higher
    if (motor_current > max_motor_current)
        max_motor_current = motor_current;
}

void LiftArm::getLimits()
{
    // Get the limit switch readings
    // int analogRead = Limit_High->GetValue();
    // max_reached = (analogVoltage > 2000);

    double analogVoltage = Limit_High->GetVoltage();
    max_reached = false; //(analogVoltage > 2.5);

    min_reached = Limit_Low->Get();
}