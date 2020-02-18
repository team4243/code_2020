#include "CustomClasses.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/Phoenix.h"

#include "frc/DigitalInput.h"

/* TUNING VARIABLES */
#define HANG_JOYSTICK_DEADBAND (0.15)
#define MANUAL_HANG_SPEED (0.25)

#define COUNTS_PER_REVOLUTION (4096)
#define MAX_ENCODER_VALUE (123456789)

void LiftArm::Init()
{
    // Configure encoders
    Lift_Leader->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    Lift_Leader->SetSensorPhase(false);

    Lift_Leader->SetSelectedSensorPosition(0, 0, 10);
    Lift_Leader->Set(ControlMode::Position, 0);

    // Set follower motor controller
    Lift_Follower->Follow(*Lift_Leader);
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

void LiftArm::ManualHang(double joystick)
{
    // Deadband
    joystick = Utils::DeadBand(joystick, HANG_JOYSTICK_DEADBAND);

    // Check for direction limit
    if ((max_reached && joystick > 0) || (min_reached && joystick < 0))
        joystick = 0;

    // Scale
    joystick *= MANUAL_HANG_SPEED;

    // Update position
    UpdatePosition(joystick * COUNTS_PER_REVOLUTION);
}

void LiftArm::UpdatePosition(double positionChange)
{
    // Get the limit switch readings
    max_reached = Limit_High->Get();
    min_reached = Limit_Low->Get();

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