#include "CustomClasses.h"

#include "ctre/Phoenix.h"
#include "string.h"

#include "frc/smartdashboard/SmartDashboard.h"

/* TUNING VARIABLEs */
#define CONTROL_PANEL_SCALAR (0.5)
#define REQUIRED_CONFIDENCE (0.80)
#define WRITE_TALON_CONFIGURATIONS (true)
#define CONTROLPANEL_TRIGGER_DEADBAND (0.15)

/* TALON SRX CAN DEVICE(S)*/
#define CONTROL_PANEL_DEVICE_NUMBER (2) //do we have a follower?
#define FOLLOW_CTRL_PANEL_DEVICE_NUMBER (58)

/* TALON CONFIGURATION */
#define CONTROLPANEL_PEAK_OUTPUT_FWD (0.5)
#define CONTROLPANEL_PEAK_OUTPUT_REV (-0.5)
#define CONTROLPANEL_PROPORTIONAL_CTRL (0.25)
#define CONTROLPANEL_DERIVATIVE_CTRL (10)
#define CONTROLPANEL_FEED_FWD_CTRL (0.0)
#define CONTROLPANEL_RAMP_TIME (0)
#define CONTROLPANEL_SLOT_IDX (0)

WPI_TalonSRX ControlPanel_Motor{CONTROL_PANEL_DEVICE_NUMBER};
WPI_TalonSRX FollowControlPanel_Motor{FOLLOW_CTRL_PANEL_DEVICE_NUMBER};

ColorSensorInterface colorSensorInterface;

void ControlPanel::Init()
{
    /*This method gives Driver 2 the ability to manually spin the control panel wheel.
    The wheel is turned when the operator presses and holds button B, and it stops turning
    when button B is released.
    
    When X button is pressed, sensor relays the color being sensed back to the Smart
    Dashboard.
    
    No automation implemented yet, but it may well be using these capabilities.
    
    Need a way to count whole revolutions of the control panel in order to automate for
    ROTATION control.*/

//Control Panel Follower TEMP DELETE LATER
 FollowControlPanel_Motor.Follow(ControlPanel_Motor);
 
    if (driver_two.GetRawButton(B_BUTTON))
    {
        ControlPanel_Motor.Set(0.1);
        // bool cpButtonPressed = ControlPanel_Motor.HasAnyFault();
        frc::SmartDashboard::PutNumber("Motor", 0.1);
        // frc::SmartDashboard::PutBoolean("Button Pressed", cpButtonPressed);
        std::cout << "MOTOR....MOVE PLEASE" << std::endl;
    }

    isTurningThrice = false;
    isTurningToColour = false;
    isManual = true;

    if (WRITE_TALON_CONFIGURATIONS)
        writeTalonConfigs();
}

void ControlPanel::Turn()
{
    commandChecks();

    if (isManual)
        manualTurn();
    // else
    // if ()
    //     turnThreeTimes();
    // if ()
    //     turnToColour();
}

void ControlPanel::manualTurn()
{
    double triggerValue = driver_two.GetRawAxis(RIGHT_TRIGGER);
    triggerValue = Utils::DeadBand(triggerValue, CONTROLPANEL_TRIGGER_DEADBAND);

    ControlPanel_Motor.Set(ControlMode::PercentOutput, CONTROL_PANEL_SCALAR * triggerValue);
}

void ControlPanel::turnThreeTimes()
{
}

void ControlPanel::turnToColour()
{
    if (driver_two.GetRawButton(X_BUTTON))
    {
        std::string foundcolor = colorSensorInterface.GetColorFromSensor(0.70);
        frc::SmartDashboard::PutString("Color Sense", foundcolor);
    }
}

void ControlPanel::stopMotor()
{
    ControlPanel_Motor.Set(ControlMode::PercentOutput, 0);
}

void ControlPanel::commandChecks()
{
    if (driver_two.GetRawButton(STOP_BUTTON_1))
        stopMotor();
}

void ControlPanel::writeTalonConfigs()
{
    ControlPanel_Motor.ConfigPeakOutputForward(CONTROLPANEL_PEAK_OUTPUT_FWD);
    ControlPanel_Motor.ConfigPeakOutputReverse(CONTROLPANEL_PEAK_OUTPUT_REV);
    ControlPanel_Motor.ConfigClosedloopRamp(CONTROLPANEL_RAMP_TIME);
    ControlPanel_Motor.Config_kP(CONTROLPANEL_SLOT_IDX, CONTROLPANEL_PROPORTIONAL_CTRL);
    ControlPanel_Motor.Config_kD(CONTROLPANEL_SLOT_IDX, CONTROLPANEL_DERIVATIVE_CTRL);
    ControlPanel_Motor.Config_kF(CONTROLPANEL_SLOT_IDX, CONTROLPANEL_FEED_FWD_CTRL);
}