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
#define CONTROL_PANEL_DEVICE_NUMBER (56) //do we have a follower?

/* TALON CONFIGURATION */
#define CONTROLPANEL_PEAK_OUTPUT_FWD (0.5)
#define CONTROLPANEL_PEAK_OUTPUT_REV (-0.5)
#define CONTROLPANEL_PROPORTIONAL_CTRL (0.25)
#define CONTROLPANEL_DERIVATIVE_CTRL (10)
#define CONTROLPANEL_FEED_FWD_CTRL (0.0)
#define CONTROLPANEL_RAMP_TIME (0)
#define CONTROLPANEL_SLOT_IDX (0)

WPI_TalonSRX ControlPanel_Motor{CONTROL_PANEL_DEVICE_NUMBER};

ColorSensorInterface colorSensorInterface;

void ControlPanel::Init()
{
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

    ControlPanel_Motor.Set(ControlMode::PercentOutput, CONTROL_PANEL_SCALAR * (triggerValue * 100));
}

void ControlPanel::turnThreeTimes()
{
}

void ControlPanel::turnToColour()
{
    if (driver_two.GetRawButton(X_BUTTON))
    {
        std::string foundcolor = colorSensorInterface.GetColorFromSensor(.70);
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