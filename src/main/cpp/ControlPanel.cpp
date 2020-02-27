#include "CustomClasses.h"

#include "ctre/Phoenix.h"

#include "frc/smartdashboard/SmartDashboard.h"

/* TUNING VARIABLEs */
#define CONTROL_PANEL_SCALAR (0.3)
#define REQUIRED_CONFIDENCE (0.80)
#define WRITE_TALON_CONFIGURATIONS (true)
#define CONTROLPANEL_TRIGGER_DEADBAND (0.15)

/* TALON SRX CAN DEVICE(S)*/
#define CONTROL_PANEL_WHEEL (2)

/* TALON CONFIGURATION */
#define CONTROLPANEL_PEAK_OUTPUT_FWD (0.5)
#define CONTROLPANEL_PEAK_OUTPUT_REV (-0.5)
#define CONTROLPANEL_PROPORTIONAL_CTRL (0.25)
#define CONTROLPANEL_DERIVATIVE_CTRL (10)
#define CONTROLPANEL_FEED_FWD_CTRL (0.0)
#define CONTROLPANEL_RAMP_TIME (0)
#define CONTROLPANEL_SLOT_IDX (0)

VictorSPX ControlPanel_Motor{CONTROL_PANEL_WHEEL};

//ColorSensorInterface colorSensorInterface;

rev::ColorSensorV3 colorSensor{frc::I2C::Port::kOnboard};
rev::ColorMatch colorMatcher;

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

    isTurningThrice = false;
    isTurningToColour = false;
    isManual = true;

    colorMatcher.AddColorMatch(blueTarget);
    colorMatcher.AddColorMatch(redTarget);
    colorMatcher.AddColorMatch(greenTarget);
    colorMatcher.AddColorMatch(yellowTarget);

    if (WRITE_TALON_CONFIGURATIONS)
        writeTalonConfigs();
}

void ControlPanel::Turn()
{
    commandChecks();

    countTurns();

    manualTurn();

    // if (isManual)
    // {
    //     manualTurn();
    //     current_mode = "manual";
    // }
    // else if (isTurningThrice)
    // {
    //     turnThreeTimes();
    //     current_mode = "turn3";
    // }
    // if (isTurningToColour)
    // {
    //     turnToColour();
    //     current_mode = "toColour";
    // }

    // frc::SmartDashboard::PutString("Mode:", current_mode);
}

void ControlPanel::manualTurn()
{
    double triggerValue = driver_two.GetRawAxis(CONTROL_PANEL_TURN_AXIS);
    triggerValue = Utils::DeadBand(triggerValue, CONTROLPANEL_TRIGGER_DEADBAND);

    ControlPanel_Motor.Set(ControlMode::PercentOutput, CONTROL_PANEL_SCALAR * triggerValue);
}

void ControlPanel::turnThreeTimes()
{
    if (num_colour_changed < 32)
        ControlPanel_Motor.Set(ControlMode::PercentOutput, 20);
    else
        ControlPanel_Motor.Set(ControlMode::PercentOutput, 0);
}

void ControlPanel::turnToColour()
{
    // if (strcmp(colourString[0], getColorFromFMS()))
    //     ControlPanel_Motor.Set(ControlMode::PercentOutput, 0);
    // else
    //     ControlPanel_Motor.Set(ControlMode::PercentOutput, 20);
}

void ControlPanel::stopMotor()
{
    ControlPanel_Motor.Set(ControlMode::PercentOutput, 0);
}

void ControlPanel::countTurns()
{
    // try this but if it doesn't work oh well
    if (!(colourString.compare(previous_colour)))
    {
        confidence_count++;
        if (confidence_count == 3)
        {
            confidence_count = 0;
            previous_colour = colourString;
            num_colour_changed++;
        }
    }
}

void ControlPanel::commandChecks()
{
    if (driver_two.GetRawButton(STOP_BUTTON_1) && driver_two.GetRawButton(STOP_BUTTON_2))
        stopMotor();

    if (driver_two.GetRawButtonReleased(TOGGLE_SPIN_THRICE))
        isTurningThrice = !isTurningThrice;

    if (driver_two.GetRawButtonReleased(TOGGLE_SPIN_TO_COLOUR))
        isTurningToColour = !isTurningToColour;

    if (driver_two.GetRawButton(TOGGLE_AUTO_MODE_BUTTON) && driver_two.GetRawButton(TOGGLE_CONTROL_PANEL_AUTO))
    {
        //command button debouncing
        if (!pressedLastFrame_isManual)
        {
            pressedLastFrame_isManual = true;

            isManual = !isManual;
        }
        else
            pressedLastFrame_isManual = false;
    }

    if (isManual)
    {
        isTurningThrice = false;
        isTurningToColour = false;
    }
    if (isTurningThrice)
    {
        isManual = false;
        isTurningToColour = false;
    }
    if (isTurningToColour)
    {
        isManual = false;
        isTurningThrice = false;
    }

    // printing stuff
    colour = colorSensor.GetColor();
    double confidence = 0.70;
    colour = colorMatcher.MatchClosestColor(colour, confidence);
    std::string colourString = toColourString(colour);

    frc::SmartDashboard::PutString("Colour", colourString);
    frc::SmartDashboard::PutString("Target C", targetColour);
}

std::string ControlPanel::toColourString(frc::Color colorS)
{
    if (colorS == blueTarget)
        return "Blue";
    if (colorS == redTarget)
        return "Red";
    if (colorS == greenTarget)
        return "Green";
    if (colorS == yellowTarget)
        return "Yellow";
    else
    {
        return "";
    }
    
}

std::string ControlPanel::getColorFromFMS()
{
    std::string gameData;
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

    if (gameData.length() > 0)
    {
        targetColour = gameData.substr(0, 1); // B, G, R, Y
        switch (gameData[0])
        {
            case 'B' :
            //Blue case code
            break;
            case 'G' :
            //Green case code
            break;
            case 'R' :
            //Red case code
            break;
            case 'Y' :
            //Yellow case code
            break;
            default :
            //This is corrupt data
            break;
        }
    }
    else
    {
        printf("\nNo color received from FMS");
        //Code for no data received yet
    }
    return targetColour;
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