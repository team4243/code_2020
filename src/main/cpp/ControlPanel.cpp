#include "CustomClasses.h"

#include "ctre/Phoenix.h"
#include "string.h"

#include "frc/smartdashboard/SmartDashboard.h"

WPI_TalonSRX ControlPanel_Motor{56};

ColorSensorInterface colorSensorInterface;

void ControlPanel::DoTheThing()
{
    /*This method gives Driver 2 the ability to manually spin the control panel wheel.
    The wheel is turned when the operator presses and holds button B, and it stops turning
    when button B is released.
    
    When X button is pressed, sensor relays the color being sensed back to the Smart
    Dashboard.
    
    No automation implemented yet, but it may well be using these capabilities.
    
    Need a way to count whole revolutions of the control panel in order to automate for
    ROTATION control.*/
    if (driver_two.GetRawButton(B_BUTTON))
    {
        ControlPanel_Motor.Set(0.1);
        // bool cpButtonPressed = ControlPanel_Motor.HasAnyFault();
        frc::SmartDashboard::PutNumber("Motor", 0.1);
        // frc::SmartDashboard::PutBoolean("Button Pressed", cpButtonPressed);
        std::cout << "MOTOR....MOVE PLEASE" << std::endl;
    }

    else
    {
        ControlPanel_Motor.Set(0);
        frc::SmartDashboard::PutNumber("Motor", 0);
    }

    if (driver_two.GetRawButton(X_BUTTON))
    {
        std::string foundcolor = colorSensorInterface.GetColorFromSensor(0.70);
        frc::SmartDashboard::PutString("Color Sense", foundcolor);
    }
}