#include "CustomClasses.h"

#include "ctre/Phoenix.h"
#include "string.h"

#include "CIEColor.h"
#include "ColorMatch.h"
#include "ColorSensorV3.h"

#include "frc/smartdashboard/SmartDashboard.h"

WPI_TalonSRX ControlPanel_Motor{56};

ColorSensorInterface colorSensorInterface;

void ControlPanel::DoTheThing()
{
    if (driver_two.GetRawButton(B_BUTTON))
    {
        ControlPanel_Motor.Set(.1);
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
        std::string foundcolor = colorSensorInterface.GetColorFromSensor(.70);
        frc::SmartDashboard::PutString("Color Sense", foundcolor);
    }
}