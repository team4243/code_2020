#include "CustomClasses.h"

#include "ButtonMap.h"

#define PORT_JOYSTICK_DRIVER_ONE (0)
#define PORT_JOYSTICK_DRIVER_TWO (1)

frc::Joystick driver_one{PORT_JOYSTICK_DRIVER_ONE};
frc::Joystick driver_two{PORT_JOYSTICK_DRIVER_TWO};

double Commands::GetDrive_ForwardReverse()
{
    return driver_one.GetRawAxis(LEFT_WHEEL_X);
}

double Commands::GetDrive_Strafe()
{
    return driver_one.GetRawAxis(LEFT_WHEEL_Y);
}

double Commands::GetDrive_Rotate()
{
    return driver_one.GetRawAxis(RIGHT_WHEEL_X);
}

bool Commands::GetDrive_ZeroNavX()
{
    return driver_one.GetRawButton(A_BUTTON);
}