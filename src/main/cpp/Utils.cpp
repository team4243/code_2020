#include "CustomClasses.h"

#include <stdio.h>
#include <string.h>

double Utils::DeadBand(double value, double band)
{
    if (value < -band || value > band)
        return value;
    else
        return 0;
}

double Utils::Constrain(double value, double min, double max)
{
    if (value > max)
        value = max;
    else if (value < min)
        value = min;

    return value;
}

int Utils::WriteToFile(std::iostream fh, std::string value)
{
    //Sending report of the current to file
    fh << value << std::endl;
    return 0;
}