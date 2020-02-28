#include "CustomClasses.h"

#include <iostream>

#include "frc/Timer.h"
#include "frc/SerialPort.h"
#include "frc/smartdashboard/SmartDashboard.h"

int angleMeasurement;

frc::SerialPort TeensyPort{115200, frc::SerialPort::kUSB};
frc::Timer TeensyTimer;

void TeensyGyro::Reset()
{
    TeensyPort.Reset();
    // Timer.delay(0.020);
    TeensyPort.Write("!!!!");
}

int TeensyGyro::GetAngleMeasurement()
{
    return angleMeasurement;
}

void TeensyGyro::ProcessSerialData()
{
    try
    {
        // Some const properties
        const int max_measurement_size = 10;
        const char start_character = '#';

        // Initialize and null out the buffer
        char buffer[max_measurement_size];
        for (unsigned int x = 0; x < sizeof(buffer); x++)
            buffer[x] = '\0';

        unsigned int index = 0;

        // Wait until buffer has data
        if (TeensyPort.GetBytesReceived() > 0)
        {
            char next_character[1];
            TeensyPort.Read(next_character, 1);

            // Check for the start character
            if (next_character[0] == start_character)
            {
                // Dummy loop to allow buffer time to fill
                long breakoutCount = 0;
                while (TeensyPort.GetBytesReceived() < 3)
                {
                    if (++breakoutCount > 100000)
                        break;
                }

                // Parse out until newline character reached
                while (TeensyPort.GetBytesReceived() > 0)
                {
                    TeensyPort.Read(next_character, 1);

                    if (next_character[0] == '\0')
                        break;

                    if (next_character[0] == '\n')
                        break;

                    if (index >= sizeof(buffer))
                        break;

                    buffer[index++] = next_character[0];
                }

                float f = (float)atof(buffer);

                angleMeasurement = (int)std::floor(f + 0.5);
            }
        }
    }
    catch (const std::exception &e)
    {
        Reset();
    }
}