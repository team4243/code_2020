#include "CustomClasses.h"

/* TUNING VARIABLES */
#define PROPORTIONAL_CONTROL (0.0022)
#define DERIVATIVE_CONTROL (0.012)
#define ANGLE_DESIRED (0)
#define SPEED_SCALAR (0.5)
#define USE_JOYSTICK (false)
#define WRITE_TALON_CONFIGURATION (false)

/* MOTOR DEFINITIONS*/
#define PAYLOAD_LIFT_DEVICENUMBER_LEADER (53)
#define PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER (54)

// STOLEN TalonSRX Configuration -- SET Values
#define PAYLOAD_LIFT_PEAK_OUTPUT_FWD (0.0)   // DOWN
#define PAYLOAD_LIFT_PEAK_OUTPUT_REV (-0.35) // UP
#define PAYLOAD_LIFT_PROPORTIONAL_CTRL (0.25)
#define PAYLOAD_LIFT_DERIVATIVE_CTRL (0.025)
#define PAYLOAD_LIFT_FEED_FWD_CTRL (0.0)
#define PAYLOAD_LIFT_RAMP_TIME (0) // Seconds to get from neutral to full speed (peak output)
#define PAYLOAD_LIFT_SLOT_IDX (0)  // Which motor control profile to save the configuration to, 0 and 1 available

TeensyGyro teensyGyro;

void HangMech::Init()
{
    ResetSensor();
}

void HangMech::Hang_PercentOutput()
{
}

void HangMech::Hang_Position()
{
}

void HangMech::ResetSensor() { teensyGyro.Reset(); }

void HangMech::ProcessSensorData() { teensyGyro.ProcessSerialData(); }