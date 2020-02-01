#pragma once

#ifndef _H_
#define _H_

/* TUNING VARIABLES */
#define SPEED_SCALAR (0.25)
#define WRITE_TALON_CONFIGURATIONS (false)

/* MOTOR CONTROLLERS CONFIGURATION */
#define CHANNEL_TALON_LF_LEADER (53)
#define CHANNEL_TALON_LR_LEADER (60)
#define CHANNEL_TALON_RF_LEADER (51)
#define CHANNEL_TALON_RR_LEADER (50)

#define CHANNEL_TALON_LF_FOLLOWER (62)
#define CHANNEL_TALON_LR_FOLLOWER (1)
#define CHANNEL_TALON_RF_FOLLOWER (2)
#define CHANNEL_TALON_RR_FOLLOWER (59)

/* JOYSTICKS CONFIGURATION */
#define PORT_JOYSTICK_DRIVER_ONE (0)
#define JOYSTICK_DEADBAND (0.10)

/* TALON SRX CONFIGURATION */
#define MECANUM_DRIVE_PEAK_OUTPUT_FWD (0.35) // Maximum output speed 0->1
#define MECANUM_DRIVE_PEAK_OUTPUT_REV (-0.35)
#define MECANUM_DRIVE_PROPORTIONAL_CTRL (0.01)
#define MECANUM_DRIVE_DERIVATIVE_CTRL (0.001)
#define MECANUM_DRIVE_FEED_FWD_CTRL (0)
#define MECANUM_DRIVE_RAMP_TIME (0) // Seconds to get from neutral to full speed (peak output)
#define MECANUM_DRIVE_SLOT_IDX (0)  // Which motor control profile to save the configuration to, 0 and 1 available

class DriveTrain
{
public:
  void Init();
  void Drive();
  void WriteTalonConfigs();
};

#endif