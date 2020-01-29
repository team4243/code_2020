#include "DriveTrain.h"

#include "frc/Joystick.h"
//#include "frc/AnalogGyro.h"
#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"

#include "ButtonMap.h"

/* Axes identification */
#define Left_Wheel_X (0)
#define Left_Wheel_Y (1)
#define Left_Trigger (2)
#define Right_Trigger (3)
#define Right_Wheel_X (4)
#define Right_Wheel_Y (5)

/* Button identification */
#define A_Button (1)
#define B_Button (2)
#define X_Button (3)
#define Y_Button (4)

#define Left_Bumper (5)
#define Right_Bumper (6)
#define Back_Button (7)
#define Start_Button (8)
#define Click_Left_Stick (9)
#define Click_Right_Stick (10)

/* POV (Directonal Pad) identification */
#define DPad_Up (0)
#define DPad_UpRight (45)
#define DPad_Right (90)
#define DPad_DownRight (135)
#define DPad_Down (180)
#define DPad_DownLeft (225)
#define DPad_Left (270)
#define DPad_UpLeft (315)

#define OMNI_DRIVE_PEAK_OUTPUT_FWD (0.35) // Maximum output speed 0->1
#define OMNI_DRIVE_PEAK_OUTPUT_REV (-0.35)
#define OMNI_DRIVE_PROPORTIONAL_CTRL (0.01)
#define OMNI_DRIVE_DERIVATIVE_CTRL (0.001)
#define OMNI_DRIVE_FEED_FWD_CTRL (0)
#define OMNI_DRIVE_RAMP_TIME (0) // Seconds to get from neutral to full speed (peak output)
#define OMNI_DRIVE_SLOT_IDX (0)  // Which motor control profile to save the configuration to, 0 and 1 available

#define SPEED_SCALAR (0.5)

/* MOTOR CONTROLLER OBJECT INSTANTIATION */
WPI_TalonSRX leftFrontDriveMotor_Leader{CHANNEL_TALON_LF_LEADER};
WPI_TalonSRX leftRearDriveMotor_Leader{CHANNEL_TALON_LR_LEADER};
WPI_TalonSRX rightFrontDriveMotor_Leader{CHANNEL_TALON_RF_LEADER};
WPI_TalonSRX rightRearDriveMotor_Leader{CHANNEL_TALON_RR_LEADER};

WPI_TalonSRX leftFrontDriveMotor_Follower{CHANNEL_TALON_LF_FOLLOWER};
WPI_TalonSRX leftRearDriveMotor_Follower{CHANNEL_TALON_LR_FOLLOWER};
WPI_TalonSRX rightFrontDriveMotor_Follower{CHANNEL_TALON_RF_FOLLOWER};
WPI_TalonSRX rightRearDriveMotor_Follower{CHANNEL_TALON_RR_FOLLOWER};

/* MECANUM DRIVE OBJECT INSTANTIATION */
frc::MecanumDrive mecanumDrive{leftFrontDriveMotor_Leader,
                               leftRearDriveMotor_Leader,
                               rightFrontDriveMotor_Leader,
                               rightRearDriveMotor_Leader};

/* JOYSTICK OBJECTS INSTANTIATION */
frc::Joystick joystick{PORT_JOYSTICK_DRIVER_ONE};

/* ANALOG GYRO OBJECT INSTANTIATION */
//frc::AnalogGyro gyro{PORT_GYRO};

/* DEADBAND FUNCTION */
/* Takes joystick axis (-1 to 1) and returns 0 if within the deadband */
double DeadBand(double axisValue)
{
    if (axisValue < -JOYSTICK_DEADBAND)
        return axisValue;
    else if (axisValue > JOYSTICK_DEADBAND)
        return axisValue;
    else
        return 0;
}

void DriveTrain::Init()
{
    // Set inverted for all LEFT side motor controllers
    // leftFrontDriveMotor_Leader.SetInverted(true);
    // leftRearDriveMotor_Leader.SetInverted(true);
    // rightFrontDriveMotor_Leader.SetInverted(true);
    // rightRearDriveMotor_Leader.SetInverted(true);

    // leftFrontDriveMotor_Follower.SetInverted(true);
    // leftRearDriveMotor_Follower.SetInverted(true);
    // rightFrontDriveMotor_Follower.SetInverted(true);
    // rightRearDriveMotor_Follower.SetInverted(true);

    leftFrontDriveMotor_Leader.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
    leftFrontDriveMotor_Leader.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
    leftFrontDriveMotor_Leader.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
    leftFrontDriveMotor_Leader.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
    leftFrontDriveMotor_Leader.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
    leftFrontDriveMotor_Leader.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);

    leftRearDriveMotor_Leader.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
    leftRearDriveMotor_Leader.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
    leftRearDriveMotor_Leader.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
    leftRearDriveMotor_Leader.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
    leftRearDriveMotor_Leader.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
    leftRearDriveMotor_Leader.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);

    rightFrontDriveMotor_Leader.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
    rightFrontDriveMotor_Leader.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
    rightFrontDriveMotor_Leader.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
    rightFrontDriveMotor_Leader.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
    rightFrontDriveMotor_Leader.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
    rightFrontDriveMotor_Leader.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);

    rightRearDriveMotor_Leader.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
    rightRearDriveMotor_Leader.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
    rightRearDriveMotor_Leader.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
    rightRearDriveMotor_Leader.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
    rightRearDriveMotor_Leader.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
    rightRearDriveMotor_Leader.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);


    leftFrontDriveMotor_Follower.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
    leftFrontDriveMotor_Follower.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
    leftFrontDriveMotor_Follower.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
    leftFrontDriveMotor_Follower.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
    leftFrontDriveMotor_Follower.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
    leftFrontDriveMotor_Follower.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);

    leftRearDriveMotor_Follower.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
    leftRearDriveMotor_Follower.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
    leftRearDriveMotor_Follower.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
    leftRearDriveMotor_Follower.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
    leftRearDriveMotor_Follower.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
    leftRearDriveMotor_Follower.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);

    rightFrontDriveMotor_Follower.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
    rightFrontDriveMotor_Follower.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
    rightFrontDriveMotor_Follower.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
    rightFrontDriveMotor_Follower.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
    rightFrontDriveMotor_Follower.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
    rightFrontDriveMotor_Follower.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);

    rightRearDriveMotor_Follower.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
    rightRearDriveMotor_Follower.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
    rightRearDriveMotor_Follower.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
    rightRearDriveMotor_Follower.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
    rightRearDriveMotor_Follower.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
    rightRearDriveMotor_Follower.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);

    // Set followers to follow their perspective leaders
    leftFrontDriveMotor_Follower.Follow(leftFrontDriveMotor_Leader);
    leftRearDriveMotor_Follower.Follow(leftRearDriveMotor_Leader);
    rightFrontDriveMotor_Follower.Follow(rightFrontDriveMotor_Leader);
    rightRearDriveMotor_Follower.Follow(rightRearDriveMotor_Leader);

    // Set the GYRO sensitivity in kV/(deg*s)
    //gyro.SetSensitivity(GRO_KV_PER_DEGREE_SECOND);
}

void DriveTrain::Drive()
{
    // if (joystick.GetRawButton(BUTTON_CROSS) == 1)
    //     gyro.Reset();

    double joystick_X = DeadBand(joystick.GetRawAxis(Left_Wheel_X)) * SPEED_SCALAR;
    double joystick_Y = DeadBand(-joystick.GetRawAxis(Left_Wheel_Y)) * SPEED_SCALAR;
    double joystick_Z = DeadBand(joystick.GetRawAxis(Right_Wheel_X)) * SPEED_SCALAR;
    //double angle = gyro.GetAngle();

    mecanumDrive.DriveCartesian(joystick_X, joystick_Y, joystick_Z);
}