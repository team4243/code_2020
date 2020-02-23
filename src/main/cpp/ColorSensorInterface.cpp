// #include "CustomClasses.h"

// #include <frc/DriverStation.h>
// #include <frc/smartdashboard/smartdashboard.h>

// ColorSensorInterface::ColorSensorInterface()
// {
//     static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
//     colorSensor.reset(new rev::ColorSensorV3(i2cPort));

//     m_colorMatcher.AddColorMatch(kBlueTarget);
//     m_colorMatcher.AddColorMatch(kGreenTarget);
//     m_colorMatcher.AddColorMatch(kRedTarget);
//     m_colorMatcher.AddColorMatch(kYellowTarget);
// }

// ColorSensorInterface::~ColorSensorInterface()
// {
// }

// std::string ColorSensorInterface::GetColorFromSensor(double confidence)
// {
//     std::string colorString;

//     frc::Color detectedColor = colorSensor->GetColor();
//     frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

//     if (matchedColor == kBlueTarget)
//     {
//         colorString = "B";
//     }
//     else if (matchedColor == kRedTarget)
//     {
//         colorString = "R";
//     }
//     else if (matchedColor == kGreenTarget)
//     {
//         colorString = "G";
//     }
//     else if (matchedColor == kYellowTarget)
//     {
//         colorString = "Y";
//     }
//     else
//     {
//         colorString = "Unknown";
//     }

//     frc::SmartDashboard::PutString("Detected Color: ", colorString);

//     return colorString;
// }

// std::string ColorSensorInterface::getColorFromFMS()
// {
//     std::string gameData;
//     gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

//     if (gameData.length() > 0)
//     {
//         colorFromFMS = gameData.substr(0, 1); // B, G, R, Y
//         /*
//         switch (gameData[0])
//         {
//             case 'B' :
//             //Blue case code
//             break;
//             case 'G' :
//             //Green case code
//             break;
//             case 'R' :
//             //Red case code
//             break;
//             case 'Y' :
//             //Yellow case code
//             break;
//             default :
//             //This is corrupt data
//             break;
//         }
//         */
//     }
//     else
//     {
//         printf("\nNo color received from FMS");
//         //Code for no data received yet
//     }

//     frc::SmartDashboard::PutString("Reqested Color From FMS: ", colorFromFMS);

//     return colorFromFMS;
// }

// bool ColorSensorInterface::ColorMatchesColorFromFMS()
// {
//     std::string reqestedColor = getColorFromFMS();
//     std::string color = GetColorFromSensor(50.0);

//     if (reqestedColor.compare(color) == 0)
//         return true;
//     else
//         return false;
// }