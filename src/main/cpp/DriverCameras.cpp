#include "CustomClasses.h"
#include <cameraserver/CameraServer.h>

void DriverCameras::Init()
{
    camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    camera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
    server = frc::CameraServer::GetInstance()->GetServer();
    camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    camera2.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    // cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
}