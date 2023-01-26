#include "Gyro.h"
#include <frc/smartdashboard/SmartDashboard.h>

Gyro::Gyro() :
    m_pigeon(2)
{

}

double Gyro::GetHeading()
{
    auto retVal = std::remainder(m_pigeon.GetYaw(), 360.0);
    frc::SmartDashboard::PutNumber("GetAbsoluteCompassHeading", retVal);
    if (retVal > 180.0)
        retVal -= 360.0;
    frc::SmartDashboard::PutNumber("retVal", retVal);

    return retVal;
}

void Gyro::ZeroHeading()
{
    SetHeading(0.0);
}

void Gyro::SetHeading(double heading)
{
int err;

// printf("initial gyro: %f  ", GetHeading());
// printf("heading to set: %f  ", heading);  
for (int n=0; n<100; n++)
    {
    err = m_pigeon.SetYaw(heading, 30);
    if (err)
        printf("SetFusedHeading() failed with error code: %d  ", err); 
    else if(fabs(GetHeading() - heading) < 0.1)  // *** Check needed becuase SetFusedHeading() observed to faile yet still return 0! ***
        {
        // printf("finial gyro: %f\n", GetHeading())
        return;
        }
    }
    printf("************************* FAILED TO SET GYRO HEADING! **********************************");
}

double Gyro::GetTurnRate()
{
    double turnRates [3] = {0, 0, 0};
    m_pigeon.GetRawGyro(turnRates);
    return turnRates[2]; 
}