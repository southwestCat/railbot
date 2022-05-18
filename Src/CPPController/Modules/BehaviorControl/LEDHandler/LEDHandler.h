#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Tools/Module/Blackboard.h"

class LEDHandlerBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(SystemSensorData);

    USES_REPRESENTATION(RobotInfo);
};

class LEDHandler : public LEDHandlerBase
{
public:
    enum EyeColor
    {
        red,
        green,
        blue,
        white,
        magenta,
        yellow,
        cyan,
        numOfEyeColors
    };

    void update(LEDRequest &l);

private:
    void update();

    void setBatteryLevelInEar(LEDRequest &ledRequest, LEDRequest::LED baseLED);
    void setEyeColor(LEDRequest &ledRequest, bool left, EyeColor col, LEDRequest::LEDState s);
    void setChestButton(LEDRequest &ledRequest);
    void setLeftEye(LEDRequest &ledRequest);
    void setRightEye(LEDRequest &ledRequest);
    void setLeftEar(LEDRequest &ledRequest);
    void setRightEar(LEDRequest &ledRequest);
    void setHead(LEDRequest &ledRequest);

private:
    size_t chargingLED = 0;
    const int chargingLightSlowness = 5;
    const LEDRequest::LED headLEDCircle[LEDRequest::numOfHeadLEDs] =
        {
            LEDRequest::headLedRearLeft2,
            LEDRequest::headLedRearLeft1,
            LEDRequest::headLedRearLeft0,
            LEDRequest::headLedMiddleLeft0,
            LEDRequest::headLedFrontLeft0,
            LEDRequest::headLedFrontLeft1,
            LEDRequest::headLedFrontRight1,
            LEDRequest::headLedFrontRight0,
            LEDRequest::headLedMiddleRight0,
            LEDRequest::headLedRearRight0,
            LEDRequest::headLedRearRight1,
            LEDRequest::headLedRearRight2};
};
