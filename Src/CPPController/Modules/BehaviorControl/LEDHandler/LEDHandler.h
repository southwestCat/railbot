#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Communication/RobotInfo.h"
#include "Tools/Module/Blackboard.h"

class LEDHandlerBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);

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

    void setEyeColor(LEDRequest &ledRequest, bool left, EyeColor col, LEDRequest::LEDState s);
    void setChestButton(LEDRequest &ledRequest);
      void setLeftEye(LEDRequest &ledRequest);
    void setRightEye(LEDRequest &ledRequest);

    void update(LEDRequest &l);

private:
    void update();
};
