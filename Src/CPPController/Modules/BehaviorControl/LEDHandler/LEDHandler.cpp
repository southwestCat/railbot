#include "LEDHandler.h"
#include "Tools/Module/ModuleManager.h"

#include <iostream>

void LEDHandler::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(SystemSensorData);
}

void LEDHandler::update(LEDRequest &ledRequest)
{
    update();

    //! reset
    for (int led = 0; led < LEDRequest::numOfLEDs; led++)
        ledRequest.ledStates[led] = LEDRequest::off;

    setLeftEye(ledRequest);
    setRightEye(ledRequest);
    
    setChestButton(ledRequest);

    setLeftEar(ledRequest);
    setRightEar(ledRequest);

    setHead(ledRequest);
}

void LEDHandler::setEyeColor(LEDRequest &ledRequest, bool left, EyeColor col, LEDRequest::LEDState s)
{
    LEDRequest::LED first = left ? LEDRequest::faceLeftRed0Deg : LEDRequest::faceRightRed0Deg;

    static const int redOffset = 0,
                     greenOffset = LEDRequest::faceLeftGreen0Deg - LEDRequest::faceLeftRed0Deg,
                     blueOffset = LEDRequest::faceLeftBlue0Deg - LEDRequest::faceLeftRed0Deg,
                     numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg;

    LEDRequest::LEDState halfState = s == LEDRequest::off ? LEDRequest::off : LEDRequest::half;

    switch (col)
    {
    case red:
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + redOffset + i] = s;
        break;
    case green:
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + greenOffset + i] = s;
        break;
    case blue:
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + blueOffset + i] = s;
        break;
    case white:
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + redOffset + i] = s;
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + greenOffset + i] = s;
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + blueOffset + i] = s;
        break;
    case magenta:
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + redOffset + i] = halfState;
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + blueOffset + i] = s;
        break;
    case yellow:
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + greenOffset + i] = halfState;
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + redOffset + i] = s;
        break;
    case cyan:
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + greenOffset + i] = halfState;
        for (int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + blueOffset + i] = s;
        break;
    default:
        printf("Unknown color.\n");
        break;
    }
}

void LEDHandler::setLeftEye(LEDRequest &ledRequest)
{
    setEyeColor(ledRequest, true, white, LEDRequest::on);
}

void LEDHandler::setRightEye(LEDRequest &ledRequest)
{
    setEyeColor(ledRequest, false, white, LEDRequest::on);
}

void LEDHandler::setChestButton(LEDRequest &ledRequest)
{
    if (theRobotInfo->penalty != PENALTY_NONE)
        ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
    else
    {
        switch (theRobotInfo->state)
        {
        case STATE_READY:
            ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::on;
            break;
        case STATE_SET:
            ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
            ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::half;
            break;
        case STATE_PLAYING:
            ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::on;
            break;

        default:
            break;
        }
    }
}

void LEDHandler::setBatteryLevelInEar(LEDRequest &ledRequest, LEDRequest::LED baseLED)
{
    int onLEDs = std::min(static_cast<int>(theSystemSensorData->batteryLevel / 0.1f), 9);

    for (int i = 0; i <= onLEDs; ++i)
        ledRequest.ledStates[baseLED + i] = LEDRequest::on;
}

void LEDHandler::setRightEar(LEDRequest &ledRequest)
{
    // right ear -> battery
    setBatteryLevelInEar(ledRequest, LEDRequest::earsRight0Deg);
}

void LEDHandler::setLeftEar(LEDRequest &ledRequest)
{
    setBatteryLevelInEar(ledRequest, LEDRequest::earsLeft0Deg);
}

void LEDHandler::setHead(LEDRequest &ledRequest)
{
    if (theSystemSensorData->batteryLevel > 0.9f)
    {
        for (int i = LEDRequest::firstHeadLED; i <= LEDRequest::lastHeadLED; i++)
        {
            ledRequest.ledStates[i] = LEDRequest::on;
        }
    }
    else if (theSystemSensorData->batteryCharging)
    {
        for (LEDRequest::LED i = LEDRequest::firstHeadLED; i <= LEDRequest::lastHeadLED; i = LEDRequest::LED(unsigned(i) + 1))
        {
            ledRequest.ledStates[i] = LEDRequest::off;
        }
        ++chargingLED %= (LEDRequest::numOfHeadLEDs * chargingLightSlowness);
        const LEDRequest::LED currentLED = headLEDCircle[chargingLED / chargingLightSlowness];
        const LEDRequest::LED nextLED = headLEDCircle[(chargingLED / chargingLightSlowness + 1u) % LEDRequest::numOfHeadLEDs];
        ledRequest.ledStates[currentLED] = LEDRequest::on;
        ledRequest.ledStates[nextLED] = LEDRequest::on;
    }
}
