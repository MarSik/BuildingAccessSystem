#include "config.h"
#include "hal.h"
#include "Utils.h"
#include "states.h"

void SetLED(eLED e_LED)
{
    Utils::WritePin(LED_RED_PIN,   LOW);
    Utils::WritePin(LED_GREEN_PIN, LOW);
    Utils::WritePin(LED_BUILTIN,   LOW);

    if (mfrcIntf.ready()) {
        mfrcIntf.led(1, false);
        mfrcIntf.led(2, false);
    }

    switch (e_LED)
    {
        case LED_RED:
            Utils::WritePin(LED_RED_PIN, HIGH);
            if (mfrcIntf.ready())
                mfrcIntf.led(2, true);
            break;
        case LED_GREEN:
            Utils::WritePin(LED_GREEN_PIN, HIGH);
            if (mfrcIntf.ready())
                mfrcIntf.led(1, true);
            break;
        default:  // Just to avoid gcc compiler warning
            break;
    }
}
