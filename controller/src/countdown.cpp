#include <Arduino.h>
#include "countdown.h"

static volatile uint32_t _counter = 0;
static volatile uint32_t _lastKnownMillis = 0;
static volatile bool _countdownFinished = false;

void _processCountdown(uint32_t _millis) {
    if (_counter > 0) {
        if((--_counter) == 0) {
            _countdownFinished = true;
        }
    }
}

void initializeCountdown() {
    _counter = 0;
    _countdownFinished = false;
    registerSysTickCb(_processCountdown);
}

void countdown(uint32_t millis) {
    noInterrupts();
    _counter = millis;
    _countdownFinished = false;
    interrupts();
}

bool countdownFinished() {
    if (_countdownFinished) {
        _countdownFinished = false;
        return true;
    } else {
        return false;
    }
}

void countdownAbort() {
    countdown(0);
}