#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(uint8_t pin, bool activeLow)
    : pin_(pin), activeLow_(activeLow)
{
}

void LimitSwitch::begin()
{
    pinMode(pin_, INPUT_PULLUP);
}

bool LimitSwitch::triggered() const
{
    bool state = digitalRead(pin_);
    return activeLow_ ? (state == LOW) : (state == HIGH);
}
