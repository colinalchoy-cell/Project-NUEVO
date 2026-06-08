#pragma once
#include <Arduino.h>

class LimitSwitch {
public:
    LimitSwitch(uint8_t pin, bool activeLow = true);

    void begin();
    bool triggered() const;

private:
    uint8_t pin_;
    bool activeLow_;
};
