#include <Arduino.h>
#include "src/pins.h"
#include "src/config.h"
#include "src/LimitSwitch.h"

#if defined(PIN_ST5_LIMIT)
  static const uint8_t LIMIT_PIN = PIN_ST5_LIMIT;
#elif defined(PIN_LIM5)
  static const uint8_t LIMIT_PIN = PIN_LIM5;
#else
#error "No limit switch pin defined. Define PIN_ST5_LIMIT in config.h or enable PIN_LIM5."
#endif

LimitSwitch limitSwitch(LIMIT_PIN, LIMIT_ACTIVE_LOW);

bool lastTriggeredState = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    ;
  }

  Serial.println(F("=== Limit Switch Test ==="));
  Serial.print(F("Using pin: "));
  Serial.println(LIMIT_PIN);
  Serial.print(F("Active-low mode: "));
  Serial.println(LIMIT_ACTIVE_LOW ? F("YES") : F("NO"));

  limitSwitch.begin();
  lastTriggeredState = limitSwitch.triggered();
  Serial.print(F("Initial state: "));
  Serial.println(lastTriggeredState ? F("TRIGGERED") : F("NOT TRIGGERED"));
}

void loop() {
  bool triggered = limitSwitch.triggered();
  if (triggered != lastTriggeredState) {
    lastTriggeredState = triggered;
    if (triggered) {
      Serial.println(F("LIMIT SWITCH TRIGGERED"));
    } else {
      Serial.println(F("Limit switch released"));
    }
  }
  delay(100);
}
