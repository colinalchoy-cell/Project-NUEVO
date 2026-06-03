/**
 * @file test_m3_arm.ino
 * @brief M3 DC motor test for arm-height control
 *
 * This sketch exercises the M3 DC motor driver pins used by the arm-height
 * motor in manipulation-style operation. It allows manual forward/reverse
 * control via the serial console at 115200 baud.
 *
 * Commands:
 *   u - raise arm (forward)
 *   d - lower arm (reverse)
 *   s - stop motor
 *   + - increase speed
 *   - - decrease speed
 *   p - print current status
 *
 * Hardware:
 *   - M3_EN  = PWM enable pin
 *   - M3_IN1 = direction input 1
 *   - M3_IN2 = direction input 2
 *
 * Notes:
 *   - This sketch uses the production pin map from firmware/arduino/src.
 *   - It does not depend on the full production firmware scheduler.
 */

#include <Arduino.h>
#include "src/config.h"
#include "src/pins.h"

const int16_t PWM_STEP = 32;
int16_t currentSpeed = 128;
// Debug: force EN pin fully on (digital HIGH) to test motor/driver power path.
// Set to true to ignore analogWrite and drive EN high when enabled.
const bool TEST_FORCE_FULL_ON = false;
int8_t currentDirection = 0; // 1 = forward, -1 = reverse, 0 = stop

const uint8_t kStepperStepPin = PIN_ST1_STEP;
const uint8_t kStepperDirPin = PIN_ST1_DIR;
const uint8_t kStepperEnablePin = PIN_ST1_EN;
const uint32_t kStepperStepIntervalUs = 2000; // 500 Hz stepping rate
int8_t currentStepperDirection = 0; // 1 = forward, -1 = reverse, 0 = stop
uint32_t lastStepperStepUs = 0;

static bool isMotorEnabled() {
    return currentDirection != 0;
}

static void stopMotor() {
    digitalWrite(PIN_M3_IN1, LOW);
    digitalWrite(PIN_M3_IN2, LOW);
    analogWrite(PIN_M3_EN, 0);
    currentDirection = 0;
}

static void setMotorPWM(int16_t pwm) {
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;

    bool inverted = (DC_MOTOR_3_DIR_INVERTED != 0);
    if (inverted) {
        pwm = -pwm;
    }

    if (pwm > 0) {
        digitalWrite(PIN_M3_IN1, HIGH);
        digitalWrite(PIN_M3_IN2, LOW);
        analogWrite(PIN_M3_EN, (uint8_t)pwm);
        currentDirection = inverted ? -1 : 1;
    } else if (pwm < 0) {
        digitalWrite(PIN_M3_IN1, LOW);
        digitalWrite(PIN_M3_IN2, HIGH);
        analogWrite(PIN_M3_EN, (uint8_t)(-pwm));
        currentDirection = inverted ? 1 : -1;
    } else {
        digitalWrite(PIN_M3_IN1, LOW);
        digitalWrite(PIN_M3_IN2, LOW);
        analogWrite(PIN_M3_EN, 0);
        currentDirection = 0;
    }
}

static void disableStepper() {
    digitalWrite(kStepperEnablePin, HIGH); // active LOW enable
    currentStepperDirection = 0;
}

static void setStepperDirection(int8_t dir) {
    if (dir == 0) {
        disableStepper();
        return;
    }

    digitalWrite(kStepperDirPin, (dir > 0) ? HIGH : LOW);
    digitalWrite(kStepperEnablePin, LOW);
    currentStepperDirection = dir;
}

static void stopStepper() {
    disableStepper();
}

static void updateStepper() {
    if (currentStepperDirection == 0) {
        return;
    }

    uint32_t now = micros();
    if ((uint32_t)(now - lastStepperStepUs) >= kStepperStepIntervalUs) {
        lastStepperStepUs = now;
        digitalWrite(kStepperStepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(kStepperStepPin, LOW);
    }
}

static void printStatus() {
    Serial.println();
    Serial.print(F("M3 Motor Status -> "));
    if (currentDirection > 0) {
        Serial.print(F("Forward"));
    } else if (currentDirection < 0) {
        Serial.print(F("Reverse"));
    } else {
        Serial.print(F("Stopped"));
    }
    Serial.print(F(" | Speed="));
    Serial.println(currentSpeed);
    Serial.print(F("Stepper1 -> "));
    if (currentStepperDirection > 0) {
        Serial.print(F("Forward"));
    } else if (currentStepperDirection < 0) {
        Serial.print(F("Reverse"));
    } else {
        Serial.print(F("Stopped"));
    }
    Serial.println();
    Serial.println(F("Commands: u=forward d=reverse s=stop +=faster -=slower f=stepper forward b=stepper backward t=stepper stop p=status"));
}

void setup() {
    Serial.begin(DEBUG_BAUD_RATE);
    while (!Serial && millis() < 2000) {
        ;
    }

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  M3 Arm Height Motor Test"));
    Serial.println(F("========================================"));
    Serial.print(F("Using pins: EN="));
    Serial.print(PIN_M3_EN);
    Serial.print(F(" IN1="));
    Serial.print(PIN_M3_IN1);
    Serial.print(F(" IN2="));
    Serial.println(PIN_M3_IN2);
    Serial.print(F("Direction inverted: "));
    Serial.println(DC_MOTOR_3_DIR_INVERTED ? F("YES") : F("NO"));
    Serial.println(F(""));

    pinMode(PIN_M3_EN, OUTPUT);
    pinMode(PIN_M3_IN1, OUTPUT);
    pinMode(PIN_M3_IN2, OUTPUT);
    pinMode(kStepperStepPin, OUTPUT);
    pinMode(kStepperDirPin, OUTPUT);
    pinMode(kStepperEnablePin, OUTPUT);
    stopMotor();
    disableStepper();

    Serial.print(F("Starting speed: "));
    Serial.println(currentSpeed);
    Serial.println(F("Enter command now."));
    printStatus();
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = (char)Serial.read();
        switch (cmd) {
            case 'u':
            case 'U':
                setMotorPWM(currentSpeed);
                Serial.println(F("Command: forward"));
                break;
            case 'd':
            case 'D':
                setMotorPWM(-currentSpeed);
                Serial.println(F("Command: reverse"));
                break;
            case 's':
            case 'S':
                setMotorPWM(0);
                Serial.println(F("Command: stop"));
                break;
            case '+':
                currentSpeed = currentSpeed + PWM_STEP;
                if (currentSpeed > 255) {
                    currentSpeed = 255;
                }
                Serial.print(F("Speed increased to "));
                Serial.println(currentSpeed);
                if (isMotorEnabled()) {
                    setMotorPWM(currentDirection * currentSpeed);
                }
                break;
            case '-':
                currentSpeed = currentSpeed - PWM_STEP;
                if (currentSpeed < 0) {
                    currentSpeed = 0;
                }
                Serial.print(F("Speed decreased to "));
                Serial.println(currentSpeed);
                if (isMotorEnabled()) {
                    setMotorPWM(currentDirection * currentSpeed);
                }
                break;
            case 'f':
            case 'F':
                setStepperDirection(1);
                Serial.println(F("Command: stepper forward"));
                break;
            case 'b':
            case 'B':
                setStepperDirection(-1);
                Serial.println(F("Command: stepper backward"));
                break;
            case 't':
            case 'T':
                stopStepper();
                Serial.println(F("Command: stepper stop"));
                break;
            case 'p':
            case 'P':
                printStatus();
                break;
            default:
                if (cmd != '\r' && cmd != '\n') {
                    Serial.print(F("Unknown command: "));
                    Serial.println(cmd);
                    Serial.println(F("Use u/d/s/+/\-/p/f/b/t."));
                }
                break;
        }
    }

    updateStepper();
}
