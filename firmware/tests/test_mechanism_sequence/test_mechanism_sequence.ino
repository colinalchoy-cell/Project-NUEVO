/**
 * @file burger_assembly.ino
 * @brief Automated burger assembly sequence using M3 DC motor and Stepper 1
 *
 * Sequence:
 *   1. Zero stepper and DC motor (homing)
 *   2. Pick up bottom bun
 *   3. Drive forward to patty position
 *   4. Pick up patty
 *   5. Drive forward to top bun position
 *   6. Pick up top bun
 *   7. Transfer burger to dynamic platform
 *
 * Hardware assumptions:
 *  - M1/M2 drive motors connected to PIN_M1_/PIN_M2_ (front wheels)
 *  - M3 motor driver connected to PIN_M3_EN / PIN_M3_IN1 / PIN_M3_IN2 (lift)
 *  - M3 home switch wired to PIN_LIM5
 *  - Stepper 1 driver connected to PIN_ST1_STEP / PIN_ST1_DIR / PIN_ST1_EN
 *  - Stepper 1 home switch wired to PIN_ST1_LIMIT (PIN_LIM1 by default)
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/modules/EncoderCounter.h"

// ============================================================================
// ENCODER INSTANCES
// ============================================================================

EncoderCounter4x m3Encoder;   // Lift motor encoder

#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif

#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif

#define DEBUG_SERIAL Serial

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

ISR(PCINT2_vect) {
    m3Encoder.onInterruptA();   // M3 lift encoder (A14/A15 PCINT)
}

void encoderISR_M1() { encoder1.onInterruptA(); }
void encoderISR_M2() { encoder2.onInterruptA(); }

// ============================================================================
// HARD-CODED POSITIONS — edit these to match your mechanism
// ============================================================================

// DC lift motor encoder positions (counts from home)
const int32_t kDcAboveTable         = 8000;  // High enough to clear table surface
const int32_t kDcTableHeight        = 5700;   // Down to table level to grab ingredient
const int32_t kDcStaticPlatform     = 820;   // Down to static platform surface
const int32_t kDcStaticPlusBun      = 3500;   // Static platform + one bun height
const int32_t kDcStaticPlusBunPatty = 5700;   // Static platform + bun + patty height

// Stepper positions (steps from home)
const int32_t kStepperTable           = 4500;   // Above ingredient on table
const int32_t kStepperDynamicPlatform = 1750;   // Dynamic platform position
const int32_t kStepperStaticPlatform  = 0;  // Static platform drop-off

// Drive motor settings
const int16_t  kDriveForwardPwm  = 100;    // PWM for driving forward (0-255)
const uint32_t kDriveInitialMs   = 2450;   // Drive after homing to reach first ingredient
const uint32_t kDrivePattyMs     = 365;   // Drive duration to patty position (ms)
const uint32_t kDriveBottomBunMs = 365;   // Drive duration to top bun position (ms)

// ============================================================================
// HARDWARE CONSTANTS — lift motor
// ============================================================================

const uint8_t kDcPwmPin        = PIN_M3_EN;
const uint8_t kDcDir1Pin       = PIN_M3_IN1;
const uint8_t kDcDir2Pin       = PIN_M3_IN2;
const bool    kDcDirInverted   = DC_MOTOR_3_DIR_INVERTED;
const uint8_t kDcLimitPin      = PIN_LIM5;
const int8_t  kDcHomeDirection = -1;
const uint8_t kDcHomePwm       = 80;
const uint8_t kDcMovePwm       = 200;

// Hardware constants — stepper
const uint8_t  kStepperStepPin       = PIN_ST1_STEP;
const uint8_t  kStepperDirPin        = PIN_ST1_DIR;
const uint8_t  kStepperEnablePin     = PIN_ST1_EN;
const uint8_t  kStepperLimitPin      = PIN_ST1_LIMIT;
const int8_t   kStepperHomeDirection = -1;
const uint16_t kStepperStepPulseUs   = 1200;

// ============================================================================
// STATE
// ============================================================================

bool    stopRequested   = false;
int32_t stepperPosition = 0;

// ============================================================================
// LOW-LEVEL HELPERS — lift motor
// ============================================================================

int32_t dcMotorPosition() { return m3Encoder.getCount(); }

void pollStopRequest() {
    while (DEBUG_SERIAL.available() > 0) {
        char c = DEBUG_SERIAL.read();
        if (c == '\r' || c == '\n') continue;
        if (c == '5') { stopRequested = true; break; }
    }
}

bool isLimitTriggered(uint8_t pin, uint8_t activeState) {
    return digitalRead(pin) == activeState;
}

void setLiftPwm(int16_t pwm) {
    if (pwm >  255) pwm =  255;
    if (pwm < -255) pwm = -255;
    if (kDcDirInverted) pwm = -pwm;

    if (pwm > 0) {
        digitalWrite(kDcDir1Pin, HIGH);
        digitalWrite(kDcDir2Pin, LOW);
        analogWrite(kDcPwmPin, (uint8_t)pwm);
    } else if (pwm < 0) {
        digitalWrite(kDcDir1Pin, LOW);
        digitalWrite(kDcDir2Pin, HIGH);
        analogWrite(kDcPwmPin, (uint8_t)(-pwm));
    } else {
        digitalWrite(kDcDir1Pin, LOW);
        digitalWrite(kDcDir2Pin, LOW);
        analogWrite(kDcPwmPin, 0);
    }
}

void stopLiftMotor() { setLiftPwm(0); }

// ============================================================================
// LOW-LEVEL HELPERS — drive motors (M1/M2)
// ============================================================================

/**
 * @brief Set PWM for a drive wheel motor (M1=0, M2=1)
 */
void setDriveMotorPwm(uint8_t motorId, int16_t pwm) {
    uint8_t pinEN, pinIN1, pinIN2;
    bool dirInverted;

    switch (motorId) {
        case 0:
            pinEN = PIN_M1_EN; pinIN1 = PIN_M1_IN1; pinIN2 = PIN_M1_IN2;
            dirInverted = DC_MOTOR_1_DIR_INVERTED;
            break;
        case 1:
            pinEN = PIN_M2_EN; pinIN1 = PIN_M2_IN1; pinIN2 = PIN_M2_IN2;
            dirInverted = DC_MOTOR_2_DIR_INVERTED;
            break;
        default: return;
    }

    if (pwm >  255) pwm =  255;
    if (pwm < -255) pwm = -255;
    if (dirInverted) pwm = -pwm;

    if (pwm > 0) {
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
        analogWrite(pinEN, (uint8_t)pwm);
    } else if (pwm < 0) {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
        analogWrite(pinEN, (uint8_t)(-pwm));
    } else {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);
        analogWrite(pinEN, 0);
    }
}

void stopDriveMotors() {
    setDriveMotorPwm(0, 0);
    setDriveMotorPwm(1, 0);
}

/**
 * @brief Drive both wheels forward for a fixed duration, then stop.
 * @param pwm     PWM magnitude (0-255)
 * @param durationMs  How long to drive (milliseconds)
 */
void driveForward(int16_t pwm, uint32_t durationMs) {
    DEBUG_SERIAL.print(F("[Drive] Forward PWM="));
    DEBUG_SERIAL.print(pwm);
    DEBUG_SERIAL.print(F(" for "));
    DEBUG_SERIAL.print(durationMs);
    DEBUG_SERIAL.println(F("ms"));

    setDriveMotorPwm(0, pwm);
    setDriveMotorPwm(1, pwm);

    uint32_t startMs = millis();
    while (millis() - startMs < durationMs && !stopRequested) {
        pollStopRequest();
    }

    stopDriveMotors();
    DEBUG_SERIAL.println(F("[Drive] Stopped."));
}

// ============================================================================
// LOW-LEVEL HELPERS — stepper
// ============================================================================

void enableStepper()  { digitalWrite(kStepperEnablePin, LOW);  }
void disableStepper() { digitalWrite(kStepperEnablePin, HIGH); }

void stepOnce() {
    digitalWrite(kStepperStepPin, HIGH);
    delayMicroseconds(kStepperStepPulseUs);
    digitalWrite(kStepperStepPin, LOW);
    delayMicroseconds(kStepperStepPulseUs);
}

// ============================================================================
// MOVE PRIMITIVES
// ============================================================================

void moveDcTo(int32_t target) {
    DEBUG_SERIAL.print(F("[Lift] Moving to "));
    DEBUG_SERIAL.println(target);

    int32_t error = target - dcMotorPosition();
    if (abs(error) < 5) {
        DEBUG_SERIAL.println(F("[Lift] Already at target."));
        return;
    }

    setLiftPwm(error > 0 ? kDcMovePwm : -kDcMovePwm);
    while (!stopRequested) {
        pollStopRequest();
        error = target - dcMotorPosition();
        if (abs(error) < 5) break;
    }

    stopLiftMotor();
    DEBUG_SERIAL.print(F("[Lift] Arrived at "));
    DEBUG_SERIAL.println(dcMotorPosition());
}

void moveStepperTo(int32_t target) {
    DEBUG_SERIAL.print(F("[Stepper] Moving to "));
    DEBUG_SERIAL.println(target);

    int32_t delta = target - stepperPosition;
    if (delta == 0) {
        DEBUG_SERIAL.println(F("[Stepper] Already at target."));
        return;
    }

    int8_t direction = (delta > 0) ? 1 : -1;
    digitalWrite(kStepperDirPin, direction > 0 ? HIGH : LOW);
    enableStepper();

    int32_t count = abs(delta);
    while (count-- > 0 && !stopRequested) {
        pollStopRequest();
        stepOnce();
        stepperPosition += direction;
    }

    disableStepper();
    DEBUG_SERIAL.print(F("[Stepper] Arrived at "));
    DEBUG_SERIAL.println(stepperPosition);
}

// ============================================================================
// HOMING
// ============================================================================

void homeDcMotor() {
    DEBUG_SERIAL.println(F("[Lift] Homing..."));
    pinMode(kDcLimitPin, LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);

    stopLiftMotor();
    stopRequested = false;
    setLiftPwm(kDcHomeDirection * kDcHomePwm);

    uint32_t startMs = millis();
    bool triggered = false;
    while (millis() - startMs < 10000 && !stopRequested) {
        pollStopRequest();
        if (isLimitTriggered(kDcLimitPin, LIMIT_ACTIVE_LOW ? LOW : HIGH)) {
            triggered = true;
            break;
        }
    }

    stopLiftMotor();

    if (triggered) {
        m3Encoder.resetCount();
        uint8_t backoffPwm = max(kDcHomePwm / 2, 40);
        setLiftPwm(-kDcHomeDirection * backoffPwm);
        while (isLimitTriggered(kDcLimitPin, LIMIT_ACTIVE_LOW ? LOW : HIGH));
        stopLiftMotor();
        delay(200);
        m3Encoder.resetCount();
        DEBUG_SERIAL.println(F("[Lift] Homed."));
    } else {
        DEBUG_SERIAL.println(F("[Lift] Home FAILED — timeout or aborted."));
    }
}

void homeStepper() {
    DEBUG_SERIAL.println(F("[Stepper] Homing..."));
    pinMode(kStepperLimitPin, LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);

    stopRequested = false;
    digitalWrite(kStepperDirPin, kStepperHomeDirection > 0 ? HIGH : LOW);
    enableStepper();

    uint32_t startMs = millis();
    bool triggered = false;
    while (millis() - startMs < 15000 && !stopRequested) {
        pollStopRequest();
        if (isLimitTriggered(kStepperLimitPin, LIMIT_ACTIVE_LOW ? LOW : HIGH)) {
            triggered = true;
            break;
        }
        stepOnce();
    }

    disableStepper();

    if (triggered) {
        stepperPosition = 0;
        DEBUG_SERIAL.println(F("[Stepper] Homed."));
    } else {
        DEBUG_SERIAL.println(F("[Stepper] Home FAILED — timeout or aborted."));
    }
}

// ============================================================================
// BURGER ASSEMBLY STEPS
// ============================================================================

void pickUpIngredient(const char* name, int32_t dcDropHeight) {
    DEBUG_SERIAL.print(F("\n--- Picking up: "));
    DEBUG_SERIAL.println(name);

    moveDcTo(kDcAboveTable);
    if (stopRequested) return;
    moveStepperTo(kStepperTable);
    if (stopRequested) return;
    moveDcTo(kDcTableHeight);
    if (stopRequested) return;
    moveStepperTo(kStepperDynamicPlatform);
    if (stopRequested) return;
    moveDcTo(dcDropHeight);
    if (stopRequested) return;
    moveStepperTo(kStepperStaticPlatform);

    DEBUG_SERIAL.print(F("--- Done: "));
    DEBUG_SERIAL.println(name);
}

void transferBurgerToDynamicPlatform() {
    DEBUG_SERIAL.println(F("\n--- Transferring burger to dynamic platform"));
    moveDcTo(kDcStaticPlatform);
    if (stopRequested) return;
    moveStepperTo(kStepperDynamicPlatform);
    DEBUG_SERIAL.println(F("--- Transfer complete."));
}

// ============================================================================
// FULL ASSEMBLY SEQUENCE
// ============================================================================

void runBurgerAssembly() {
    DEBUG_SERIAL.println(F("\n========================================"));
    DEBUG_SERIAL.println(F("  Starting Burger Assembly Sequence"));
    DEBUG_SERIAL.println(F("========================================\n"));

    // Step 1: Home both axes
    homeStepper();
    if (stopRequested) return;
    homeDcMotor();
    if (stopRequested) return;

    // Step 2: Drive forward to first ingredient (bottom bun)
    driveForward(kDriveForwardPwm, kDriveInitialMs);
    if (stopRequested) return;

    // Step 3: Pick up bottom bun
    pickUpIngredient("Bottom Bun", kDcStaticPlatform);
    if (stopRequested) return;

    // Step 4: Drive forward to patty position
    driveForward(kDriveForwardPwm, kDrivePattyMs);
    if (stopRequested) return;

    // Step 5: Pick up patty
    pickUpIngredient("Patty", kDcStaticPlusBun);
    if (stopRequested) return;

    // Step 6: Drive forward to top bun position
    driveForward(kDriveForwardPwm, kDriveBottomBunMs);
    if (stopRequested) return;

    // Step 7: Pick up top bun
    pickUpIngredient("Top Bun", kDcStaticPlusBunPatty);
    if (stopRequested) return;

    // Step 8: Transfer assembled burger to dynamic platform
    transferBurgerToDynamicPlatform();

    DEBUG_SERIAL.println(F("\n========================================"));
    DEBUG_SERIAL.println(F("  Burger Assembly Complete!"));
    DEBUG_SERIAL.println(F("========================================\n"));
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000);

    // Lift encoder (M3 — PCINT)
    m3Encoder.init(PIN_M3_ENC_A, PIN_M3_ENC_B, ENCODER_3_DIR_INVERTED);
    PCMSK2 |= (1 << PCINT14) | (1 << PCINT15);
    PCICR  |= (1 << PCIE2);

    // Drive encoders (M1/M2 — hardware INT)
    encoder1.init(PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED);
    encoder2.init(PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED);
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2, CHANGE);

    // Lift motor pins
    pinMode(kDcPwmPin,  OUTPUT);
    pinMode(kDcDir1Pin, OUTPUT);
    pinMode(kDcDir2Pin, OUTPUT);
    stopLiftMotor();

    // Drive motor pins
    pinMode(PIN_M1_EN,  OUTPUT); pinMode(PIN_M1_IN1, OUTPUT); pinMode(PIN_M1_IN2, OUTPUT);
    pinMode(PIN_M2_EN,  OUTPUT); pinMode(PIN_M2_IN1, OUTPUT); pinMode(PIN_M2_IN2, OUTPUT);
    stopDriveMotors();

    // Stepper pins
    pinMode(kStepperStepPin,   OUTPUT);
    pinMode(kStepperDirPin,    OUTPUT);
    pinMode(kStepperEnablePin, OUTPUT);
    disableStepper();

    // Limit switch pins
    pinMode(kDcLimitPin,      LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
    pinMode(kStepperLimitPin, LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);

    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("  Burger Assembly Machine"));
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("Send 'g' to start assembly sequence."));
    DEBUG_SERIAL.println(F("Send '5' at any time to stop all motors."));
}

void loop() {
    if (DEBUG_SERIAL.available() > 0) {
        char c = DEBUG_SERIAL.read();
        if (c == '\r' || c == '\n') return;

        if (c == 'g' || c == 'G') {
            stopRequested = false;
            runBurgerAssembly();
        } else if (c == '5') {
            stopRequested = true;
            stopLiftMotor();
            stopDriveMotors();
            disableStepper();
            DEBUG_SERIAL.println(F("STOP — all motors halted."));
        } else {
            DEBUG_SERIAL.println(F("Send 'g' to start, '5' to stop."));
        }
    }
}