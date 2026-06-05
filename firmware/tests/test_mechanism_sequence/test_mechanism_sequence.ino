/**
 * @file burger_assembly.ino
 * @brief Automated burger assembly sequence using M3 DC motor and Stepper 1
 *
 * Sequence:
 *   1. Zero stepper and DC motor (homing)
 *   2. Pick up bottom bun
 *   3. Pick up patty
 *   4. Pick up top bun
 *   5. Transfer burger to dynamic platform
 *  
 *  g to start sequence
 *  5 to emergency stop
 *
 * Hardware assumptions:
 *  - M3 motor driver connected to PIN_M3_EN / PIN_M3_IN1 / PIN_M3_IN2
 *  - M3 home switch wired to PIN_LIM5
 *  - Stepper 1 driver connected to PIN_ST1_STEP / PIN_ST1_DIR / PIN_ST1_EN
 *  - Stepper 1 home switch wired to PIN_ST1_LIMIT (PIN_LIM1 by default)
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/modules/EncoderCounter.h"

EncoderCounter4x m3Encoder;

#define DEBUG_SERIAL Serial

ISR(PCINT2_vect) {
    m3Encoder.onInterruptA();
}

// ============================================================================
// HARD-CODED POSITIONS — edit these to match your mechanism
// ============================================================================

// DC motor encoder positions (counts from home)
const int32_t kDcAboveTable      = 8000;   // High enough to clear table surface
const int32_t kDcTableHeight     = 4000;    // Down to table level to pick up ingredient
const int32_t kDcStaticPlatform  = 820;    // Down to static platform surface
const int32_t kDcStaticPlusBun   = 1000;    // Static platform + one bun height
const int32_t kDcStaticPlusBunPatty = 2000; // Static platform + bun + patty height

// Stepper positions (steps from home)
const int32_t kStepperTable           = 4000;   // Above ingredient on table
const int32_t kStepperDynamicPlatform = 2300;   // Dynamic platform drop-off
const int32_t kStepperStaticPlatform  = 0;  // Static platform drop-off

// ============================================================================
// HARDWARE CONSTANTS
// ============================================================================

const uint8_t kDcPwmPin          = PIN_M3_EN;
const uint8_t kDcDir1Pin         = PIN_M3_IN1;
const uint8_t kDcDir2Pin         = PIN_M3_IN2;
const bool    kDcDirInverted     = DC_MOTOR_3_DIR_INVERTED;
const uint8_t kDcLimitPin        = PIN_LIM5;
const int8_t  kDcHomeDirection   = -1;
const uint8_t kDcHomePwm         = 100;
const uint8_t kDcMovePwm         = 120;     // PWM for normal moves

const uint8_t  kStepperStepPin      = PIN_ST1_STEP;
const uint8_t  kStepperDirPin       = PIN_ST1_DIR;
const uint8_t  kStepperEnablePin    = PIN_ST1_EN;
const uint8_t  kStepperLimitPin     = PIN_ST1_LIMIT;
const int8_t   kStepperHomeDirection = -1;
const uint16_t kStepperStepPulseUs  = 2000;

// ============================================================================
// STATE
// ============================================================================

bool    stopRequested   = false;
int32_t stepperPosition = 0;

// ============================================================================
// LOW-LEVEL MOTOR HELPERS
// ============================================================================

int32_t dcMotorPosition() {
    return m3Encoder.getCount();
}

void pollStopRequest() {
    while (DEBUG_SERIAL.available() > 0) {
        char c = DEBUG_SERIAL.read();
        if (c == '\r' || c == '\n') continue;
        if (c == '5') {
            stopRequested = true;
            break;
        }
    }
}

bool isLimitTriggered(uint8_t pin, uint8_t activeState) {
    return digitalRead(pin) == activeState;
}

void setMotorPwm(int16_t pwm) {
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

void stopDcMotor() {
    setMotorPwm(0);
}

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

/**
 * @brief Move DC motor to an absolute encoder position and block until there.
 */
void moveDcTo(int32_t target) {
    DEBUG_SERIAL.print(F("[DC] Moving to "));
    DEBUG_SERIAL.println(target);

    int32_t error = target - dcMotorPosition();
    if (abs(error) < 5) {
        DEBUG_SERIAL.println(F("[DC] Already at target."));
        return;
    }

    setMotorPwm(error > 0 ? kDcMovePwm : -kDcMovePwm);

    while (!stopRequested) {
        pollStopRequest();
        error = target - dcMotorPosition();
        if (abs(error) < 5) break;
    }

    stopDcMotor();
    DEBUG_SERIAL.print(F("[DC] Arrived at "));
    DEBUG_SERIAL.println(dcMotorPosition());
}

/**
 * @brief Move stepper to an absolute step position and block until there.
 */
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
    DEBUG_SERIAL.println(F("[DC] Homing..."));
    pinMode(kDcLimitPin, LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);

    stopDcMotor();
    stopRequested = false;
    setMotorPwm(kDcHomeDirection * kDcHomePwm);

    uint32_t startMs = millis();
    bool triggered = false;
    while (millis() - startMs < 10000 && !stopRequested) {
        pollStopRequest();
        if (isLimitTriggered(kDcLimitPin, LIMIT_ACTIVE_LOW ? LOW : HIGH)) {
            triggered = true;
            break;
        }
    }

    stopDcMotor();

    if (triggered) {
        m3Encoder.resetCount();

        // Back off until switch releases
        uint8_t backoffPwm = max(kDcHomePwm / 2, 40);
        setMotorPwm(-kDcHomeDirection * backoffPwm);
        while (isLimitTriggered(kDcLimitPin, LIMIT_ACTIVE_LOW ? LOW : HIGH));
        stopDcMotor();
        delay(200);
        m3Encoder.resetCount();
        DEBUG_SERIAL.println(F("[DC] Homed."));
    } else {
        DEBUG_SERIAL.println(F("[DC] Home FAILED — timeout or aborted."));
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

    // Raise DC above table
    moveDcTo(kDcAboveTable);
    // Stepper to above ingredient on table
    moveStepperTo(kStepperTable);
    // DC down to table height to grab ingredient
    moveDcTo(kDcTableHeight);
    // Stepper carry ingredient to dynamic platform position
    moveStepperTo(kStepperDynamicPlatform);
    // DC down to drop height on static platform
    moveDcTo(dcDropHeight);
    // Stepper slide to static platform drop-off
    moveStepperTo(kStepperStaticPlatform);

    DEBUG_SERIAL.print(F("--- Done: "));
    DEBUG_SERIAL.println(name);
}

void transferBurgerToDynamicPlatform() {
    DEBUG_SERIAL.println(F("\n--- Transferring burger to dynamic platform"));

    // DC down to static platform level to engage burger
    moveDcTo(kDcStaticPlatform);
    // Stepper pull burger back to dynamic platform
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

    // Step 2: Pick up bottom bun
    // Drop at static platform height (nothing stacked yet)
    pickUpIngredient("Bottom Bun", kDcStaticPlatform);
    if (stopRequested) return;

    // TODO: drive robot forward to patty position

    // Step 3: Pick up patty
    // Drop at static platform + bun height (stacking on top of bun)
    pickUpIngredient("Patty", kDcStaticPlusBun);
    if (stopRequested) return;

    // TODO: drive robot forward to top bun position

    // Step 4: Pick up top bun
    // Drop at static platform + bun + patty height
    pickUpIngredient("Top Bun", kDcStaticPlusBunPatty);
    if (stopRequested) return;

    // Step 5: Transfer assembled burger to dynamic platform
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

    m3Encoder.init(
        PIN_M3_ENC_A,
        PIN_M3_ENC_B,
        ENCODER_3_DIR_INVERTED);

    // Enable PCINT2 for M3 encoder (A14=PCINT22, A15=PCINT23 on Mega 2560)
    PCMSK2 |= (1 << PCINT14) | (1 << PCINT15);
    PCICR  |= (1 << PCIE2);

    pinMode(kDcPwmPin,  OUTPUT);
    pinMode(kDcDir1Pin, OUTPUT);
    pinMode(kDcDir2Pin, OUTPUT);
    stopDcMotor();

    pinMode(kStepperStepPin,   OUTPUT);
    pinMode(kStepperDirPin,    OUTPUT);
    pinMode(kStepperEnablePin, OUTPUT);
    disableStepper();

    // Limit switch pins always configured regardless of homing
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
            stopDcMotor();
            disableStepper();
            DEBUG_SERIAL.println(F("STOP requested — all motors halted."));
        } else {
            DEBUG_SERIAL.println(F("Send 'g' to start, '5' to stop."));
        }
    }
}