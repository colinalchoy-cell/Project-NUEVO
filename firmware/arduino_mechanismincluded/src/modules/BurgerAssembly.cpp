#include "BurgerAssembly.h"
#include "DebugLog.h"

namespace BurgerAssembly {

    // ============================================================================
    // STATE MACHINE
    // ============================================================================

    enum class State : uint8_t {
        IDLE,

        // Homing
        HOME_STEPPER_START,
        HOME_STEPPER_MOVING,
        HOME_LIFT_START,
        HOME_LIFT_MOVING,
        HOME_LIFT_BACKOFF,

        // Per-ingredient sub-sequence (re-entered 3 times)
        INGREDIENT_LIFT_ABOVE,
        INGREDIENT_STEP_TO_TABLE,
        INGREDIENT_LIFT_TO_TABLE,
        INGREDIENT_STEP_TO_DYNAMIC,
        INGREDIENT_LIFT_TO_DROP,
        INGREDIENT_STEP_TO_STATIC,

        // Drive between ingredients
        DRIVE_FORWARD,

        // Final transfer
        TRANSFER_LIFT_DOWN,
        TRANSFER_STEP_TO_DYNAMIC,

        DONE,
        ABORTED,
    };

    // Ingredient sub-sequence context
    struct IngredientCtx {
        const char* name;
        int32_t     dcDropHeight;
    };

    static const IngredientCtx kIngredients[] = {
        { "Bottom Bun", kDcStaticPlatform      },
        { "Patty",      kDcStaticPlusBun       },
        { "Top Bun",    kDcStaticPlusBunPatty  },
    };
    static const uint8_t kNumIngredients = sizeof(kIngredients) / sizeof(kIngredients[0]);

    // ============================================================================
    // MODULE STATE
    // ============================================================================

    static IEncoderCounter* sLiftEncoder = nullptr;
    static State            sState = State::IDLE;
    static bool             sStopRequested = false;

    // Ingredient sequencing
    static uint8_t  sIngredientIndex = 0;   // which ingredient we're on
    static uint32_t sDriveAfterIndex = 0;   // which drive leg (0=after bun, 1=after patty)

    // Timing
    static uint32_t sStateStartMs = 0;
    static uint32_t sStateStartUs = 0;

    // Non-blocking stepper state
    static int32_t  sStepperPosition = 0;
    static int32_t  sStepperTarget = 0;
    static int8_t   sStepperDir = 0;
    static int32_t  sStepperRemaining = 0;
    static uint32_t sLastStepUs = 0;
    static bool     sStepPinHigh = false;

    // ============================================================================
    // INTERNAL HELPERS — lift motor
    // ============================================================================

    static int32_t liftPosition() {
        return sLiftEncoder ? sLiftEncoder->getCount() : 0;
    }

    static void setLiftPwm(int16_t pwm) {
        if (pwm > 255) pwm = 255;
        if (pwm < -255) pwm = -255;
        if (kBaLiftDirInv) pwm = -pwm;

        if (pwm > 0) {
            digitalWrite(kBaLiftDir1Pin, HIGH);
            digitalWrite(kBaLiftDir2Pin, LOW);
            analogWrite(kBaLiftPwmPin, (uint8_t)pwm);
        }
        else if (pwm < 0) {
            digitalWrite(kBaLiftDir1Pin, LOW);
            digitalWrite(kBaLiftDir2Pin, HIGH);
            analogWrite(kBaLiftPwmPin, (uint8_t)(-pwm));
        }
        else {
            digitalWrite(kBaLiftDir1Pin, LOW);
            digitalWrite(kBaLiftDir2Pin, LOW);
            analogWrite(kBaLiftPwmPin, 0);
        }
    }

    static void stopLift() { setLiftPwm(0); }

    static bool liftAtTarget(int32_t target) {
        return abs(target - liftPosition()) < kBaDcTolerance;
    }

    static void startLiftMove(int32_t target) {
        int32_t error = target - liftPosition();
        setLiftPwm(error > 0 ? kBaLiftMovePwm : -kBaLiftMovePwm);
    }

    // ============================================================================
    // INTERNAL HELPERS — drive motors
    // ============================================================================

    static void setDriveMotorPwm(uint8_t motorId, int16_t pwm) {
        uint8_t pinEN, pinIN1, pinIN2;
        bool dirInverted;
        switch (motorId) {
        case 0:
            pinEN = PIN_M1_EN; pinIN1 = PIN_M1_IN1; pinIN2 = PIN_M1_IN2;
            dirInverted = DC_MOTOR_1_DIR_INVERTED; break;
        case 1:
            pinEN = PIN_M2_EN; pinIN1 = PIN_M2_IN1; pinIN2 = PIN_M2_IN2;
            dirInverted = DC_MOTOR_2_DIR_INVERTED; break;
        default: return;
        }
        if (pwm > 255) pwm = 255;
        if (pwm < -255) pwm = -255;
        if (dirInverted) pwm = -pwm;

        if (pwm > 0) {
            digitalWrite(pinIN1, HIGH); digitalWrite(pinIN2, LOW);
            analogWrite(pinEN, (uint8_t)pwm);
        }
        else if (pwm < 0) {
            digitalWrite(pinIN1, LOW); digitalWrite(pinIN2, HIGH);
            analogWrite(pinEN, (uint8_t)(-pwm));
        }
        else {
            digitalWrite(pinIN1, LOW); digitalWrite(pinIN2, LOW);
            analogWrite(pinEN, 0);
        }
    }

    static void stopDrive() {
        setDriveMotorPwm(0, 0);
        setDriveMotorPwm(1, 0);
    }

    static void startDrive() {
        setDriveMotorPwm(0, kDriveForwardPwm);
        setDriveMotorPwm(1, kDriveForwardPwm);
    }

    // ============================================================================
    // INTERNAL HELPERS — stepper (non-blocking)
    // ============================================================================

    static void enableStepper() { digitalWrite(kBaEnPin, LOW); }
    static void disableStepper() { digitalWrite(kBaEnPin, HIGH); }

    static void startStepperMove(int32_t target) {
        int32_t delta = target - sStepperPosition;
        if (delta == 0) return;
        sStepperTarget = target;
        sStepperDir = (delta > 0) ? 1 : -1;
        sStepperRemaining = abs(delta);
        sLastStepUs = micros();
        sStepPinHigh = false;
        digitalWrite(kBaDirPin, sStepperDir > 0 ? HIGH : LOW);
        enableStepper();
    }

    static bool stepperAtTarget() {
        return sStepperRemaining == 0;
    }

    /**
     * @brief Advance stepper by one step if the interval has elapsed.
     * Called every task() tick. Never blocks.
     */
    static void tickStepper() {
        if (sStepperRemaining == 0) return;

        uint32_t now = micros();
        if ((uint32_t)(now - sLastStepUs) < kBaStepPulseUs) return;
        sLastStepUs = now;

        if (!sStepPinHigh) {
            digitalWrite(kBaStepPin, HIGH);
            sStepPinHigh = true;
        }
        else {
            digitalWrite(kBaStepPin, LOW);
            sStepPinHigh = false;
            sStepperPosition += sStepperDir;
            sStepperRemaining--;
            if (sStepperRemaining == 0) {
                disableStepper();
            }
        }
    }

    // ============================================================================
    // INTERNAL HELPERS — limit switches
    // ============================================================================

    static bool liftLimitTriggered() {
        uint8_t activeState = LIMIT_ACTIVE_LOW ? LOW : HIGH;
        return digitalRead(kBaLiftLimitPin) == activeState;
    }

    static bool stepperLimitTriggered() {
        uint8_t activeState = LIMIT_ACTIVE_LOW ? LOW : HIGH;
        return digitalRead(kBaStepLimitPin) == activeState;
    }

    // ============================================================================
    // INTERNAL — state transitions
    // ============================================================================

    static void transitionTo(State next) {
        sState = next;
        sStateStartMs = millis();
        sStateStartUs = micros();
    }

    static void abortSequence(const __FlashStringHelper* reason) {
        stopLift();
        stopDrive();
        disableStepper();
        DEBUG_SERIAL.println(reason);
        transitionTo(State::ABORTED);
    }

    // ============================================================================
    // PUBLIC API
    // ============================================================================

    void init(IEncoderCounter* liftEncoder) {
        sLiftEncoder = liftEncoder;

        // Lift motor pins
        pinMode(kBaLiftPwmPin, OUTPUT);
        pinMode(kBaLiftDir1Pin, OUTPUT);
        pinMode(kBaLiftDir2Pin, OUTPUT);
        stopLift();

        // Drive motor pins
        pinMode(PIN_M1_EN, OUTPUT); pinMode(PIN_M1_IN1, OUTPUT); pinMode(PIN_M1_IN2, OUTPUT);
        pinMode(PIN_M2_EN, OUTPUT); pinMode(PIN_M2_IN1, OUTPUT); pinMode(PIN_M2_IN2, OUTPUT);
        stopDrive();

        // Stepper pins
        pinMode(kBaStepPin, OUTPUT);
        pinMode(kBaDirPin, OUTPUT);
        pinMode(kBaEnPin, OUTPUT);
        disableStepper();

        // Limit switch pins
        pinMode(kBaLiftLimitPin, LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
        pinMode(kBaStepLimitPin, LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);

        sState = State::IDLE;
        sStopRequested = false;
        DEBUG_SERIAL.println(F("[BurgerAssembly] Initialized."));
    }

    void start() {
        if (sState != State::IDLE && sState != State::DONE && sState != State::ABORTED) {
            DEBUG_SERIAL.println(F("[BurgerAssembly] Already running."));
            return;
        }
        sStopRequested = false;
        sIngredientIndex = 0;
        sDriveAfterIndex = 0;
        sStepperPosition = 0;
        DEBUG_SERIAL.println(F("[BurgerAssembly] Starting sequence."));
        transitionTo(State::HOME_STEPPER_START);
    }

    void stop() {
        sStopRequested = true;
    }

    bool isRunning() {
        return sState != State::IDLE
            && sState != State::DONE
            && sState != State::ABORTED;
    }

    // ============================================================================
    // TASK — non-blocking state machine tick
    // ============================================================================

    void task() {
        // Always tick stepper so pulses complete even mid-transition
        tickStepper();

        if (sStopRequested && isRunning()) {
            abortSequence(F("[BurgerAssembly] Stopped by request."));
            return;
        }

        switch (sState) {

            // ── IDLE / terminal states ──────────────────────────────────────────
        case State::IDLE:
        case State::DONE:
        case State::ABORTED:
            return;

            // ── HOME STEPPER ────────────────────────────────────────────────────
        case State::HOME_STEPPER_START:
            DEBUG_SERIAL.println(F("[BurgerAssembly] Homing stepper..."));
            sStepperPosition = 0;
            sStepperRemaining = 0;
            // Start homing: drive stepper in home direction continuously
            // We use the raw step pin here since we're not tracking position yet
            digitalWrite(kBaDirPin, kBaStepHomeDir > 0 ? HIGH : LOW);
            enableStepper();
            sLastStepUs = micros();
            sStepPinHigh = false;
            // Repurpose sStepperRemaining=INT32_MAX as "run until limit"
            sStepperRemaining = INT32_MAX;
            sStepperDir = kBaStepHomeDir;
            transitionTo(State::HOME_STEPPER_MOVING);
            break;

        case State::HOME_STEPPER_MOVING:
            if (millis() - sStateStartMs > kBaStepperHomeTimeoutMs) {
                abortSequence(F("[BurgerAssembly] Stepper home timeout."));
                return;
            }
            if (stepperLimitTriggered()) {
                // Hit the limit — stop and zero
                sStepperRemaining = 0;
                disableStepper();
                sStepperPosition = 0;
                DEBUG_SERIAL.println(F("[BurgerAssembly] Stepper homed."));
                transitionTo(State::HOME_LIFT_START);
            }
            // Otherwise tickStepper() at top of task() keeps pulsing
            break;

            // ── HOME LIFT ───────────────────────────────────────────────────────
        case State::HOME_LIFT_START:
            DEBUG_SERIAL.println(F("[BurgerAssembly] Homing lift..."));
            setLiftPwm(kBaLiftHomeDir * kBaLiftHomePwm);
            transitionTo(State::HOME_LIFT_MOVING);
            break;

        case State::HOME_LIFT_MOVING:
            if (millis() - sStateStartMs > kBaLiftHomeTimeoutMs) {
                stopLift();
                abortSequence(F("[BurgerAssembly] Lift home timeout."));
                return;
            }
            if (liftLimitTriggered()) {
                stopLift();
                sLiftEncoder->resetCount();
                // Back off until switch releases
                uint8_t backoffPwm = max(kBaLiftHomePwm / 2, 40);
                setLiftPwm(-kBaLiftHomeDir * backoffPwm);
                transitionTo(State::HOME_LIFT_BACKOFF);
            }
            break;

        case State::HOME_LIFT_BACKOFF:
            if (!liftLimitTriggered()) {
                stopLift();
                delay(200);  // short settle — acceptable here, only happens once at startup
                sLiftEncoder->resetCount();
                DEBUG_SERIAL.println(F("[BurgerAssembly] Lift homed."));
                // Start first ingredient
                DEBUG_SERIAL.print(F("[BurgerAssembly] Picking up: "));
                DEBUG_SERIAL.println(kIngredients[sIngredientIndex].name);
                startLiftMove(kDcAboveTable);
                transitionTo(State::INGREDIENT_LIFT_ABOVE);
            }
            break;

            // ── INGREDIENT SUB-SEQUENCE ─────────────────────────────────────────

        case State::INGREDIENT_LIFT_ABOVE:
            if (liftAtTarget(kDcAboveTable)) {
                stopLift();
                startStepperMove(kStepperTable);
                transitionTo(State::INGREDIENT_STEP_TO_TABLE);
            }
            break;

        case State::INGREDIENT_STEP_TO_TABLE:
            if (stepperAtTarget()) {
                startLiftMove(kDcTableHeight);
                transitionTo(State::INGREDIENT_LIFT_TO_TABLE);
            }
            break;

        case State::INGREDIENT_LIFT_TO_TABLE:
            if (liftAtTarget(kDcTableHeight)) {
                stopLift();
                startStepperMove(kStepperDynamicPlatform);
                transitionTo(State::INGREDIENT_STEP_TO_DYNAMIC);
            }
            break;

        case State::INGREDIENT_STEP_TO_DYNAMIC:
            if (stepperAtTarget()) {
                startLiftMove(kIngredients[sIngredientIndex].dcDropHeight);
                transitionTo(State::INGREDIENT_LIFT_TO_DROP);
            }
            break;

        case State::INGREDIENT_LIFT_TO_DROP:
            if (liftAtTarget(kIngredients[sIngredientIndex].dcDropHeight)) {
                stopLift();
                startStepperMove(kStepperStaticPlatform);
                transitionTo(State::INGREDIENT_STEP_TO_STATIC);
            }
            break;

        case State::INGREDIENT_STEP_TO_STATIC:
            if (stepperAtTarget()) {
                DEBUG_SERIAL.print(F("[BurgerAssembly] Done: "));
                DEBUG_SERIAL.println(kIngredients[sIngredientIndex].name);
                sIngredientIndex++;

                if (sIngredientIndex >= kNumIngredients) {
                    // All ingredients picked up — do final transfer
                    startLiftMove(kDcStaticPlatform);
                    transitionTo(State::TRANSFER_LIFT_DOWN);
                }
                else if (sDriveAfterIndex < 2) {
                    // Drive forward to next ingredient position
                    uint32_t driveMs = (sDriveAfterIndex == 0)
                        ? kDriveBottomBunMs
                        : kDrivePattyMs;
                    DEBUG_SERIAL.print(F("[BurgerAssembly] Driving forward "));
                    DEBUG_SERIAL.println(driveMs);
                    startDrive();
                    sStateStartMs = millis();
                    // Reuse DRIVE_FORWARD state, store duration in sStepperTarget
                    sStepperTarget = (int32_t)driveMs;
                    sDriveAfterIndex++;
                    transitionTo(State::DRIVE_FORWARD);
                }
                else {
                    // Next ingredient, no drive needed
                    startLiftMove(kDcAboveTable);
                    transitionTo(State::INGREDIENT_LIFT_ABOVE);
                }
            }
            break;

            // ── DRIVE FORWARD ───────────────────────────────────────────────────
        case State::DRIVE_FORWARD:
            if ((uint32_t)(millis() - sStateStartMs) >= (uint32_t)sStepperTarget) {
                stopDrive();
                DEBUG_SERIAL.println(F("[BurgerAssembly] Drive complete."));
                // Continue to next ingredient
                DEBUG_SERIAL.print(F("[BurgerAssembly] Picking up: "));
                DEBUG_SERIAL.println(kIngredients[sIngredientIndex].name);
                startLiftMove(kDcAboveTable);
                transitionTo(State::INGREDIENT_LIFT_ABOVE);
            }
            break;

            // ── FINAL TRANSFER ──────────────────────────────────────────────────
        case State::TRANSFER_LIFT_DOWN:
            if (liftAtTarget(kDcStaticPlatform)) {
                stopLift();
                startStepperMove(kStepperDynamicPlatform);
                transitionTo(State::TRANSFER_STEP_TO_DYNAMIC);
            }
            break;

        case State::TRANSFER_STEP_TO_DYNAMIC:
            if (stepperAtTarget()) {
                DEBUG_SERIAL.println(F("[BurgerAssembly] Burger assembly complete!"));
                transitionTo(State::DONE);
            }
            break;
        }
    }

} // namespace BurgerAssembly