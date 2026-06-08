/**
 * @file test_m3_stepper_homing.ino
 * @brief Standalone Arduino test for M3 DC motor and stepper homing
 *
 * This sketch is designed to run independently and verify:
 *  - DC motor M3 homing using a limit switch
 *  - Stepper motor 1 homing using a limit switch
 *  - Hard-coded DC lift heights via timed PWM
 *  - Hard-coded stepper distances via step count
 *
 * Hardware assumptions:
 *  - M3 motor driver connected to PIN_M3_EN / PIN_M3_IN1 / PIN_M3_IN2
 *  - M3 home switch wired to PIN_LIM5
 *  - Stepper 1 driver connected to PIN_ST1_STEP / PIN_ST1_DIR / PIN_ST1_EN
 *  - Stepper 1 home switch wired to PIN_ST1_LIMIT (PIN_LIM1 by default)
 *
 * Notes:
 *  - The stepper enable pin is assumed active LOW.
 *  - Adjust DC_HOME_DIRECTION or STEPPER_HOME_DIRECTION if the motor moves
 *    away from the limit switch instead of toward it.
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/modules/EncoderCounter.h"
EncoderCounter4x m3Encoder;

#define DEBUG_SERIAL Serial

ISR(PCINT2_vect){
    m3Encoder.onInterruptA();
}

// DC motor M3 hardware
const uint8_t kDcPwmPin = PIN_M3_EN;
const uint8_t kDcDir1Pin = PIN_M3_IN1;
const uint8_t kDcDir2Pin = PIN_M3_IN2;
const bool kDcDirInverted = DC_MOTOR_3_DIR_INVERTED;
const uint8_t kDcLimitPin = PIN_LIM5;  // Home switch for M3 motor
const int8_t kDcHomeDirection = -1;    // +1 or -1; flip if homing direction is wrong
const uint8_t kDcHomePwm = 50
;        // PWM magnitude while homing

// Stepper 1 hardware
const uint8_t kStepperStepPin = PIN_ST1_STEP;
const uint8_t kStepperDirPin = PIN_ST1_DIR;
const uint8_t kStepperEnablePin = PIN_ST1_EN;
const uint8_t kStepperLimitPin = PIN_ST1_LIMIT;  // PIN_LIM1 by default
const int8_t kStepperHomeDirection = -1;         // +1 or -1; flip if needed
const uint16_t kStepperStepPulseUs = 1000;        // Step pulse width and spacing

// Hard-coded test values
const uint32_t kDcEncoderPositions[] = {750, 3500, 5700, 8000};
const int32_t kStepperDistances[] = {0, 1750, 4500};

bool stopRequested = false;

// Position tracking (distance from home)
int32_t dcMotorPosition(){         // DC encoder counts
    return m3Encoder.getCount();
} 
int32_t stepperPosition = 0;       // Position in steps from home
int16_t manualDcPwm = 180;        // Manual DC motor PWM speed
int8_t dcMotorManualDir = 0;       // 1=fwd, -1=rev, 0=stopped
int8_t stepperManualDir = 0;       // 1=fwd, -1=rev, 0=stopped
uint32_t lastStepperUpdateUs = 0;
const uint32_t kStepperStepIntervalUs = 1500; // Step interval (1000 Hz)

void encoderISR_M3A(){
    m3Encoder.onInterruptA();
}

void encoderISR_M3B(){
    m3Encoder.onInterruptB();
}

bool isLimitTriggered(uint8_t pin, uint8_t activeState) {
    return digitalRead(pin) == activeState;
}

void pollStopRequest() {
    while (DEBUG_SERIAL.available() > 0) {
        char c = DEBUG_SERIAL.read();
        if (c == '\r' || c == '\n') {
            continue;
        }
        if (c == '5') {
            stopRequested = true;
            break;
        }
    }
}

void setMotorPwm(int16_t pwm) {
    if (pwm > 255) pwm = 100;
    if (pwm < -255) pwm = -100;
    if (kDcDirInverted) {
        pwm = -pwm;
    }

    if (pwm > 0) {
        digitalWrite(kDcDir1Pin, HIGH);
        digitalWrite(kDcDir2Pin, LOW);
        analogWrite(kDcPwmPin, pwm);
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
    dcMotorManualDir = 0;
}

void manualDriveDcMotor(int8_t direction) {
    if (direction == 0) {
        stopDcMotor();
        return;
    }
    dcMotorManualDir = direction;
    setMotorPwm(direction * manualDcPwm);
}

void updateManualDcMotor()
{
    if (dcMotorManualDir == 0)
        return;
    // if (isLimitTriggered(kDcLimitPin, LIMIT_ACTIVE_LOW ? LOW : HIGH))
    // {
    //     stopDcMotor();
    //     m3Encoder.resetCount();
    //     DEBUG_SERIAL.println(
    //         F("[DC] Home switch reached"));
    // }
}

void enableStepper() {
    digitalWrite(kStepperEnablePin, LOW);  // active LOW enable
}

void disableStepper() {
    digitalWrite(kStepperEnablePin, HIGH); // disable
    stepperManualDir = 0;
}

void manualDriveStepper(int8_t direction) {
    if (direction == 0) {
        disableStepper();
        return;
    }
    stepperManualDir = direction;
    digitalWrite(kStepperDirPin, (direction > 0) ? HIGH : LOW);
    enableStepper();
}

void updateManualStepper() {
    if (stepperManualDir == 0) {
        return;
    }
    uint32_t now = micros();
    if ((uint32_t)(now - lastStepperUpdateUs) >= kStepperStepIntervalUs) {
        lastStepperUpdateUs = now;
        digitalWrite(kStepperStepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(kStepperStepPin, LOW);
        stepperPosition += stepperManualDir;
    }
}

void printDistanceStatus() {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("===== Distance from Home ====="));
    DEBUG_SERIAL.print(F("DC Motor Position: "));
    DEBUG_SERIAL.print(dcMotorPosition()); 
    DEBUG_SERIAL.println(F(" counts"));
    DEBUG_SERIAL.print(F("Stepper Position: "));
    DEBUG_SERIAL.print(stepperPosition);
    DEBUG_SERIAL.println(F(" steps"));
    DEBUG_SERIAL.println();
}

void stepOnce() {
    digitalWrite(kStepperStepPin, HIGH);
    delayMicroseconds(kStepperStepPulseUs);
    digitalWrite(kStepperStepPin, LOW);
    delayMicroseconds(kStepperStepPulseUs);
}

void moveStepperSteps(int32_t steps) {
    if (steps == 0) {
        return;
    }

    int8_t direction = (steps > 0) ? 1 : -1;
    digitalWrite(kStepperDirPin, steps > 0 ? HIGH : LOW);
    enableStepper();
    int32_t count = abs(steps);
    while (count-- > 0 && !stopRequested) {
        stepOnce();
        stepperPosition += direction;  // Track position for each step
        pollStopRequest();
    }
    disableStepper();
}

void homeDcMotor() {
    DEBUG_SERIAL.println(F("[DC] Starting homing sequence"));
    if (LIMIT_ACTIVE_LOW) {
        pinMode(kDcLimitPin, INPUT_PULLUP);
    } else {
        pinMode(kDcLimitPin, INPUT);
    }

    stopDcMotor();
    
    m3Encoder.resetCount(); // Reset position at home
    stopRequested = false;
    int16_t homePwm = kDcHomeDirection * kDcHomePwm;
    setMotorPwm(homePwm);

    uint32_t startMs = millis();
    const uint32_t timeoutMs = 10000;
    bool triggered = false;

    while (millis() - startMs < timeoutMs && !stopRequested) {
        pollStopRequest();
        if (isLimitTriggered(kDcLimitPin, LIMIT_ACTIVE_LOW ? LOW : HIGH)) {
            triggered = true;
            break;
        }
    }

    stopDcMotor();
    if (stopRequested) {
        DEBUG_SERIAL.println(F("[DC] Home aborted by stop command."));
    } else if (triggered) {
        delay(200);
        DEBUG_SERIAL.println(F("[DC] Home limit reached."));
        m3Encoder.resetCount();  // Confirm position reset at home
    } else {
        DEBUG_SERIAL.println(F("[DC] Home timeout - limit switch not reached."));
    }
}

void homeStepper() {
    DEBUG_SERIAL.println(F("[Stepper] Starting homing sequence"));
    if (LIMIT_ACTIVE_LOW) {
        pinMode(kStepperLimitPin, INPUT_PULLUP);
    } else {
        pinMode(kStepperLimitPin, INPUT);
    }

    stopRequested = false;
    stepperPosition = 0;  // Reset position at home
    digitalWrite(kStepperDirPin, kStepperHomeDirection > 0 ? HIGH : LOW);
    enableStepper();

    uint32_t startMs = millis();
    const uint32_t timeoutMs = 15000;
    bool triggered = false;

    while (millis() - startMs < timeoutMs && !stopRequested) {
        pollStopRequest();
        if (isLimitTriggered(kStepperLimitPin, LIMIT_ACTIVE_LOW ? LOW : HIGH)) {
            triggered = true;
            break;
        }
        stepOnce();
    }

    disableStepper();
    if (stopRequested) {
        DEBUG_SERIAL.println(F("[Stepper] Home aborted by stop command."));
    } else if (triggered) {
        DEBUG_SERIAL.println(F("[Stepper] Home limit reached."));
        stepperPosition = 0;  // Confirm position reset at home
    } else {
        DEBUG_SERIAL.println(F("[Stepper] Home timeout - limit switch not reached."));
    }
}

void runDcHeightTest(uint8_t index)
{
    if (index >= sizeof(kDcEncoderPositions)/sizeof(kDcEncoderPositions[0]))
    {
        DEBUG_SERIAL.println(F("[DC] Invalid position."));
        return;
    }

    int32_t target = kDcEncoderPositions[index];
    DEBUG_SERIAL.print(F("[DC] Moving to encoder position "));
    DEBUG_SERIAL.println(target);
    stopRequested = false;
    int32_t current = m3Encoder.getCount();
    int32_t error = target - current;

    if (abs(error) < 10)
    {
        DEBUG_SERIAL.println(F("[DC] Already at target."));
        return;
    }

    int pwm = 60;

    if (error > 0)
        setMotorPwm(pwm);
    else
        setMotorPwm(-pwm);
    while (!stopRequested)
    {
        pollStopRequest();
        current = m3Encoder.getCount();
        error = target - current;
        if (abs(error) < 10)
            break;
    }
    stopDcMotor();
    DEBUG_SERIAL.print(F("[DC] Final encoder count = "));
    DEBUG_SERIAL.println(m3Encoder.getCount());
}

void runStepperDistanceTest(uint8_t index) {
    if (index >= sizeof(kStepperDistances) / sizeof(kStepperDistances[0])) {
        DEBUG_SERIAL.println(F("[Stepper] Invalid distance selection."));
        return;
    }

    DEBUG_SERIAL.print(F("[Stepper] Moving hard-coded distance "));
    DEBUG_SERIAL.println(index + 1);
    // moveStepperSteps(kStepperDistances[index]);
    int32_t targetPosition = kStepperDistances[index];
    int32_t delta = targetPosition - stepperPosition;
    moveStepperSteps(delta);
    DEBUG_SERIAL.println(targetPosition);
    DEBUG_SERIAL.println(stepperPosition);
    DEBUG_SERIAL.println(delta);
    DEBUG_SERIAL.print(F("[Stepper] Completed move of "));
    DEBUG_SERIAL.print(kStepperDistances[index]);
    DEBUG_SERIAL.println(F(" steps. Current position: "));
    DEBUG_SERIAL.println(stepperPosition);
}

void printMenu() {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("===== M3 + Stepper Homing Test Menu ====="));
    DEBUG_SERIAL.println(F("1 - Home M3 DC motor"));
    DEBUG_SERIAL.println(F("2 - Home Stepper 1"));
    DEBUG_SERIAL.println(F("3 - Run DC height test (preset heights)"));
    DEBUG_SERIAL.println(F("4 - Run stepper distance test (preset distances)"));
    DEBUG_SERIAL.println(F("5 - Stop all motors"));
    DEBUG_SERIAL.println(F("u - Manual DC motor forward"));
    DEBUG_SERIAL.println(F("d - Manual DC motor backward"));
    DEBUG_SERIAL.println(F("f - Manual stepper forward"));
    DEBUG_SERIAL.println(F("b - Manual stepper backward"));
    DEBUG_SERIAL.println(F("i - Print distance from home"));
    DEBUG_SERIAL.println(F("h - Print this menu"));
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print(F("Enter command: "));
}

void setup() {
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000);

    m3Encoder.init(
        PIN_M3_ENC_A,
        PIN_M3_ENC_B,
        ENCODER_3_DIR_INVERTED);

    PCMSK2 |= (1 << PCINT14) | (1 << PCINT15);
    PCICR  |= (1 << PCIE2);

    // pinMode(PIN_M3_ENC_A, INPUT_PULLUP);
    // pinMode(PIN_M3_ENC_B, INPUT_PULLUP);   

    pinMode(kDcPwmPin, OUTPUT);
    pinMode(kDcDir1Pin, OUTPUT);
    pinMode(kDcDir2Pin, OUTPUT);
    stopDcMotor();

    pinMode(kStepperStepPin, OUTPUT);
    pinMode(kStepperDirPin, OUTPUT);
    pinMode(kStepperEnablePin, OUTPUT);
    disableStepper();

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("  M3 DC Motor + Stepper Homing Test"));
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("Use PIN_LIM5 for M3 home limit switch."));
    DEBUG_SERIAL.println(F("Use PIN_ST1_LIMIT for stepper 1 home limit switch."));
    printMenu();
}

char readSerialCommand() {
    while (DEBUG_SERIAL.available() > 0) {
        char c = DEBUG_SERIAL.read();
        if (c == '\r' || c == '\n') {
            continue;
        }
        return c;
    }
    return 0;
}

void loop() {

    updateManualStepper();
    updateManualDcMotor();

    char command = readSerialCommand();
    if (command == 0) {
        return;
    }

    switch (command) {
        case '1':
            homeDcMotor();
            break;
        case '2':
            homeStepper();
            break;
        case '3': {
            DEBUG_SERIAL.println(F("Choose DC height: 1=low,2=mid,3=high"));
            char selection = 0;
            while ((selection = readSerialCommand()) == 0);
            runDcHeightTest(selection - '1');
            break;
        }
        case '4': {
            DEBUG_SERIAL.println(F("Choose stepper distance: 1=short,2=medium,3=long"));
            char selection = 0;
            while ((selection = readSerialCommand()) == 0);
            runStepperDistanceTest(selection - '1');
            break;
        }
        case '5':
            stopRequested = true;
            stopDcMotor();
            disableStepper();
            DEBUG_SERIAL.println(F("All motors stopped."));
            break;
        case 'u':
        case 'U':
            manualDriveDcMotor(1);
            DEBUG_SERIAL.println(F("DC motor: forward"));
            break;
        case 'd':
        case 'D':
            manualDriveDcMotor(-1);
            DEBUG_SERIAL.println(F("DC motor: backward"));
            break;
        case 'f':
        case 'F':
            manualDriveStepper(1);
            DEBUG_SERIAL.println(F("Stepper: forward"));
            break;
        case 'b':
        case 'B':
            manualDriveStepper(-1);
            DEBUG_SERIAL.println(F("Stepper: backward"));
            break;
        case 'i':
        case 'I':
            printDistanceStatus();
            break;
        case 'h':
        case 'H':
            printMenu();
            break;
        default:
            DEBUG_SERIAL.println(F("Unknown command."));
            printMenu();
            break;
    }

    if (command >= '1' && command <= '5') {
        printMenu();
    }
    
}
