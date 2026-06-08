/**
 * @file BurgerAssembly.h
 * @brief Non-blocking burger assembly state machine for scheduler integration
 *
 * Runs as a soft task inside the existing scheduler. All blocking loops from
 * burger_assembly.ino are converted to state machine transitions so the
 * scheduler's ISRs and UART tasks are never starved.
 */

#ifndef BURGER_ASSEMBLY_H
#define BURGER_ASSEMBLY_H

#include <Arduino.h>
#include "../config.h"
#include "../pins.h"
#include "EncoderCounter.h"

 // ============================================================================
 // HARD-CODED POSITIONS — edit these to match your mechanism
 // ============================================================================

 // DC lift motor encoder positions (counts from home)
static const int32_t kDcAboveTable = 8000;
static const int32_t kDcTableHeight = 4000;
static const int32_t kDcStaticPlatform = 820;
static const int32_t kDcStaticPlusBun = 1000;
static const int32_t kDcStaticPlusBunPatty = 2000;

// Stepper positions (steps from home)
static const int32_t kStepperTable = 4000;
static const int32_t kStepperDynamicPlatform = 2300;
static const int32_t kStepperStaticPlatform = 0;

// Drive motor settings
static const int16_t  kDriveForwardPwm = 250;
static const uint32_t kDriveBottomBunMs = 1000;
static const uint32_t kDrivePattyMs = 1000;

// Lift motor hardware
static const uint8_t kBaLiftPwmPin = PIN_M3_EN;
static const uint8_t kBaLiftDir1Pin = PIN_M3_IN1;
static const uint8_t kBaLiftDir2Pin = PIN_M3_IN2;
static const bool    kBaLiftDirInv = DC_MOTOR_3_DIR_INVERTED;
static const uint8_t kBaLiftLimitPin = PIN_LIM5;
static const int8_t  kBaLiftHomeDir = -1;
static const uint8_t kBaLiftHomePwm = 100;
static const uint8_t kBaLiftMovePwm = 120;

// Stepper hardware
static const uint8_t  kBaStepPin = PIN_ST1_STEP;
static const uint8_t  kBaDirPin = PIN_ST1_DIR;
static const uint8_t  kBaEnPin = PIN_ST1_EN;
static const uint8_t  kBaStepLimitPin = PIN_ST1_LIMIT;
static const int8_t   kBaStepHomeDir = -1;
static const uint16_t kBaStepPulseUs = 1200;  // step pulse half-period

// Stepper step interval for non-blocking stepping (must be >= 2*kBaStepPulseUs)
static const uint32_t kBaStepIntervalUs = 2500;

// DC position tolerance (encoder counts)
static const int32_t kBaDcTolerance = 5;

// Homing timeouts
static const uint32_t kBaLiftHomeTimeoutMs = 10000;
static const uint32_t kBaStepperHomeTimeoutMs = 15000;

namespace BurgerAssembly {

	// ============================================================================
	// PUBLIC API
	// ============================================================================

	/**
	 * @brief Initialize pins. Call once from setup() after main firmware init.
	 * @param liftEncoder  Pointer to the m3Encoder instance in main firmware
	 */
	void init(IEncoderCounter* liftEncoder);

	/**
	 * @brief Start the assembly sequence. No-op if already running.
	 */
	void start();

	/**
	 * @brief Request an immediate stop. Safe to call from any context.
	 */
	void stop();

	/**
	 * @brief Returns true while the sequence is running.
	 */
	bool isRunning();

	/**
	 * @brief Scheduler task — call at ~200 Hz from a registered periodic task.
	 *
	 * Advances the state machine by one tick. Never blocks.
	 */
	void task();

} // namespace BurgerAssembly

#endif // BURGER_ASSEMBLY_H