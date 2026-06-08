/**
 * @file SafetyManager.h
 * @brief Safety fault detector for the 100 Hz soft safety task
 *
 * SafetyManager::check() detects heartbeat and battery faults, then delegates
 * all transition/shutdown policy to SystemManager.
 *
 * Fault table:
 *
 *   Condition                       Active in state(s)   Response
 *   ──────────────────────────────  ───────────────────  ───────────────────────────────
 *   Heartbeat lost (RPi timeout)    RUNNING only          triggerSafetyFaultFromIsr(ERROR)
 *   Battery lost / undervoltage     RUNNING only*         triggerSafetyFaultFromIsr(ERROR)
 *   Battery voltage > VBAT_OVERV    RUNNING only*         triggerSafetyFaultFromIsr(ERROR)
 *
 *   * Power faults are armed by SystemManager after battery power has been
 *     observed during the current RUNNING session. RUNNING without battery is
 *     allowed, but actuator-enable commands remain ineffective.
 *
 * Recovery:
 *   Only SYS_CMD_RESET (from RPi) can transition ERROR → IDLE.
 *   The underlying fault must also be resolved (battery OK, RPi reconnected).
 *
 * Note: check() is a no-op when state is INIT, ERROR, or ESTOP — already safe.
 */

#ifndef SAFETYMANAGER_H
#define SAFETYMANAGER_H

#include <Arduino.h>
#include <stdint.h>

class SafetyManager {
public:
    /**
     * @brief Run all periodic safety checks.
     *
     * Call from the 100 Hz soft safety task. Executes in O(1) time on the
     * common (no-fault) path.
     *
     * On any fault: delegates to SystemManager, which owns the shutdown and
     * ERROR-state transition policy.
     */
    static void check();
};

#endif // SAFETYMANAGER_H
