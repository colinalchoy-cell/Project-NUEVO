from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    LED,
    Limit,
    Motor,
    POSITION_UNIT,
    StepMoveType,
    Stepper,
)
from robot.robot import FirmwareState, Robot

# ============================================================================
# CONFIGURATION — matches burger_assembly.ino exactly
# ============================================================================

# DC lift motor encoder positions (counts from home)
DC_ABOVE_TABLE            = 8000
DC_TABLE_HEIGHT           = 5700
DC_STATIC_PLATFORM        = 820
DC_STATIC_PLUS_BUN        = 3500
DC_STATIC_PLUS_BUN_PATTY  = 5700

# Stepper positions (steps from home)
STEPPER_TABLE             = 4500
STEPPER_DYNAMIC_PLATFORM  = 1750
STEPPER_STATIC_PLATFORM   = 0

# Drive settings — matches kDriveForwardPwm, kDriveInitialMs, etc.
DRIVE_FORWARD_PWM         = 100
DRIVE_INITIAL_MS          = 2450
DRIVE_PATTY_MS            = 365
DRIVE_BOTTOM_BUN_MS       = 365

# Lift motor (M3) — matches kDcHomePwm=80, kDcMovePwm=200, kDcHomeDirection=-1
# PIN_M3_LIMIT is NOT defined in config.h so home_motor() won't work.
# We home manually using PWM + LIM5 polling, exactly like the Arduino.
LIFT_MOTOR                = Motor.DC_M3
LIFT_LIMIT                = Limit.LIM_5
LIFT_HOME_DIRECTION       = -1             # matches kDcHomeDirection
LIFT_HOME_PWM             = 200             # matches kDcHomePwm
LIFT_BACKOFF_PWM          = 200             # matches max(kDcHomePwm/2, 40)
LIFT_MOVE_PWM             = 200            # matches kDcMovePwm
LIFT_TOLERANCE_TICKS      = 5              # matches abs(error) < 5
LIFT_MOVE_TIMEOUT_S       = 20.0
LIFT_HOME_TIMEOUT_S       = 10.0           # matches 10000ms

# Stepper 1 — PIN_ST1_LIMIT = PIN_LIM1 already correct in config.h
ARM_STEPPER               = Stepper.STEPPER_1
ARM_LIMIT                 = Limit.LIM_1
ARM_HOME_DIRECTION        = -1             # matches kStepperHomeDirection
ARM_HOME_VELOCITY         = 300
ARM_HOME_BACKOFF_STEPS    = 50
ARM_MOVE_TIMEOUT_S        = 15.0           # matches 15000ms
ARM_MAX_VELOCITY          = 700
ARM_ACCELERATION          = 400

# Drive motors — M2 is physically reversed so needs negative PWM
# matches DC_MOTOR_2_DIR_INVERTED = 1 in config.h
DRIVE_M1_PWM_SIGN         =  1             # M1 forward = positive PWM
DRIVE_M2_PWM_SIGN         = -1             # M2 physically reversed

# ============================================================================
# SETUP
# ============================================================================

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ============================================================================
# STOP — mirrors Arduino '5' handler exactly
# ============================================================================

def stop_all_motors(robot: Robot) -> None:
    """
    Mirrors Arduino:
        stopLiftMotor()   → PWM=0, disable M3
        stopDriveMotors() → PWM=0, disable M1/M2
        disableStepper()  → EN HIGH
    """
    robot.set_motor_pwm(int(LIFT_MOTOR),  0)
    robot.set_motor_pwm(int(Motor.DC_M1), 0)
    robot.set_motor_pwm(int(Motor.DC_M2), 0)
    robot.disable_motor(int(LIFT_MOTOR))
    robot.disable_motor(int(Motor.DC_M1))
    robot.disable_motor(int(Motor.DC_M2))
    robot.step_disable(int(ARM_STEPPER))
    print("[STOP] All motors halted.")


# ============================================================================
# LIMIT SWITCH HELPERS
# ============================================================================

def poll_limit_triggered(robot: Robot, limit: Limit, timeout_s: float) -> bool:
    """
    Poll until limit switch triggers or timeout.
    Mirrors Arduino: while (...) { if (isLimitTriggered(...)) break; }
    """
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if robot.get_limit(int(limit)):
            return True
        time.sleep(0.005)
    return False


def poll_limit_release(robot: Robot, limit: Limit, timeout_s: float = 5.0) -> bool:
    """
    Poll until limit switch releases.
    Mirrors Arduino: while (isLimitTriggered(...));
    """
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if not robot.get_limit(int(limit)):
            return True
        time.sleep(0.005)
    return False


# ============================================================================
# HOMING
# ============================================================================

def home_lift(robot: Robot) -> bool:
    """
    Manually homes lift using PWM + LIM5 polling.
    Mirrors Arduino homeDcMotor() exactly including two-reset backoff.

    Does NOT use robot.home_motor() because PIN_M3_LIMIT is not defined
    in config.h — the firmware would never trigger on LIM5 for M3.
    """
    print("[Lift] Homing (LIM5)...")

    # Drive toward home at homing PWM — mirrors setLiftPwm(kDcHomeDirection * kDcHomePwm)
    robot.enable_motor(int(LIFT_MOTOR), DCMotorMode.PWM)
    robot.set_motor_pwm(int(LIFT_MOTOR), LIFT_HOME_DIRECTION * LIFT_HOME_PWM)

    # Wait for LIM5 to trigger — mirrors Arduino while loop with isLimitTriggered
    triggered = poll_limit_triggered(robot, LIFT_LIMIT, LIFT_HOME_TIMEOUT_S)

    robot.set_motor_pwm(int(LIFT_MOTOR), 0)
    robot.disable_motor(int(LIFT_MOTOR))

    if not triggered:
        print("[Lift] Homing FAILED — LIM5 not triggered within timeout.")
        return False

    # First encoder reset at contact point — mirrors m3Encoder.resetCount()
    robot.reset_motor_position(int(LIFT_MOTOR))

    # Back off slowly until switch releases
    # mirrors: setLiftPwm(-kDcHomeDirection * backoffPwm)
    robot.enable_motor(int(LIFT_MOTOR), DCMotorMode.PWM)
    robot.set_motor_pwm(int(LIFT_MOTOR), -LIFT_HOME_DIRECTION * LIFT_BACKOFF_PWM)

    released = poll_limit_release(robot, LIFT_LIMIT, timeout_s=5.0)

    robot.set_motor_pwm(int(LIFT_MOTOR), 0)
    robot.disable_motor(int(LIFT_MOTOR))

    if not released:
        print("[Lift] Backoff FAILED — LIM5 did not release.")
        return False

    # mirrors Arduino: delay(200); m3Encoder.resetCount()
    time.sleep(0.2)
    robot.reset_motor_position(int(LIFT_MOTOR))
    print("[Lift] Homed. Encoder zeroed at switch release edge.")
    return True


def home_stepper(robot: Robot) -> bool:
    """
    Mirrors Arduino homeStepper() — step_enable() first, then step_home().
    PIN_ST1_LIMIT = PIN_LIM1 is already defined in config.h so this works correctly.
    """
    print("[Stepper] Homing (LIM1)...")

    robot.step_set_config(
        int(ARM_STEPPER),
        max_velocity=ARM_MAX_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )
    robot.step_enable(int(ARM_STEPPER))   # CRITICAL — must enable before home

    ok = robot.step_home(
        int(ARM_STEPPER),
        direction=ARM_HOME_DIRECTION,
        home_velocity=ARM_HOME_VELOCITY,
        backoff_steps=ARM_HOME_BACKOFF_STEPS,
        blocking=True,
        timeout=ARM_MOVE_TIMEOUT_S,
    )

    robot.step_disable(int(ARM_STEPPER))

    if not ok:
        print("[Stepper] Homing FAILED — check LIM1 wiring.")
        return False

    print("[Stepper] Homed.")
    return True


# ============================================================================
# MOVE PRIMITIVES
# ============================================================================

def move_lift_to(robot: Robot, target: int) -> bool:
    """
    Mirrors Arduino moveDcTo():
        drive at kDcMovePwm toward target
        stop when abs(error) < 5
    Uses POSITION mode which polls encoder internally via _wait_dc_position().
    tolerance_ticks=5 matches Arduino's abs(error) < 5.
    max_vel_ticks=LIFT_MOVE_PWM matches kDcMovePwm=200.
    """
    print(f"[Lift] Moving to {target}")

    robot.enable_motor(int(LIFT_MOTOR), DCMotorMode.POSITION)
    ok = robot.set_motor_position(
        int(LIFT_MOTOR),
        target,
        max_vel_ticks=LIFT_MOVE_PWM,
        tolerance_ticks=LIFT_TOLERANCE_TICKS,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    )
    robot.disable_motor(int(LIFT_MOTOR))

    if not ok:
        print(f"[Lift] FAILED to reach {target}")
        return False

    print(f"[Lift] Arrived at {target}")
    return True


def move_stepper_to(robot: Robot, target: int) -> bool:
    """
    Mirrors Arduino moveStepperTo() with absolute positioning.
    step_enable() before, step_disable() after — same as Arduino.
    """
    print(f"[Stepper] Moving to {target}")

    robot.step_set_config(
        int(ARM_STEPPER),
        max_velocity=ARM_MAX_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )
    robot.step_enable(int(ARM_STEPPER))

    ok = robot.step_move(
        int(ARM_STEPPER),
        steps=target,
        move_type=StepMoveType.ABSOLUTE,
        blocking=True,
        timeout=ARM_MOVE_TIMEOUT_S,
    )
    robot.step_disable(int(ARM_STEPPER))

    if not ok:
        print(f"[Stepper] FAILED to reach {target}")
        return False

    print(f"[Stepper] Arrived at {target}")
    return True


def drive_forward(robot: Robot, pwm: int, duration_ms: int) -> bool:
    """
    Mirrors Arduino driveForward().
    M2 PWM is negated to match DC_MOTOR_2_DIR_INVERTED=1 in config.h.
    """
    print(f"[Drive] Forward PWM={pwm} for {duration_ms}ms")

    robot.enable_motor(int(Motor.DC_M1), DCMotorMode.PWM)
    robot.enable_motor(int(Motor.DC_M2), DCMotorMode.PWM)
    robot.set_motor_pwm(int(Motor.DC_M1), DRIVE_M1_PWM_SIGN * pwm)
    robot.set_motor_pwm(int(Motor.DC_M2), DRIVE_M2_PWM_SIGN * pwm)

    end = time.monotonic() + (duration_ms / 1000.0)
    while time.monotonic() < end:
        time.sleep(0.01)

    stop_all_motors(robot)
    print("[Drive] Stopped.")
    return True


# ============================================================================
# BURGER ASSEMBLY STEPS
# ============================================================================

def pick_up_ingredient(robot: Robot, name: str, dc_drop_height: int) -> bool:
    """Mirrors Arduino pickUpIngredient() step for step."""
    print(f"\n--- Picking up: {name}")

    if not move_lift_to(robot, DC_ABOVE_TABLE):              return False
    if not move_stepper_to(robot, STEPPER_TABLE):             return False
    if not move_lift_to(robot, DC_TABLE_HEIGHT):              return False
    if not move_stepper_to(robot, STEPPER_DYNAMIC_PLATFORM):  return False
    if not move_lift_to(robot, dc_drop_height):               return False
    if not move_stepper_to(robot, STEPPER_STATIC_PLATFORM):   return False

    print(f"--- Done: {name}")
    return True


def transfer_burger(robot: Robot) -> bool:
    """Mirrors Arduino transferBurgerToDynamicPlatform()."""
    print("\n--- Transferring burger to dynamic platform")

    if not move_lift_to(robot, DC_STATIC_PLATFORM):           return False
    if not move_stepper_to(robot, STEPPER_DYNAMIC_PLATFORM):  return False

    print("--- Transfer complete.")
    return True


# ============================================================================
# FULL ASSEMBLY SEQUENCE
# ============================================================================

def run_burger_assembly(robot: Robot) -> bool:
    """Mirrors Arduino runBurgerAssembly() step for step."""
    print("\n========================================")
    print("  Starting Burger Assembly Sequence")
    print("========================================\n")

    # Step 1: Home stepper first, then lift — matches Arduino order
    if not home_stepper(robot): return False
    if not home_lift(robot):    return False

    # Step 2: Initial drive to first ingredient
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_INITIAL_MS): return False

    # Step 3: Bottom bun
    if not pick_up_ingredient(robot, "Bottom Bun", DC_STATIC_PLATFORM): return False

    # Step 4: Drive to patty
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_PATTY_MS): return False

    # Step 5: Patty
    if not pick_up_ingredient(robot, "Patty", DC_STATIC_PLUS_BUN): return False

    # Step 6: Drive to top bun
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_BOTTOM_BUN_MS): return False

    # Step 7: Top bun
    if not pick_up_ingredient(robot, "Top Bun", DC_STATIC_PLUS_BUN_PATTY): return False

    # Step 8: Transfer
    transfer_burger(robot)

    print("\n========================================")
    print("  Burger Assembly Complete!")
    print("========================================\n")
    return True


# ============================================================================
# FSM ENTRY POINT — matches manipulation.py run() structure
# ============================================================================

def run(robot: Robot) -> None:
    configure_robot(robot)

    state  = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        if state == "INIT":
            start_robot(robot)
            home_stepper(robot)
            home_lift(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start burger assembly")
            print("[FSM] Press BTN_2 to re-home axes")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] RUNNING")
                state = "RUNNING"
            if robot.was_button_pressed(Button.BTN_2):
                print("[FSM] Re-homing...")
                home_stepper(robot)
                home_lift(robot)
                print("[FSM] IDLE")

        elif state == "RUNNING":
            ok = run_burger_assembly(robot)
            stop_all_motors(robot)
            show_idle_leds(robot)
            if ok:
                print("[FSM] IDLE — assembly complete. BTN_1 to run again.")
            else:
                print("[FSM] IDLE — assembly stopped. BTN_2 to re-home, BTN_1 to retry.")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()