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
# CONFIGURATION — must match burger_assembly.ino
# ============================================================================

# DC lift motor (M3) encoder positions (counts from home)
DC_ABOVE_TABLE            = 8000
DC_TABLE_HEIGHT           = 5700
DC_STATIC_PLATFORM        = 820
DC_STATIC_PLUS_BUN        = 3500
DC_STATIC_PLUS_BUN_PATTY  = 5700

# Stepper positions (steps from home)
STEPPER_TABLE             = 4500
STEPPER_DYNAMIC_PLATFORM  = 1750
STEPPER_STATIC_PLATFORM   = 0

# Drive settings
DRIVE_FORWARD_PWM         = 100
DRIVE_INITIAL_MS          = 2450
DRIVE_PATTY_MS            = 365
DRIVE_BOTTOM_BUN_MS       = 365

# Lift motor (M3)
# NOTE: Add this to config.h for home_motor() to use the correct limit pin:
#   #define PIN_M3_LIMIT    PIN_LIM5
LIFT_MOTOR                = Motor.DC_M3
LIFT_LIMIT                = Limit.LIM_5
LIFT_HOME_DIRECTION       = -1
LIFT_HOME_VELOCITY        = 80
LIFT_BACKOFF_VELOCITY     = 40
LIFT_MOVE_PWM             = 200
LIFT_TOLERANCE_TICKS      = 5
LIFT_MOVE_TIMEOUT_S       = 20.0
LIFT_HOME_TIMEOUT_S       = 10.0

# Stepper 1 — LIM1 is already correct in config.h (PIN_ST1_LIMIT = PIN_LIM1)
ARM_STEPPER               = Stepper.STEPPER_1
ARM_LIMIT                 = Limit.LIM_1
ARM_HOME_DIRECTION        = -1
ARM_HOME_VELOCITY         = 300
ARM_HOME_BACKOFF_STEPS    = 50
ARM_MOVE_TIMEOUT_S        = 15.0
ARM_MAX_VELOCITY          = 700
ARM_ACCELERATION          = 400

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
# STOP
# ============================================================================

def stop_all_motors(robot: Robot) -> None:
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

def poll_limit_release(robot: Robot, limit: Limit, timeout_s: float = 5.0) -> bool:
    """Poll until limit switch releases. No wait_for_limit_release() in API."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if not robot.get_limit(int(limit)):
            return True
        time.sleep(0.005)
    return False


# ============================================================================
# HOMING — modeled on manipulation.py home_arm() pattern
# ============================================================================

def home_stepper(robot: Robot) -> bool:
    """
    Mirrors manipulation.py home_arm() exactly:
        step_enable() first
        step_home() with blocking=True
        step_disable() after
    """
    print("[Stepper] Homing stepper 1 (LIM1)...")

    # Set config BEFORE enable, same order as manipulation.py run_pick_sequence
    robot.step_set_config(
        int(ARM_STEPPER),
        max_velocity=ARM_MAX_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )

    # CRITICAL: enable before home — manipulation.py always does this
    robot.step_enable(int(ARM_STEPPER))

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
        print("[Stepper] Homing FAILED — check LIM1 wiring or home direction.")
        return False

    print("[Stepper] Homed.")
    return True


def home_lift(robot: Robot) -> bool:
    """
    Mirrors Arduino homeDcMotor() with two-reset backoff.
    Uses robot.home_motor() which sets DCMotorMode.HOMING internally
    and waits via _wait_dc_not_homing().

    REQUIRES config.h: #define PIN_M3_LIMIT PIN_LIM5
    """
    print("[Lift] Homing DC lift motor (LIM5)...")

    ok = robot.home_motor(
        int(LIFT_MOTOR),
        direction=LIFT_HOME_DIRECTION,
        home_velocity=LIFT_HOME_VELOCITY,
        blocking=True,
        timeout=LIFT_HOME_TIMEOUT_S,
    )

    if not ok:
        print("[Lift] Homing FAILED — check LIM5 wiring or PIN_M3_LIMIT in config.h.")
        return False

    # First encoder reset at switch contact point
    robot.reset_motor_position(int(LIFT_MOTOR))

    # Back off slowly until switch releases — mirrors Arduino backoff loop
    robot.enable_motor(int(LIFT_MOTOR), DCMotorMode.PWM)
    robot.set_motor_pwm(int(LIFT_MOTOR), -LIFT_HOME_DIRECTION * LIFT_BACKOFF_VELOCITY)

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


# ============================================================================
# MOVE PRIMITIVES — modeled on manipulation.py run_pick_sequence() pattern
# ============================================================================

def move_lift_to(robot: Robot, target: int) -> bool:
    """
    Mirrors manipulation.py set_motor_position() pattern:
        enable POSITION mode
        set_motor_position() with blocking
        disable after arrival
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
    Mirrors manipulation.py step_move() pattern:
        step_set_config() before each move
        step_enable()
        step_move() ABSOLUTE with blocking
        step_disable() after
    """
    print(f"[Stepper] Moving to {target}")

    # Set config before every move — same as manipulation.py
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
    """Drive both wheels forward for duration_ms milliseconds."""
    print(f"[Drive] Forward PWM={pwm} for {duration_ms}ms")
    robot.enable_motor(int(Motor.DC_M1), DCMotorMode.PWM)
    robot.enable_motor(int(Motor.DC_M2), DCMotorMode.PWM)
    robot.set_motor_pwm(int(Motor.DC_M1),  pwm)
    robot.set_motor_pwm(int(Motor.DC_M2),  pwm)

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
    print("\n--- Transferring burger to dynamic platform")

    if not move_lift_to(robot, DC_STATIC_PLATFORM):           return False
    if not move_stepper_to(robot, STEPPER_DYNAMIC_PLATFORM):  return False

    print("--- Transfer complete.")
    return True


# ============================================================================
# FULL ASSEMBLY SEQUENCE
# ============================================================================

def run_burger_assembly(robot: Robot) -> bool:
    print("\n========================================")
    print("  Starting Burger Assembly Sequence")
    print("========================================\n")

    if not home_stepper(robot): return False
    if not home_lift(robot):    return False

    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_INITIAL_MS): return False

    if not pick_up_ingredient(robot, "Bottom Bun", DC_STATIC_PLATFORM): return False
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_PATTY_MS):     return False

    if not pick_up_ingredient(robot, "Patty", DC_STATIC_PLUS_BUN):      return False
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_BOTTOM_BUN_MS): return False

    if not pick_up_ingredient(robot, "Top Bun", DC_STATIC_PLUS_BUN_PATTY): return False

    transfer_burger(robot)

    print("\n========================================")
    print("  Burger Assembly Complete!")
    print("========================================\n")
    return True


# ============================================================================
# FSM ENTRY POINT — matches manipulation.py run() structure exactly
# ============================================================================

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            # Home both axes at startup, same as manipulation.py homes arm
            # and restores lift in INIT before entering IDLE
            home_stepper(robot)
            home_lift(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to run burger assembly")
            print("[FSM] Press BTN_2 to emergency stop")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            # BTN_1 starts — mirrors manipulation.py was_button_pressed(BTN_1)
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] RUNNING")
                state = "RUNNING"
            # BTN_2 stops from idle — re-home
            if robot.was_button_pressed(Button.BTN_2):
                print("[FSM] BTN_2 in IDLE — re-homing both axes")
                home_stepper(robot)
                home_lift(robot)

        elif state == "RUNNING":
            ok = run_burger_assembly(robot)
            show_idle_leds(robot)
            stop_all_motors(robot)
            if ok:
                print("[FSM] IDLE — assembly complete. Press BTN_1 to run again.")
            else:
                print("[FSM] IDLE — assembly stopped. Press BTN_2 to re-home, BTN_1 to retry.")
            state = "IDLE"

        # FSM tick rate — matches manipulation.py
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()