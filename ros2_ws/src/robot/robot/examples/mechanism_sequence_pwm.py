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
# CONFIGURATION
# ============================================================================

# Stepper positions (steps from home)
STEPPER_TABLE             = 4500
STEPPER_DYNAMIC_PLATFORM  = 1750
STEPPER_STATIC_PLATFORM   = 0

# Drive settings
DRIVE_FORWARD_PWM         = 200
DRIVE_INITIAL_MS          = 3350
DRIVE_PATTY_MS            = 500
DRIVE_BOTTOM_BUN_MS       = 500

# Lift motor
LIFT_MOTOR                = Motor.DC_M3
LIFT_LIMIT                = Limit.LIM_5
LIFT_HOME_DIRECTION       = -1
LIFT_HOME_PWM             = 200
LIFT_BACKOFF_PWM          = 200
LIFT_MOVE_PWM             = 200
LIFT_MOVE_DOWN_PWM        = -200
LIFT_TOLERANCE_TICKS      = 5
LIFT_MOVE_TIMEOUT_S       = 20.0
LIFT_HOME_TIMEOUT_S       = 10.0

# ============================================================================
# LIFT TRAVEL TIMES (seconds) — tune these
# Each time is the travel between the two specific positions listed
# ============================================================================

LIFT_TIME_HOME_TO_ABOVE_TABLE        = 4.0   # home (0) → above table (top)
LIFT_TIME_ABOVE_TO_TABLE_HEIGHT      = 1.5   # above table → table height
LIFT_TIME_TABLE_TO_DROP_PLATFORM     = 1.5   # table height → DC_STATIC_PLATFORM drop
LIFT_TIME_TABLE_TO_DROP_BUN          = 1.0   # table height → DC_STATIC_PLUS_BUN drop
LIFT_TIME_TABLE_TO_DROP_BUN_PATTY    = 0.5   # table height → DC_STATIC_PLUS_BUN_PATTY drop
LIFT_TIME_DROP_TO_HOME               = 2.0   # any drop height → home (back down to 0)
LIFT_TIME_HOME_TO_TRANSFER           = 1.8   # home → transfer height

# Stepper
ARM_STEPPER               = Stepper.STEPPER_1
ARM_LIMIT                 = Limit.LIM_1
ARM_HOME_DIRECTION        = -1
ARM_HOME_VELOCITY         = 400
ARM_HOME_BACKOFF_STEPS    = 50
ARM_MOVE_TIMEOUT_S        = 45.0
ARM_MAX_VELOCITY          = 900
ARM_ACCELERATION          = 400

# Drive motors
DRIVE_M1_PWM_SIGN         =  1
DRIVE_M2_PWM_SIGN         = -1

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

def poll_limit_triggered(robot: Robot, limit: Limit, timeout_s: float) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if robot.get_limit(int(limit)):
            return True
        time.sleep(0.005)
    return False


def poll_limit_release(robot: Robot, limit: Limit, timeout_s: float = 5.0) -> bool:
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
    print("[Lift] Homing (LIM5)...")

    robot.enable_motor(int(LIFT_MOTOR), DCMotorMode.PWM)
    robot.set_motor_pwm(int(LIFT_MOTOR), LIFT_HOME_DIRECTION * LIFT_HOME_PWM)

    triggered = poll_limit_triggered(robot, LIFT_LIMIT, LIFT_HOME_TIMEOUT_S)

    robot.set_motor_pwm(int(LIFT_MOTOR), 0)
    robot.disable_motor(int(LIFT_MOTOR))

    if not triggered:
        print("[Lift] Homing FAILED — LIM5 not triggered within timeout.")
        return False

    robot.reset_motor_position(int(LIFT_MOTOR))

    robot.enable_motor(int(LIFT_MOTOR), DCMotorMode.PWM)
    robot.set_motor_pwm(int(LIFT_MOTOR), -LIFT_HOME_DIRECTION * LIFT_BACKOFF_PWM)

    released = poll_limit_release(robot, LIFT_LIMIT, timeout_s=5.0)

    robot.set_motor_pwm(int(LIFT_MOTOR), 0)
    robot.disable_motor(int(LIFT_MOTOR))

    if not released:
        print("[Lift] Backoff FAILED — LIM5 did not release.")
        return False

    time.sleep(0.2)
    robot.reset_motor_position(int(LIFT_MOTOR))
    print("[Lift] Homed.")
    return True


def home_stepper(robot: Robot) -> bool:
    print("[Stepper] Homing (LIM1)...")

    robot.step_set_config(
        int(ARM_STEPPER),
        max_velocity=ARM_MAX_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )
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
        print("[Stepper] Homing FAILED — check LIM1 wiring.")
        return False

    print("[Stepper] Homed.")
    return True


# ============================================================================
# MOVE PRIMITIVES
# ============================================================================

def move_lift_timed(robot: Robot, pwm: int, duration_s: float, label: str = "") -> bool:
    """Drive lift at pwm for duration_s seconds."""
    if label:
        print(f"[Lift] {label} ({duration_s}s at PWM {pwm})")
    robot.enable_motor(int(LIFT_MOTOR), DCMotorMode.PWM)
    robot.set_motor_pwm(int(LIFT_MOTOR), pwm)
    time.sleep(duration_s)
    robot.set_motor_pwm(int(LIFT_MOTOR), 0)
    robot.disable_motor(int(LIFT_MOTOR))
    return True


def move_stepper_to(robot: Robot, target: int) -> bool:
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

def pick_up_ingredient(robot: Robot, name: str, lift_time_to_drop: float) -> bool:
    """
    Full ingredient pick sequence using timed lift moves.
    lift_time_to_drop = time from table height down to the drop height for this ingredient.
    Sequence assumes lift starts at home (0) and returns to home after.
    """
    print(f"\n--- Picking up: {name}")

    # 1. Up to above table
    if not move_lift_timed(robot, LIFT_MOVE_PWM, LIFT_TIME_HOME_TO_ABOVE_TABLE, "home → above table"):
        return False

    # 2. Arm out to table
    if not move_stepper_to(robot, STEPPER_TABLE):
        return False

    # 3. Down to table height
    if not move_lift_timed(robot, LIFT_MOVE_DOWN_PWM, LIFT_TIME_ABOVE_TO_TABLE_HEIGHT, "above table → table height"):
        return False

    # 4. Arm to dynamic platform
    if not move_stepper_to(robot, STEPPER_DYNAMIC_PLATFORM):
        return False

    # 5. Down to drop height for this ingredient
    if not move_lift_timed(robot, LIFT_MOVE_DOWN_PWM, lift_time_to_drop, f"table height → {name} drop"):
        return False

    # 6. Arm back to static platform
    if not move_stepper_to(robot, STEPPER_STATIC_PLATFORM):
        return False

    # 7. Return lift to home for next move
    if not move_lift_timed(robot, LIFT_MOVE_DOWN_PWM, LIFT_TIME_DROP_TO_HOME, "drop → home"):
        return False

    print(f"--- Done: {name}")
    return True


def transfer_burger(robot: Robot) -> bool:
    """Move lift up slightly and arm to dynamic platform for final transfer."""
    print("\n--- Transferring burger to dynamic platform")

    # Lift up to transfer height
    if not move_lift_timed(robot, LIFT_MOVE_DOWN_PWM, LIFT_TIME_HOME_TO_TRANSFER, "home → transfer height"):
        return False

    # Arm to dynamic platform
    if not move_stepper_to(robot, STEPPER_DYNAMIC_PLATFORM):
        return False

    print("--- Transfer complete.")
    return True


# ============================================================================
# FULL ASSEMBLY SEQUENCE
# ============================================================================

def run_burger_assembly(robot: Robot) -> bool:
    print("\n========================================")
    print("  Starting Burger Assembly Sequence")
    print("========================================\n")

    # Step 1: Home
    if not home_stepper(robot): return False
    if not home_lift(robot):    return False

    # Step 2: Initial drive to first ingredient
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_INITIAL_MS): return False

    # Step 3: Bottom bun
    # ---------------------------------------------------------------
    # LIFT_TIME_TABLE_TO_DROP_PLATFORM — tune this for bottom bun drop height
    # ---------------------------------------------------------------
    if not pick_up_ingredient(robot, "Bottom Bun", LIFT_TIME_TABLE_TO_DROP_PLATFORM): return False

    # Step 4: Drive to patty
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_PATTY_MS): return False

    # Step 5: Patty
    # ---------------------------------------------------------------
    # LIFT_TIME_TABLE_TO_DROP_BUN — tune this for patty drop height
    # ---------------------------------------------------------------
    if not pick_up_ingredient(robot, "Patty", LIFT_TIME_TABLE_TO_DROP_BUN): return False

    # Step 6: Drive to top bun
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_BOTTOM_BUN_MS): return False

    # Step 7: Top bun
    # ---------------------------------------------------------------
    # LIFT_TIME_TABLE_TO_DROP_BUN_PATTY — tune this for top bun drop height
    # ---------------------------------------------------------------
    if not pick_up_ingredient(robot, "Top Bun", LIFT_TIME_TABLE_TO_DROP_BUN_PATTY): return False

    # Step 8: Transfer
    transfer_burger(robot)

    print("\n========================================")
    print("  Burger Assembly Complete!")
    print("========================================\n")
    return True


# ============================================================================
# FSM ENTRY POINT
# ============================================================================

def run(robot: Robot) -> None:
    configure_robot(robot)

    state  = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        if state == "INIT":
            start_robot(robot)
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