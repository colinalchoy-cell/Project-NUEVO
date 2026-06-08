from __future__ import annotations

import select
import sys
import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
    Limit,
    Motor,
    StepMoveType,
    Stepper,
    Unit,
)
from robot.robot import FirmwareState, Robot

# ============================================================================
# CONFIGURATION — must match burger_assembly.ino exactly
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

# Drive settings
DRIVE_FORWARD_PWM         = 100
DRIVE_INITIAL_MS          = 2450
DRIVE_PATTY_MS            = 365
DRIVE_BOTTOM_BUN_MS       = 365

# Lift motor (M3) — LIM5
# IMPORTANT: config.h must have PIN_M3_LIMIT defined as PIN_LIM5
# for robot.home_motor(LIFT_MOTOR) to use the correct limit switch.
# Add this line to config.h:
#   #define PIN_M3_LIMIT    PIN_LIM5
LIFT_MOTOR                = Motor.DC_M3
LIFT_LIMIT                = Limit.LIM_5    # PIN_LIM5 = pin 50
LIFT_HOME_DIRECTION       = -1
LIFT_HOME_VELOCITY        = 80             # matches kDcHomePwm
LIFT_BACKOFF_VELOCITY     = 40             # matches max(kDcHomePwm/2, 40)
LIFT_MOVE_PWM             = 200            # matches kDcMovePwm
LIFT_TOLERANCE_TICKS      = 5              # matches Arduino abs(error) < 5
LIFT_MOVE_TIMEOUT_S       = 20.0
LIFT_HOME_TIMEOUT_S       = 10.0

# Stepper 1 — LIM1
# config.h already defines: #define PIN_ST1_LIMIT PIN_LIM1 — correct
ARM_STEPPER               = Stepper.STEPPER_1
ARM_LIMIT                 = Limit.LIM_1    # PIN_LIM1 = pin 40
ARM_HOME_DIRECTION        = -1
ARM_HOME_VELOCITY         = 300
ARM_HOME_BACKOFF_STEPS    = 50
ARM_MOVE_TIMEOUT_S        = 15.0
ARM_MOVE_VELOCITY         = 700
ARM_ACCELERATION          = 400

STOP_COMMAND              = "5"
START_COMMAND             = "g"

# ============================================================================
# GLOBAL STOP FLAG
# ============================================================================

_stop_requested: bool = False


def _check_stop() -> bool:
    """Non-blocking stdin check for '5'. Returns True if stop requested."""
    global _stop_requested
    if _stop_requested:
        return True
    try:
        if select.select([sys.stdin], [], [], 0)[0]:
            c = sys.stdin.read(1).strip()
            if c == STOP_COMMAND:
                _stop_requested = True
    except Exception:
        pass
    return _stop_requested


# ============================================================================
# ROBOT SETUP
# ============================================================================

def configure_robot(robot: Robot) -> None:
    robot.set_unit(Unit.MM)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


# ============================================================================
# STOP — mirrors Arduino '5' handler exactly
# ============================================================================

def stop_all_motors(robot: Robot) -> None:
    """
    Mirrors Arduino stop handler:
        stopLiftMotor()   → PWM=0 then disable M3
        stopDriveMotors() → PWM=0 then disable M1/M2
        disableStepper()  → stepper EN HIGH
    Order matches Arduino: lift first, then drive, then stepper.
    """
    # Zero PWM before disabling — prevents abrupt current spike on re-enable
    robot.set_motor_pwm(LIFT_MOTOR,  0)
    robot.set_motor_pwm(Motor.DC_M1, 0)
    robot.set_motor_pwm(Motor.DC_M2, 0)

    robot.disable_motor(LIFT_MOTOR)
    robot.disable_motor(Motor.DC_M1)
    robot.disable_motor(Motor.DC_M2)
    robot.step_disable(ARM_STEPPER)
    print("[STOP] All motors halted.")


# ============================================================================
# LIMIT SWITCH HELPERS
# ============================================================================

def poll_limit_release(robot: Robot, limit: Limit, timeout_s: float = 5.0) -> bool:
    """
    Poll until limit switch releases (reads False).
    Mirrors Arduino: while (isLimitTriggered(...));
    Uses polling because robot.wait_for_limit() only waits for trigger,
    not release — there is no wait_for_limit_release() in hardware.py.
    """
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if _check_stop():
            return False
        if not robot.get_limit(int(limit)):
            return True
        time.sleep(0.005)
    return False


# ============================================================================
# HOMING
# ============================================================================

def home_lift(robot: Robot) -> bool:
    """
    Mirrors Arduino homeDcMotor() including the two-reset backoff sequence.

    REQUIRES config.h to have:
        #define PIN_M3_LIMIT    PIN_LIM5

    Without that, robot.home_motor(Motor.DC_M3) will not use LIM5.
    The firmware's _wait_dc_not_homing() polls until mode leaves HOMING=4,
    so blocking=True here correctly waits for firmware homing to complete.

    After firmware homing stops at the switch, we manually:
        1. Reset encoder (first zero at contact point)
        2. Back off slowly until LIM5 releases
        3. Wait 200ms, reset encoder again (true zero at release edge)
    This matches the Arduino exactly.
    """
    print("[Lift] Homing DC lift motor (LIM5)...")

    # Firmware drives motor in direction at home_velocity until limit triggers,
    # then stops and sets mode back from HOMING. blocking=True waits for this.
    ok = robot.home_motor(
        int(LIFT_MOTOR),
        direction=LIFT_HOME_DIRECTION,
        home_velocity=LIFT_HOME_VELOCITY,
        blocking=True,
        timeout=LIFT_HOME_TIMEOUT_S,
    )

    if not ok:
        print("[Lift] Homing FAILED — timeout or limit not reached.")
        return False

    # Verify LIM5 is actually triggered (sanity check)
    if not robot.get_limit(int(LIFT_LIMIT)):
        print("[Lift] WARNING — homing completed but LIM5 not reading as triggered.")

    # First encoder reset at switch contact point
    robot.reset_motor_position(int(LIFT_MOTOR))

    # Back off slowly in opposite direction until switch releases
    # matches: setLiftPwm(-kDcHomeDirection * backoffPwm)
    robot.enable_motor(int(LIFT_MOTOR), DCMotorMode.PWM)
    robot.set_motor_pwm(int(LIFT_MOTOR), -LIFT_HOME_DIRECTION * LIFT_BACKOFF_VELOCITY)

    released = poll_limit_release(robot, LIFT_LIMIT, timeout_s=5.0)

    robot.set_motor_pwm(int(LIFT_MOTOR), 0)
    robot.disable_motor(int(LIFT_MOTOR))

    if not released:
        print("[Lift] Backoff FAILED — LIM5 did not release within timeout.")
        return False

    # matches Arduino: delay(200); m3Encoder.resetCount();
    time.sleep(0.2)
    robot.reset_motor_position(int(LIFT_MOTOR))
    print("[Lift] Homed. Encoder zeroed at switch release edge.")
    return True


def home_stepper(robot: Robot) -> bool:
    """
    Mirrors Arduino homeStepper().
    robot.step_home() sends StepHome TLV which the firmware handles using
    PIN_ST1_LIMIT (= PIN_LIM1 per config.h) — already correct, no changes needed.
    backoff_steps matches Arduino behavior of stopping just past the switch.
    """
    print("[Stepper] Homing stepper 1 (LIM1)...")

    robot.step_set_config(
        int(ARM_STEPPER),
        max_velocity=ARM_MOVE_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )

    # step_home() internally calls _wait_stepper_idle() which waits for
    # motion_state to go non-IDLE then back to IDLE — correct completion detection
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
        print("[Stepper] Homing FAILED or timed out.")
        return False

    # Verify LIM1 was actually reached
    if not robot.get_limit(int(ARM_LIMIT)):
        print("[Stepper] WARNING — homing completed but LIM1 not reading as triggered.")

    print("[Stepper] Homed. Position zeroed.")
    return True


# ============================================================================
# MOVE PRIMITIVES
# ============================================================================

def move_lift_to(robot: Robot, target: int) -> bool:
    """
    Mirrors Arduino moveDcTo().
    _wait_dc_position() polls dc_state.motors[2].position every 20ms
    and returns when abs(position - target) <= tolerance_ticks.
    tolerance_ticks=5 matches Arduino's abs(error) < 5.
    """
    if _check_stop():
        return False

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
    _wait_stepper_idle() waits for motion_state to go active then idle —
    this correctly detects move completion without false-positives.
    """
    if _check_stop():
        return False

    print(f"[Stepper] Moving to {target}")
    robot.step_set_config(
        int(ARM_STEPPER),
        max_velocity=ARM_MOVE_VELOCITY,
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
    Polls _check_stop() every 10ms during the drive — matches Arduino's
    pollStopRequest() being called inside the wait loop.
    """
    if _check_stop():
        return False

    print(f"[Drive] Forward PWM={pwm} for {duration_ms}ms")
    robot.enable_motor(int(Motor.DC_M1), DCMotorMode.PWM)
    robot.enable_motor(int(Motor.DC_M2), DCMotorMode.PWM)
    robot.set_motor_pwm(int(Motor.DC_M1),  pwm)
    robot.set_motor_pwm(int(Motor.DC_M2),  pwm)

    end = time.monotonic() + (duration_ms / 1000.0)
    while time.monotonic() < end:
        time.sleep(0.01)
        if _check_stop():
            stop_all_motors(robot)
            return False

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


def transfer_burger_to_dynamic_platform(robot: Robot) -> bool:
    """Mirrors Arduino transferBurgerToDynamicPlatform()."""
    print("\n--- Transferring burger to dynamic platform")

    if not move_lift_to(robot, DC_STATIC_PLATFORM):           return False
    if not move_stepper_to(robot, STEPPER_DYNAMIC_PLATFORM):  return False

    print("--- Transfer complete.")
    return True


# ============================================================================
# FULL ASSEMBLY SEQUENCE
# ============================================================================

def run_burger_assembly(robot: Robot) -> None:
    """
    Mirrors Arduino runBurgerAssembly() step for step.
    Sequence order matches Arduino exactly including initial drive after homing.
    """
    global _stop_requested
    _stop_requested = False

    print("\n========================================")
    print("  Starting Burger Assembly Sequence")
    print("========================================\n")

    # Step 1: Home stepper first, then lift — matches Arduino order
    if not home_stepper(robot): return
    if not home_lift(robot):    return

    # Step 2: Initial drive to first ingredient position
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_INITIAL_MS): return

    # Step 3: Pick up bottom bun
    if not pick_up_ingredient(robot, "Bottom Bun", DC_STATIC_PLATFORM): return

    # Step 4: Drive to patty — uses DRIVE_PATTY_MS (matches Arduino kDrivePattyMs)
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_PATTY_MS): return

    # Step 5: Pick up patty
    if not pick_up_ingredient(robot, "Patty", DC_STATIC_PLUS_BUN): return

    # Step 6: Drive to top bun — uses DRIVE_BOTTOM_BUN_MS (matches Arduino kDriveBottomBunMs)
    if not drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_BOTTOM_BUN_MS): return

    # Step 7: Pick up top bun
    if not pick_up_ingredient(robot, "Top Bun", DC_STATIC_PLUS_BUN_PATTY): return

    # Step 8: Transfer assembled burger to dynamic platform
    transfer_burger_to_dynamic_platform(robot)

    print("\n========================================")
    print("  Burger Assembly Complete!")
    print("========================================\n")


# ============================================================================
# ENTRY POINT
# ============================================================================

def run(robot: Robot) -> None:
    global _stop_requested
    configure_robot(robot)
    start_robot(robot)

    print("========================================")
    print("  Burger Assembly Sequence")
    print("========================================")
    print("Type 'g' to start assembly sequence.")
    print("Type '5' at any time to stop all motors.")

    while True:
        try:
            command = input("Enter command: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            stop_all_motors(robot)
            return

        if command == START_COMMAND:
            _stop_requested = False
            run_burger_assembly(robot)
        elif command == STOP_COMMAND:
            _stop_requested = True
            stop_all_motors(robot)
        elif command == "":
            continue
        else:
            print("Send 'g' to start, '5' to stop.")