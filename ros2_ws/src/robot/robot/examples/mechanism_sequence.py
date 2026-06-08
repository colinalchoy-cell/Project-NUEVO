from __future__ import annotations

import time

from robot.hardware_map import DCMotorMode, Motor, StepMoveType, Stepper, Unit
from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Burger assembly sequence configuration from firmware test_mechanism_sequence
# ---------------------------------------------------------------------------

DC_ABOVE_TABLE = 8000
DC_TABLE_HEIGHT = 5700
DC_STATIC_PLATFORM = 820
DC_STATIC_PLUS_BUN = 3500
DC_STATIC_PLUS_BUN_PATTY = 5700

STEPPER_TABLE = 4500
STEPPER_DYNAMIC_PLATFORM = 1750
STEPPER_STATIC_PLATFORM = 0

DRIVE_FORWARD_PWM = 100
DRIVE_INITIAL_MS = 2450
DRIVE_PATTY_MS = 365
DRIVE_BOTTOM_BUN_MS = 365

LIFT_MOTOR = Motor.DC_M3
LIFT_HOME_DIRECTION = -1
LIFT_HOME_VELOCITY = 100
LIFT_MOVE_PWM = 200
LIFT_TOLERANCE_TICKS = 20
LIFT_MOVE_TIMEOUT_S = 20.0

ARM_STEPPER = Stepper.STEPPER_1
ARM_HOME_DIRECTION = -1
ARM_HOME_VELOCITY = 300
ARM_HOME_BACKOFF_STEPS = 50
ARM_MOVE_TIMEOUT_S = 15.0
ARM_MOVE_VELOCITY = 700
ARM_ACCELERATION = 400

STOP_COMMAND = "5"
START_COMMAND = "g"


def configure_robot(robot: Robot) -> None:
    robot.set_unit(Unit.MM)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def stop_all_motors(robot: Robot) -> None:
    robot.disable_motor(Motor.DC_M1)
    robot.disable_motor(Motor.DC_M2)
    robot.disable_motor(LIFT_MOTOR)
    robot.step_disable(ARM_STEPPER)
    print("[STOP] All motors halted.")


def set_uptime_sleep(robot: Robot, duration_s: float) -> None:
    # Allow ROS callbacks to run while sleeping.
    end = time.monotonic() + duration_s
    while time.monotonic() < end:
        time.sleep(0.01)


def home_lift(robot: Robot) -> bool:
    print("[Lift] Homing DC lift motor")
    ok = robot.home_motor(
        LIFT_MOTOR,
        direction=LIFT_HOME_DIRECTION,
        home_velocity=LIFT_HOME_VELOCITY,
        blocking=True,
        timeout=10.0,
    )
    robot.disable_motor(LIFT_MOTOR)
    if ok:
        robot.reset_motor_position(LIFT_MOTOR)
        print("[Lift] Homed and encoder zeroed.")
    else:
        print("[Lift] Homing failed or timed out.")
    return ok


def home_stepper(robot: Robot) -> bool:
    print("[Stepper] Homing stepper 1")
    robot.step_set_config(
        ARM_STEPPER,
        max_velocity=ARM_MOVE_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )
    ok = robot.step_home(
        ARM_STEPPER,
        direction=ARM_HOME_DIRECTION,
        home_velocity=ARM_HOME_VELOCITY,
        backoff_steps=ARM_HOME_BACKOFF_STEPS,
        blocking=True,
        timeout=ARM_MOVE_TIMEOUT_S,
    )
    robot.step_disable(ARM_STEPPER)
    if ok:
        print("[Stepper] Homed.")
    else:
        print("[Stepper] Homing failed or timed out.")
    return ok


def move_lift_to(robot: Robot, target: int) -> bool:
    print(f"[Lift] Moving to {target}")
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    ok = robot.set_motor_position(
        LIFT_MOTOR,
        target,
        max_vel_ticks=LIFT_MOVE_PWM,
        tolerance_ticks=LIFT_TOLERANCE_TICKS,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    )
    robot.disable_motor(LIFT_MOTOR)
    if ok:
        print(f"[Lift] Arrived at {target}")
    else:
        print("[Lift] Failed to reach target")
    return ok


def move_stepper_to(robot: Robot, target: int) -> bool:
    print(f"[Stepper] Moving to {target}")
    robot.step_set_config(
        ARM_STEPPER,
        max_velocity=ARM_MOVE_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )
    robot.step_enable(ARM_STEPPER)
    ok = robot.step_move(
        ARM_STEPPER,
        steps=target,
        move_type=StepMoveType.ABSOLUTE,
        blocking=True,
        timeout=ARM_MOVE_TIMEOUT_S,
    )
    robot.step_disable(ARM_STEPPER)
    if ok:
        print(f"[Stepper] Arrived at {target}")
    else:
        print("[Stepper] Failed to reach target")
    return ok


def drive_forward(robot: Robot, pwm: int, duration_ms: int) -> bool:
    print(f"[Drive] Forward PWM={pwm} for {duration_ms}ms")
    for motor in (Motor.DC_M1, Motor.DC_M2):
        robot.enable_motor(motor, DCMotorMode.PWM)
        robot.set_motor_pwm(motor, pwm)

    stop_time = time.monotonic() + (duration_ms / 1000.0)
    while time.monotonic() < stop_time:
        time.sleep(0.01)

    stop_all_motors(robot)
    print("[Drive] Stopped")
    return True


def pick_up_ingredient(robot: Robot, name: str, dc_drop_height: int) -> bool:
    print(f"\n--- Picking up: {name}")
    if not move_lift_to(robot, DC_ABOVE_TABLE):
        return False
    if not move_stepper_to(robot, STEPPER_TABLE):
        return False
    if not move_lift_to(robot, DC_TABLE_HEIGHT):
        return False
    if not move_stepper_to(robot, STEPPER_DYNAMIC_PLATFORM):
        return False
    if not move_lift_to(robot, dc_drop_height):
        return False
    if not move_stepper_to(robot, STEPPER_STATIC_PLATFORM):
        return False
    print(f"--- Done: {name}")
    return True


def transfer_burger_to_dynamic_platform(robot: Robot) -> bool:
    print("\n--- Transferring burger to dynamic platform")
    if not move_lift_to(robot, DC_STATIC_PLATFORM):
        return False
    if not move_stepper_to(robot, STEPPER_DYNAMIC_PLATFORM):
        return False
    print("--- Transfer complete.")
    return True


def run_burger_assembly(robot: Robot) -> None:
    print("\n========================================")
    print("  Starting Burger Assembly Sequence")
    print("========================================\n")

    if not home_stepper(robot):
        return
    if not home_lift(robot):
        return

    drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_INITIAL_MS)

    if not pick_up_ingredient(robot, "Bottom Bun", DC_STATIC_PLATFORM):
        return

    drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_PATTY_MS)

    if not pick_up_ingredient(robot, "Patty", DC_STATIC_PLUS_BUN):
        return

    drive_forward(robot, DRIVE_FORWARD_PWM, DRIVE_BOTTOM_BUN_MS)

    if not pick_up_ingredient(robot, "Top Bun", DC_STATIC_PLUS_BUN_PATTY):
        return

    transfer_burger_to_dynamic_platform(robot)

    print("\n========================================")
    print("  Burger Assembly Complete!")
    print("========================================\n")


def run(robot: Robot) -> None:
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
            print("\nExiting burger assembly sequence.")
            stop_all_motors(robot)
            return

        if command == START_COMMAND:
            run_burger_assembly(robot)
        elif command == STOP_COMMAND:
            stop_all_motors(robot)
        elif command == "":
            continue
        else:
            print("Send 'g' to start, '5' to stop.")
