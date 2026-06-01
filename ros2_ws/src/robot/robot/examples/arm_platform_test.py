"""
arm_platform_test.py — homing and hard-coded arm platform test
==============================================================

This example runs a simple arm/platform test using:

1. DC motor M3 for vertical platform lift
2. Stepper 1 for horizontal platform extension/retraction
3. Limit switches for homing both actuators

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/arm_platform_test.py main.py
    ros2 run robot robot

If you want to keep it separate, import the run() function from this
module in your own entrypoint.

WHAT THIS DOES
--------------
- Homes the lift with `robot.home_motor()` using limit switches
- Zeroes the lift encoder after homing
- Homes the horizontal arm stepper with `robot.step_home()`
- Uses hard-coded travel distances to move the lift and arm
- Lowers the platform over the target pieces and then retracts

SAFETY
------
- Adjust the constant values below to match your physical setup.
- Verify limit switches and motion directions before running.
"""

from __future__ import annotations

import time

from robot.hardware_map import Button, DCMotorMode, LED, Motor, StepMoveType, Stepper, Unit
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Hard-coded arm/platform geometry for this test
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM

LIFT_MOTOR = Motor.DC_M3
LIFT_HOME_DIRECTION = -1
LIFT_HOME_VELOCITY = 150
LIFT_START_TICKS = 1800
LIFT_TARGET_TICKS = 600
LIFT_MAX_VEL_TICKS = 450
LIFT_TOLERANCE_TICKS = 20
LIFT_MOVE_TIMEOUT_S = 12.0

ARM_STEPPER = Stepper.STEPPER_1
ARM_EXTEND_STEPS = 2200
ARM_RETRACT_STEPS = -2200
ARM_MAX_VELOCITY = 800
ARM_ACCELERATION = 400
ARM_HOME_VELOCITY = 300
ARM_HOME_BACKOFF_STEPS = 50
ARM_MOVE_TIMEOUT_S = 12.0


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


def home_lift(robot: Robot) -> bool:
    print("[ARM TEST] Homing lift motor M3")
    ok = robot.home_motor(
        LIFT_MOTOR,
        direction=LIFT_HOME_DIRECTION,
        home_velocity=LIFT_HOME_VELOCITY,
        blocking=True,
        timeout=15.0,
    )
    if not ok:
        print("[WARN] Lift homing failed")
        return False

    robot.reset_motor_position(LIFT_MOTOR)
    print("[ARM TEST] Lift homed and zeroed")
    return True


def home_arm(robot: Robot) -> bool:
    print("[ARM TEST] Homing horizontal arm stepper")
    robot.step_set_config(
        ARM_STEPPER,
        max_velocity=ARM_MAX_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )
    ok = robot.step_home(
        ARM_STEPPER,
        direction=-1,
        home_velocity=ARM_HOME_VELOCITY,
        backoff_steps=ARM_HOME_BACKOFF_STEPS,
        blocking=True,
        timeout=15.0,
    )
    robot.step_disable(ARM_STEPPER)
    if not ok:
        print("[WARN] Stepper homing failed")
        return False

    print("[ARM TEST] Stepper homed")
    return True


def move_lift(robot: Robot, target_ticks: int, label: str) -> bool:
    print(f"[ARM TEST] Moving lift to {label} ({target_ticks} ticks)")
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    ok = robot.set_motor_position(
        LIFT_MOTOR,
        target_ticks,
        max_vel_ticks=LIFT_MAX_VEL_TICKS,
        tolerance_ticks=LIFT_TOLERANCE_TICKS,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    )
    robot.disable_motor(LIFT_MOTOR)

    if not ok:
        print(f"[WARN] Lift failed to reach {label}")
    return ok


def move_arm(robot: Robot, steps: int, label: str) -> bool:
    print(f"[ARM TEST] Moving arm {label} ({steps} steps)")
    robot.step_enable(ARM_STEPPER)
    ok = robot.step_move(
        ARM_STEPPER,
        steps=steps,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=ARM_MOVE_TIMEOUT_S,
    )
    robot.step_disable(ARM_STEPPER)

    if not ok:
        print(f"[WARN] Arm stepper failed during {label}")
    return ok


def run_arm_sequence(robot: Robot) -> None:
    if not home_lift(robot):
        return

    if not home_arm(robot):
        return

    if not move_lift(robot, LIFT_START_TICKS, "starting height"):
        return

    if not move_arm(robot, ARM_EXTEND_STEPS, "extend over target"):
        return

    time.sleep(0.3)

    if not move_lift(robot, LIFT_TARGET_TICKS, "target platform height"):
        return

    time.sleep(0.3)

    if not move_arm(robot, ARM_RETRACT_STEPS, "retract horizontally"):
        return

    if not move_lift(robot, LIFT_START_TICKS, "return to start height"):
        return

    print("[ARM TEST] Sequence complete")


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    print("[ARM TEST] Ready. Press BTN_1 to run the arm/stepper test.")
    print("[ARM TEST] Press BTN_2 to exit.")

    while True:
        show_idle_leds(robot)
        if robot.get_button(Button.BTN_1):
            show_running_leds(robot)
            run_arm_sequence(robot)
            time.sleep(0.5)

        if robot.get_button(Button.BTN_2):
            print("[ARM TEST] Exiting robot run loop")
            return

        time.sleep(0.1)
