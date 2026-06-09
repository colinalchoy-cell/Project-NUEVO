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
# CONFIGURATION — paste your constants here
# ============================================================================

# TODO: paste all constants from your mini files here
# (lift times, drive PWM, stepper positions, waypoints, etc.)

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


def stop_all_motors(robot: Robot) -> None:
    # TODO: paste your stop_all_motors function here
    pass


# ============================================================================
# PHASE 1 — Home sensors, detect green light, drive forward
# ============================================================================

def home_all(robot: Robot) -> bool:
    # TODO: paste homing functions here (stepper, lift, sensors)
    pass


def detect_green_light(robot: Robot) -> bool:
    # TODO: paste green light detection function here
    # returns True when green detected
    pass


def drive_to_station(robot: Robot) -> bool:
    # TODO: paste drive forward function here
    pass


# ============================================================================
# PHASE 2 — Burger assembly sequence
# ============================================================================

def run_burger_assembly(robot: Robot) -> bool:
    # TODO: paste full burger assembly sequence here
    pass


# ============================================================================
# PHASE 3 — Pure pursuit + APF obstacle course
# ============================================================================

def run_navigation(robot: Robot) -> bool:
    # TODO: paste pure pursuit + APF path following here
    # include GPS correction logic
    pass


# ============================================================================
# PHASE 4 — Face detection, deliver to correct customer
# ============================================================================

def detect_customer(robot: Robot) -> str:
    # TODO: paste face detection function here
    # returns customer identifier (e.g. "customer_A", "customer_B")
    pass


def deliver_burger(robot: Robot, customer: str) -> bool:
    # TODO: paste delivery function here
    pass


# ============================================================================
# PHASE 5 — Drive forward, stop at stop light, continue
# ============================================================================

def wait_for_green_stoplight(robot: Robot) -> bool:
    # TODO: paste stop light detection and wait logic here
    pass


def drive_to_finish(robot: Robot) -> bool:
    # TODO: paste final drive forward function here
    pass


# ============================================================================
# MAIN FSM
# ============================================================================

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    state = "INIT"
    customer = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── INIT ──────────────────────────────────────────────────────────────
        if state == "INIT":
            show_idle_leds(robot)
            print("[FSM] INIT — homing all axes")
            if home_all(robot):
                print("[FSM] WAITING_GREEN — watching for green light")
                state = "WAITING_GREEN"
            else:
                print("[FSM] INIT failed — check homing")

        # ── WAITING FOR GREEN LIGHT ───────────────────────────────────────────
        elif state == "WAITING_GREEN":
            if detect_green_light(robot):
                print("[FSM] GREEN DETECTED — driving to station")
                state = "DRIVE_TO_STATION"

        # ── DRIVE TO BURGER STATION ───────────────────────────────────────────
        elif state == "DRIVE_TO_STATION":
            show_running_leds(robot)
            if drive_to_station(robot):
                print("[FSM] AT STATION — starting burger assembly")
                state = "BURGER_ASSEMBLY"
            else:
                print("[FSM] Drive failed")
                state = "ERROR"

        # ── BURGER ASSEMBLY ───────────────────────────────────────────────────
        elif state == "BURGER_ASSEMBLY":
            if run_burger_assembly(robot):
                print("[FSM] BURGER DONE — starting navigation")
                state = "NAVIGATION"
            else:
                print("[FSM] Burger assembly failed")
                state = "ERROR"

        # ── NAVIGATION — pure pursuit + APF ──────────────────────────────────
        elif state == "NAVIGATION":
            if run_navigation(robot):
                print("[FSM] NAVIGATION DONE — detecting customer")
                state = "DETECT_CUSTOMER"
            else:
                print("[FSM] Navigation failed")
                state = "ERROR"

        # ── DETECT CUSTOMER ───────────────────────────────────────────────────
        elif state == "DETECT_CUSTOMER":
            customer = detect_customer(robot)
            if customer:
                print(f"[FSM] Customer detected: {customer} — delivering")
                state = "DELIVER"
            else:
                print("[FSM] No customer detected — retrying")

        # ── DELIVER ───────────────────────────────────────────────────────────
        elif state == "DELIVER":
            if deliver_burger(robot, customer):
                print("[FSM] DELIVERED — waiting for stoplight")
                state = "STOPLIGHT"
            else:
                print("[FSM] Delivery failed")
                state = "ERROR"

        # ── STOPLIGHT ─────────────────────────────────────────────────────────
        elif state == "STOPLIGHT":
            if wait_for_green_stoplight(robot):
                print("[FSM] GREEN — driving to finish")
                state = "DRIVE_TO_FINISH"

        # ── DRIVE TO FINISH ───────────────────────────────────────────────────
        elif state == "DRIVE_TO_FINISH":
            if drive_to_finish(robot):
                print("[FSM] COMPLETE")
                state = "DONE"

        # ── DONE ──────────────────────────────────────────────────────────────
        elif state == "DONE":
            show_idle_leds(robot)
            print("[FSM] Mission complete — press BTN_1 to restart")
            if robot.was_button_pressed(Button.BTN_1):
                state = "INIT"

        # ── ERROR ─────────────────────────────────────────────────────────────
        elif state == "ERROR":
            show_idle_leds(robot)
            stop_all_motors(robot)
            print("[FSM] ERROR — press BTN_2 to restart")
            if robot.was_button_pressed(Button.BTN_2):
                state = "INIT"

        # ── TICK ──────────────────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()