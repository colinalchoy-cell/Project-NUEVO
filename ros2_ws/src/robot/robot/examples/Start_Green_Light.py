from __future__ import annotations

import time

from robot.hardware_map import DEFAULT_FSM_HZ, LED, POSITION_UNIT
from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

LED_BRIGHTNESS              = 255
LIGHT_HOLD_SEC              = 2.0
VISION_STALE_SEC            = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50

# Turning — slow left turn while scanning for light
TURN_LEFT_PWM               = 60    # tune this — slow enough to detect reliably

# Forward drive after green
DRIVE_DISTANCE_MM           = 500.0
DRIVE_VELOCITY_MM_S         = 150.0
DRIVE_TOLERANCE_MM          = 15.0

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def show_traffic_light_color(robot: Robot, color: str) -> None:
    if color == "red":
        robot.set_led(LED.RED, LED_BRIGHTNESS)
        robot.set_led(LED.GREEN, 0)
    elif color == "green":
        robot.set_led(LED.RED, 0)
        robot.set_led(LED.GREEN, LED_BRIGHTNESS)


def find_traffic_light_color(robot: Robot) -> str | None:
    """Return the best recent red/green traffic-light result, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color      = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue
        attributes      = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color           = color_attribute.get("value")
        if color not in ("red", "green"):
            continue
        if confidence > best_confidence:
            best_confidence = confidence
            best_color      = str(color)

    return best_color


def start_turn_left(robot: Robot) -> None:
    """Slow left turn in place — left motor backward, right motor forward."""
    from robot.hardware_map import DCMotorMode, Motor
    robot.enable_motor(Motor.DC_M1, DCMotorMode.PWM)
    robot.enable_motor(Motor.DC_M2, DCMotorMode.PWM)
    robot.set_motor_pwm(int(Motor.DC_M1), -TURN_LEFT_PWM)   # left backward
    robot.set_motor_pwm(int(Motor.DC_M2),  TURN_LEFT_PWM)   # right forward


def stop_turn(robot: Robot) -> None:
    from robot.hardware_map import DCMotorMode, Motor
    robot.set_motor_pwm(int(Motor.DC_M1), 0)
    robot.set_motor_pwm(int(Motor.DC_M2), 0)
    robot.disable_motor(Motor.DC_M1)
    robot.disable_motor(Motor.DC_M2)


# ---------------------------------------------------------------------------
# run() — entry point
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state         = "INIT"
    motion_handle = None

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            dim_all_leds(robot)
            print("[FSM] SCANNING — turning left slowly until red light found")
            start_turn_left(robot)
            state = "SCANNING_FOR_RED"

        # ── SCANNING FOR RED ─────────────────────────────────────────────────
        # Robot turns left slowly until a red light is detected
        elif state == "SCANNING_FOR_RED":
            color = find_traffic_light_color(robot)

            if color == "red":
                print("[FSM] RED detected — stopping turn, waiting for green")
                stop_turn(robot)
                show_traffic_light_color(robot, "red")
                state = "WAITING_FOR_GREEN"

        # ── WAITING FOR GREEN ────────────────────────────────────────────────
        # Robot is stopped, watching for light to turn green
        elif state == "WAITING_FOR_GREEN":
            color = find_traffic_light_color(robot)

            if color == "green":
                print("[FSM] GREEN detected — driving forward")
                show_traffic_light_color(robot, "green")
                motion_handle = robot.move_forward(
                    distance=DRIVE_DISTANCE_MM,
                    velocity=DRIVE_VELOCITY_MM_S,
                    tolerance=DRIVE_TOLERANCE_MM,
                    blocking=False,
                )
                state = "MOVING_FORWARD"

            elif color == "red":
                # Still red — keep LED on
                show_traffic_light_color(robot, "red")

            else:
                # Light lost — dim LEDs but stay waiting
                dim_all_leds(robot)

        # ── MOVING FORWARD ───────────────────────────────────────────────────
        elif state == "MOVING_FORWARD":
            if motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                robot.stop()
                dim_all_leds(robot)
                print("[FSM] DONE — forward drive complete")
                state = "DONE"

        # ── DONE ─────────────────────────────────────────────────────────────
        elif state == "DONE":
            pass  # sit idle — extend with next phase here

        # ── TICK ─────────────────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()