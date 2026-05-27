"""
shelf_patty_follow.py - follow a shelf, find a patty, stop in front of it
========================================================================

HOW TO RUN
----------
Start the vision node in another terminal:

    ros2 run vision vision_node

Then copy this file over main.py and restart the robot node:

    cp examples/shelf_patty_follow.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Press BTN_1 to start. The robot follows a shelf/wall on its left using lidar.
When the camera sees a patty, it slows down and keeps the patty centered in
the camera while still maintaining safe shelf distance. It stops once the patty
is centered or if lidar sees something too close in front.
"""

from __future__ import annotations

import math
import time

import numpy as np

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------

PATTY_CLASS_NAME = "patty"
MIN_PATTY_CONFIDENCE = 0.50
VISION_STALE_SEC = 1.0

# Shelf is assumed to be on the robot's left side. Robot frame: +x forward,
# +y left, so the left shelf has y > 0.
SHELF_DISTANCE_MM = 300.0
SIDE_CONE_DEG = 30.0
FRONT_CONE_DEG = 25.0
FRONT_STOP_MM = 260.0

SEARCH_SPEED_MM_S = 110.0
TRACK_SPEED_MM_S = 60.0
MAX_TURN_DEG_S = 55.0

WALL_KP = 0.45
VISION_KP = 35.0
CENTER_TOLERANCE_FRACTION = 0.08
CENTER_HOLD_SEC = 0.5


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    robot.enable_lidar()
    robot.set_lidar_mount(
        x_mm=LIDAR_MOUNT_X_MM,
        y_mm=LIDAR_MOUNT_Y_MM,
        theta_deg=LIDAR_MOUNT_THETA_DEG,
    )
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=LIDAR_RANGE_MAX_MM,
        fov_deg=LIDAR_FOV_DEG,
    )

    robot.enable_vision()


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def set_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)


def set_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 160)
    robot.set_led(LED.RED, 0)


def set_done_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.RED, 180)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def get_obstacles(robot: Robot) -> np.ndarray:
    obs = np.asarray(robot._obstacles_mm, dtype=float)
    if obs.ndim != 2 or obs.shape[1] != 2:
        return np.empty((0, 2), dtype=float)
    return obs


def points_in_cone(obs: np.ndarray, center_deg: float, half_angle_deg: float) -> np.ndarray:
    if obs.shape[0] == 0:
        return obs
    angles = np.degrees(np.arctan2(obs[:, 1], obs[:, 0]))
    delta = (angles - center_deg + 180.0) % 360.0 - 180.0
    return obs[np.abs(delta) <= half_angle_deg]


def front_clear(robot: Robot) -> bool:
    front = points_in_cone(get_obstacles(robot), center_deg=0.0, half_angle_deg=FRONT_CONE_DEG)
    if front.shape[0] == 0:
        return True
    distances = np.hypot(front[:, 0], front[:, 1])
    return float(np.min(distances)) >= FRONT_STOP_MM


def wall_follow_turn_deg_s(robot: Robot) -> float:
    left_side = points_in_cone(get_obstacles(robot), center_deg=90.0, half_angle_deg=SIDE_CONE_DEG)
    if left_side.shape[0] == 0:
        return 0.0

    lateral_distance_mm = float(np.median(np.abs(left_side[:, 1])))
    error_mm = SHELF_DISTANCE_MM - lateral_distance_mm

    # Positive angular velocity is CCW. If the robot is too close to the left
    # shelf, turn right/CW. If it is too far, turn left/CCW.
    return clamp(-WALL_KP * error_mm, -MAX_TURN_DEG_S, MAX_TURN_DEG_S)


def best_patty(robot: Robot) -> dict | None:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    candidates = []
    for detection in robot.get_detections(PATTY_CLASS_NAME):
        if float(detection["confidence"]) >= MIN_PATTY_CONFIDENCE:
            candidates.append(detection)

    if not candidates:
        return None

    return max(candidates, key=lambda det: float(det["confidence"]))


def patty_center_error(robot: Robot, detection: dict) -> float | None:
    image_width, _image_height = robot.get_detection_image_size()
    if image_width <= 0:
        return None

    bbox = detection["bbox"]
    center_x = float(bbox["x"]) + 0.5 * float(bbox["width"])
    image_center_x = 0.5 * float(image_width)
    return (center_x - image_center_x) / image_center_x


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    centered_since = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if robot.was_button_pressed(Button.BTN_2):
            robot.stop()
            set_idle_leds(robot)
            centered_since = 0.0
            print("[FSM] IDLE - cancelled")
            state = "IDLE"

        if state == "INIT":
            start_robot(robot)
            set_idle_leds(robot)
            print("[FSM] IDLE - press BTN_1 to follow shelf and search for patty")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                set_running_leds(robot)
                centered_since = 0.0
                print("[FSM] SEARCH_SHELF")
                state = "SEARCH_SHELF"

        elif state == "SEARCH_SHELF":
            if not front_clear(robot):
                robot.stop()
                set_done_leds(robot)
                print("[safe] stopped - obstacle too close in front")
                state = "DONE"
            else:
                detection = best_patty(robot)
                if detection is None:
                    robot.set_velocity(SEARCH_SPEED_MM_S, wall_follow_turn_deg_s(robot))
                else:
                    centered_since = 0.0
                    print("[FSM] TRACK_PATTY")
                    state = "TRACK_PATTY"

        elif state == "TRACK_PATTY":
            if not front_clear(robot):
                robot.stop()
                set_done_leds(robot)
                print("[safe] stopped - obstacle too close in front")
                state = "DONE"
            else:
                detection = best_patty(robot)
                if detection is None:
                    centered_since = 0.0
                    print("[FSM] SEARCH_SHELF - lost patty")
                    state = "SEARCH_SHELF"
                else:
                    center_error = patty_center_error(robot, detection)
                    if center_error is None:
                        robot.set_velocity(TRACK_SPEED_MM_S, wall_follow_turn_deg_s(robot))
                    else:
                        vision_turn = -VISION_KP * center_error
                        turn = wall_follow_turn_deg_s(robot) + vision_turn
                        robot.set_velocity(TRACK_SPEED_MM_S, clamp(turn, -MAX_TURN_DEG_S, MAX_TURN_DEG_S))

                        if abs(center_error) <= CENTER_TOLERANCE_FRACTION:
                            if centered_since == 0.0:
                                centered_since = now
                            elif now - centered_since >= CENTER_HOLD_SEC:
                                robot.stop()
                                set_done_leds(robot)
                                print("[FSM] DONE - patty centered")
                                state = "DONE"
                        else:
                            centered_since = 0.0

        elif state == "DONE":
            robot.stop()
            if robot.was_button_pressed(Button.BTN_1):
                set_running_leds(robot)
                centered_since = 0.0
                print("[FSM] SEARCH_SHELF")
                state = "SEARCH_SHELF"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
