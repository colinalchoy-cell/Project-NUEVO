from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline

# ---------------------------------------------------------------------------
# Sensor toggles
# ---------------------------------------------------------------------------

ENABLE_LIDAR = True
ENABLE_GPS   = True
TAG_ID       = 24

# ---------------------------------------------------------------------------
# GPS tuning
# ---------------------------------------------------------------------------

GPS_POSITION_ALPHA              = 0.0
ENABLE_GPS_TANGENT_HEADING      = True
GPS_TANGENT_ALPHA               = 0.1
GPS_TANGENT_MIN_DISPLACEMENT_MM = 400.0

# ---------------------------------------------------------------------------
# Burger stop positions and pause durations
# These are the 3 waypoints the robot drives to before the main path,
# pausing at each one to simulate burger assembly/delivery.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------
# BURGER STOP WAYPOINTS — set these to your actual positions (mm)
# Each is (x, y) relative to the starting odometry origin
# ---------------------------------------------------------------
BURGER_STOP_1 = (-150,   900.0)   # first ingredient station
BURGER_STOP_2 = (-150,  1050.0)   # second ingredient station
BURGER_STOP_3 = (-150,  1200.0)   # third ingredient station / assembly point

# ---------------------------------------------------------------
# PAUSE DURATIONS at each burger stop (seconds)
# Replace with robot.was_button_pressed() if you want manual advance
# ---------------------------------------------------------------
BURGER_STOP_1_PAUSE_S = 5.0
BURGER_STOP_2_PAUSE_S = 5.0
BURGER_STOP_3_PAUSE_S = 5.0

# Velocity and tolerance for burger stop drives
BURGER_STOP_VELOCITY_MM_S  = 200.0
BURGER_STOP_TOLERANCE_MM   = 30.0
BURGER_STOP_LOOKAHEAD_MM   = 50.0
BURGER_STOP_MAX_ANG_RAD_S  = 1.5

# ---------------------------------------------------------------------------
# Pure pursuit configuration — main obstacle course path
# ---------------------------------------------------------------------------

RAW_PATH_CONTROL_POINTS = [
    (0.0,  1800),
    (0,    3410),
    (540,  3410),
    (200,  300),
    (800,  310),
    (800,  610),
]

PATH_CONTROL_POINTS = densify_polyline(RAW_PATH_CONTROL_POINTS, spacing=10.0)

VELOCITY_MM_S           = 400.0
LOOKAHEAD_MM            = 15.0
TOLERANCE_MM            = 25.0
ADVANCE_RADIUS_MM       = 80.0
MAX_ANGULAR_RAD_S       = 1.75
STATUS_PRINT_INTERVAL_S = 0.5

# ---------------------------------------------------------------------------
# LAPF tuning
# ---------------------------------------------------------------------------

LAPF_LEASH_LENGTH_MM      = 400.0
LAPF_REPULSION_RANGE_MM   = 300.0
LAPF_TARGET_SPEED_MM_S    = 225.0
LAPF_REPULSION_GAIN       = 550.0
LAPF_ATTRACTION_GAIN      = 1.0
LAPF_FORCE_EMA_ALPHA      = 0.35
LAPF_INFLATION_MARGIN_MM  = 130.0
LAPF_LEASH_HALF_ANGLE_DEG = 25.0
LAPF_GOAL_EXTENSION_MM    = 2450.0


def build_far_goal() -> tuple[float, float]:
    if len(RAW_PATH_CONTROL_POINTS) < 2:
        return RAW_PATH_CONTROL_POINTS[-1]
    start_x, start_y = RAW_PATH_CONTROL_POINTS[-2]
    end_x,   end_y   = RAW_PATH_CONTROL_POINTS[-1]
    dx   = end_x - start_x
    dy   = end_y - start_y
    dist = (dx ** 2 + dy ** 2) ** 0.5
    if dist < 1e-6:
        return end_x, end_y
    ux = dx / dist
    uy = dy / dist
    return (end_x + ux * LAPF_GOAL_EXTENSION_MM,
            end_y + uy * LAPF_GOAL_EXTENSION_MM)


GOAL_MM = build_far_goal()

# ---------------------------------------------------------------------------
# Setup helpers
# ---------------------------------------------------------------------------

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
    if ENABLE_LIDAR:
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
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")
        if ENABLE_GPS_TANGENT_HEADING:
            robot.enable_gps_tangent_heading(
                alpha=GPS_TANGENT_ALPHA,
                min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM,
            )


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed — continuing")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def show_paused_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 200)


def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fx, fy, ftheta = robot.get_fused_pose()
        print(
            f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ_odom={otheta:5.1f}°  |  "
            f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ_fused={ftheta:5.1f}°  "
            f"gps={'fresh' if robot.is_gps_active() else 'stale'}"
        )
    else:
        print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")

# ---------------------------------------------------------------------------
# Navigation primitives
# ---------------------------------------------------------------------------

def drive_to_waypoint(robot: Robot, x: float, y: float):
    """Non-blocking pure pursuit to a single XY waypoint."""
    waypoints = densify_polyline([(0.0, 0.0), (x, y)], spacing=10.0)
    return robot.purepursuit_follow_path(
        waypoints=waypoints,
        velocity=BURGER_STOP_VELOCITY_MM_S,
        lookahead=BURGER_STOP_LOOKAHEAD_MM,
        tolerance=BURGER_STOP_TOLERANCE_MM,
        advance_radius=BURGER_STOP_TOLERANCE_MM,
        max_angular_rad_s=BURGER_STOP_MAX_ANG_RAD_S,
        blocking=False,
    )


def start_path(robot: Robot):
    return robot.purepursuit_follow_path(
        waypoints=PATH_CONTROL_POINTS,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        blocking=False,
    )


def start_lapf(robot: Robot, goal_xy: tuple[float, float]):
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    return robot.lapf_to_goal(
        gx, gy,
        velocity=VELOCITY_MM_S,
        tolerance=TOLERANCE_MM,
        leash_length_mm=LAPF_LEASH_LENGTH_MM,
        repulsion_range_mm=LAPF_REPULSION_RANGE_MM,
        target_speed_mm_s=LAPF_TARGET_SPEED_MM_S,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        repulsion_gain=LAPF_REPULSION_GAIN,
        attraction_gain=LAPF_ATTRACTION_GAIN,
        force_ema_alpha=LAPF_FORCE_EMA_ALPHA,
        inflation_margin_mm=LAPF_INFLATION_MARGIN_MM,
        leash_half_angle_deg=LAPF_LEASH_HALF_ANGLE_DEG,
        blocking=False,
    )

# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state                = "INIT"
    drive_handle         = None
    lapf_handle          = None
    pause_until          = 0.0
    last_status_print_at = 0.0

    # Burger stops as a list of (x, y, pause_s, label)
    burger_stops = [
        (BURGER_STOP_1[0], BURGER_STOP_1[1], BURGER_STOP_1_PAUSE_S, "Stop 1 — ingredient 1"),
        (BURGER_STOP_2[0], BURGER_STOP_2[1], BURGER_STOP_2_PAUSE_S, "Stop 2 — ingredient 2"),
        (BURGER_STOP_3[0], BURGER_STOP_3[1], BURGER_STOP_3_PAUSE_S, "Stop 3 — assembly"),
    ]
    burger_stop_index = 0   # which stop we're on

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            burger_stop_index = 0
            print("[FSM] IDLE — press BTN_1 to start, BTN_2 to cancel")
            state = "IDLE"

        # ── IDLE ─────────────────────────────────────────────────────────────
        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_moving_leds(robot)
                bx, by, _, label = burger_stops[burger_stop_index]
                print(f"[FSM] BURGER_DRIVE — heading to {label} ({bx:.0f}, {by:.0f})")
                drive_handle = drive_to_waypoint(robot, bx, by)
                state = "BURGER_DRIVE"

        # ── BURGER_DRIVE — driving to next burger stop ────────────────────────
        elif state == "BURGER_DRIVE":
            if robot.was_button_pressed(Button.BTN_2):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — burger drive cancelled")
                state = "IDLE"

            elif drive_handle is not None and drive_handle.is_finished():
                drive_handle = None
                robot.stop()
                _, _, pause_s, label = burger_stops[burger_stop_index]
                pause_until = now + pause_s
                show_paused_leds(robot)
                print(f"[FSM] BURGER_PAUSE — at {label}, pausing {pause_s:.1f}s")
                # -------------------------------------------------------
                # Replace time.monotonic() pause with your burger assembly
                # function call here if needed, e.g.:
                #   run_burger_assembly(robot)
                # -------------------------------------------------------
                state = "BURGER_PAUSE"

        # ── BURGER_PAUSE — waiting at burger stop ─────────────────────────────
        elif state == "BURGER_PAUSE":
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — burger pause cancelled")
                state = "IDLE"

            elif now >= pause_until:
                burger_stop_index += 1
                if burger_stop_index < len(burger_stops):
                    # Drive to next burger stop
                    bx, by, _, label = burger_stops[burger_stop_index]
                    show_moving_leds(robot)
                    print(f"[FSM] BURGER_DRIVE — heading to {label} ({bx:.0f}, {by:.0f})")
                    drive_handle = drive_to_waypoint(robot, bx, by)
                    state = "BURGER_DRIVE"
                else:
                    # All burger stops done — reset odometry and start main path
                    print("[FSM] ALL BURGER STOPS DONE — resetting pose, starting main path")
                    reset_mission_pose(robot)
                    show_moving_leds(robot)
                    print(f"[FSM] MOVING — {len(PATH_CONTROL_POINTS)} waypoints")
                    drive_handle = start_path(robot)
                    last_status_print_at = now
                    state = "MOVING"

        # ── MOVING — main pure pursuit path ──────────────────────────────────
        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                if lapf_handle is not None:
                    lapf_handle.cancel()
                    lapf_handle.wait(timeout=1.0)
                    lapf_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — path cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if drive_handle is not None and drive_handle.is_finished():
                    print("[FSM] PURE-PURSUIT DONE — starting APF avoidance")
                    print_status(robot)
                    drive_handle = None
                    robot.stop()
                    robot.wait_for_pose_update(timeout=0.2)
                    print(f"[FSM] APF target: {GOAL_MM}")
                    lapf_handle = start_lapf(robot, GOAL_MM)
                    last_status_print_at = now
                    state = "AVOIDING"

        # ── AVOIDING — APF obstacle avoidance ────────────────────────────────
        elif state == "AVOIDING":
            if robot.was_button_pressed(Button.BTN_2):
                if lapf_handle is not None:
                    lapf_handle.cancel()
                    lapf_handle.wait(timeout=1.0)
                    lapf_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — APF cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if lapf_handle is not None and lapf_handle.is_finished():
                    print("[FSM] DONE — APF goal complete")
                    print_status(robot)
                    lapf_handle = None
                    robot.stop()
                    show_idle_leds(robot)
                    print("[FSM] IDLE — press BTN_1 to run again")
                    state = "IDLE"

        # ── TICK ─────────────────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()