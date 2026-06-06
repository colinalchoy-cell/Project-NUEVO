"""
pursuit_apf_fsm.py — FSM-based pure-pursuit + APF avoidance
=============================================================
Copy this file over main.py, then restart the robot node:

    cp examples/pursuit_apf_fsm.py main.py
    ros2 run robot robot

BTN_1 starts the path. BTN_2 cancels and returns to IDLE.
"""

from __future__ import annotations

import time
from typing import Optional

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
from robot.robot_fsm import RobotFSM
from robot.util import densify_polyline  # noqa: F401 - optional helper for students


# ---------------------------------------------------------------------------
# Sensor toggles — set True if the corresponding node is running
# Hardware calibration (wheel geometry, lidar mount, tag offset) lives in
# robot/hardware_map.py.
# ---------------------------------------------------------------------------

ENABLE_LIDAR = True
ENABLE_GPS = False

TAG_ID = 24  # IMPORTANT: set to the ArUco marker ID on your robot


# ---------------------------------------------------------------------------
# GPS tuning (only used when ENABLE_GPS = True)
#
# GPS_POSITION_ALPHA     — how strongly each GPS fix pulls the fused position.
#                          0.05 = smooth/slow, 0.10 = default, 0.30 = aggressive
#
# ENABLE_GPS_TANGENT_HEADING — derive heading from GPS trajectory direction.
#                          False = pure odometry heading (default).
#
# GPS_TANGENT_ALPHA      — how strongly GPS tangent corrects odometry heading.
#                          0.05 = gentle, 0.15 = default, 0.30 = aggressive
#
# GPS_TANGENT_MIN_DISPLACEMENT_MM — travel required before accepting a new
#                          heading sample. 100 = responsive, 200 = default,
#                          400 = noise-robust (for jittery GPS)
#
# To tune: watch θ_odom vs θ_fused in the status output while running.
# ---------------------------------------------------------------------------

GPS_POSITION_ALPHA = 0.10
ENABLE_GPS_TANGENT_HEADING = False
GPS_TANGENT_ALPHA = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0


# ---------------------------------------------------------------------------
# Pure pursuit configuration
# ---------------------------------------------------------------------------

RAW_PATH_CONTROL_POINTS = [
    (0.0, 0.0),
    (0.0, 600),
    (600, 600),
    (600, 1200),
]

# Densify long segments for smoother tracking. Adjust spacing (mm)
# for more/less intermediate points.
PATH_CONTROL_POINTS = densify_polyline(RAW_PATH_CONTROL_POINTS, spacing=10.0)

VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 0.0
TOLERANCE_MM = 25.0
ADVANCE_RADIUS_MM = 80.0
MAX_ANGULAR_RAD_S = 1.5

STATUS_PRINT_INTERVAL_S = 0.5


# ---------------------------------------------------------------------------
# LAPF (leashed APF) tuning — used when switching to obstacle-avoidance mode
# ---------------------------------------------------------------------------

LAPF_LEASH_LENGTH_MM = 400.0
LAPF_REPULSION_RANGE_MM = 300.0
LAPF_TARGET_SPEED_MM_S = 150.0
LAPF_REPULSION_GAIN = 550.0
LAPF_ATTRACTION_GAIN = 1.0
LAPF_FORCE_EMA_ALPHA = 0.35
LAPF_INFLATION_MARGIN_MM = 130.0
LAPF_LEASH_HALF_ANGLE_DEG = 25.0
LAPF_GOAL_EXTENSION_MM = 10000.0


def build_far_goal() -> tuple[float, float]:
    if len(RAW_PATH_CONTROL_POINTS) < 2:
        return RAW_PATH_CONTROL_POINTS[-1]

    start_x, start_y = RAW_PATH_CONTROL_POINTS[-2]
    end_x, end_y = RAW_PATH_CONTROL_POINTS[-1]
    dx = end_x - start_x
    dy = end_y - start_y
    dist = (dx ** 2 + dy ** 2) ** 0.5
    if dist < 1e-6:
        return end_x, end_y

    ux = dx / dist
    uy = dy / dist
    return (end_x + ux * LAPF_GOAL_EXTENSION_MM, end_y + uy * LAPF_GOAL_EXTENSION_MM)


GOAL_MM = build_far_goal()


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
        print("[sensor] lidar enabled — subscribing to /scan")

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
        print(
            "[warn] odometry reset not confirmed within 2.0s; continuing with latest pose"
        )
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
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
        gx,
        gy,
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


class PursuitAPFFSM(RobotFSM):
    def __init__(self, robot: Robot) -> None:
        super().__init__(robot, initial_state="IDLE")
        self.drive_handle: Optional[object] = None
        self.lapf_handle: Optional[object] = None
        self.last_status_print_at = 0.0

        self.add_transition("IDLE", "start_path", "MOVING", action=self._on_start_path)
        self.add_transition("MOVING", "cancel", "IDLE", action=self._on_cancel_moving)
        self.add_transition("MOVING", "path_complete", "AVOIDING", action=self._on_path_complete)
        self.add_transition("AVOIDING", "cancel", "IDLE", action=self._on_cancel_avoiding)
        self.add_transition("AVOIDING", "goal_reached", "IDLE", action=self._on_goal_reached)

    def update(self) -> None:
        now = time.monotonic()
        state = self.get_state()

        if state == "IDLE":
            if self.robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(self.robot)
                self.trigger("start_path")

        elif state == "MOVING":
            if self.robot.was_button_pressed(Button.BTN_2):
                self.trigger("cancel")
                return

            if self.drive_handle is not None and self.drive_handle.is_finished():
                self.trigger("path_complete")
                return

            self._print_status_if_needed(now)

        elif state == "AVOIDING":
            if self.robot.was_button_pressed(Button.BTN_2):
                self.trigger("cancel")
                return

            if self.lapf_handle is not None and self.lapf_handle.is_finished():
                self.trigger("goal_reached")
                return

            self._print_status_if_needed(now)

    def _on_start_path(self) -> None:
        show_moving_leds(self.robot)
        print(f"[FSM] MOVING — {len(PATH_CONTROL_POINTS)} waypoints")
        self.drive_handle = start_path(self.robot)
        self.last_status_print_at = time.monotonic()

    def _on_cancel_moving(self) -> None:
        self._cancel_drive()
        self.robot.stop()
        show_idle_leds(self.robot)
        print("[FSM] IDLE — path cancelled")

    def _on_path_complete(self) -> None:
        print("[FSM] PURE-PURSUIT DONE — starting APF avoidance")
        print_status(self.robot)
        self.drive_handle = None
        self.robot.stop()
        self.robot.wait_for_pose_update(timeout=0.2)
        print(f"[FSM] APF target set to {GOAL_MM}")
        self.lapf_handle = start_lapf(self.robot, GOAL_MM)
        self.last_status_print_at = time.monotonic()

    def _on_cancel_avoiding(self) -> None:
        self._cancel_lapf()
        self.robot.stop()
        show_idle_leds(self.robot)
        print("[FSM] IDLE — APF cancelled")

    def _on_goal_reached(self) -> None:
        print("[FSM] DONE — APF goal complete")
        print_status(self.robot)
        self.lapf_handle = None
        self.robot.stop()
        show_idle_leds(self.robot)
        print("[FSM] IDLE — press BTN_1 to run again")

    def _cancel_drive(self) -> None:
        if self.drive_handle is not None:
            self.drive_handle.cancel()
            self.drive_handle.wait(timeout=1.0)
            self.drive_handle = None

    def _cancel_lapf(self) -> None:
        if self.lapf_handle is not None:
            self.lapf_handle.cancel()
            self.lapf_handle.wait(timeout=1.0)
            self.lapf_handle = None

    def _print_status_if_needed(self, now: float) -> None:
        if now - self.last_status_print_at >= STATUS_PRINT_INTERVAL_S:
            print_status(self.robot)
            self.last_status_print_at = now


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
    reset_mission_pose(robot)
    show_idle_leds(robot)

    print("[FSM] IDLE — press BTN_1 to start path, BTN_2 to cancel")
    print(
        f"[CFG] velocity={VELOCITY_MM_S:.0f} mm/s  lookahead={LOOKAHEAD_MM:.0f} mm  "
        f"tolerance={TOLERANCE_MM:.0f} mm  advance_radius={ADVANCE_RADIUS_MM:.0f} mm"
    )
    if ENABLE_LIDAR:
        print(
            f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f}, {LIDAR_MOUNT_Y_MM:.0f}) mm "
            f"theta={LIDAR_MOUNT_THETA_DEG:.1f}° filter={LIDAR_RANGE_MIN_MM:.0f}-"
            f"{LIDAR_RANGE_MAX_MM:.0f} mm fov={LIDAR_FOV_DEG}"
        )
    if ENABLE_GPS:
        print(
            f"[CFG] gps tag_id={TAG_ID}  "
            f"tag_body=({TAG_BODY_OFFSET_X_MM:.0f}, {TAG_BODY_OFFSET_Y_MM:.0f}) mm  "
            f"position_alpha={GPS_POSITION_ALPHA:.2f}"
        )
        if ENABLE_GPS_TANGENT_HEADING:
            print(
                f"[CFG] heading=gps_tangent  "
                f"alpha={GPS_TANGENT_ALPHA:.2f}  "
                f"min_displacement={GPS_TANGENT_MIN_DISPLACEMENT_MM:.0f} mm"
            )
        else:
            print("[CFG] heading=imu")

    fsm = PursuitAPFFSM(robot)
    fsm.spin()
