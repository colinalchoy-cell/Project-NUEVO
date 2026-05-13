from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner
import math
import numpy as np


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

TAG_ID = 11 # set aruco tag ID 11 
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


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
    robot.set_tracked_tag_id(TAG_ID) # set aruco tag ID as the tracked tag for localization


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    drive_handle = None
    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"FSM period: {period:.3f} seconds")
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")
            # center lane
            # path_control_points = [
            #     (0.0,   0.0),
            #     (0.0, 2500.0),
            #     (1000.0, 2500.0),
            # ]
            # left lane
            # path_control_points = [
            #     (300.0,   0.0),
            #     (300.0, 2500.0),
            #     (1300.0, 2500.0),
            # ]
            #==================
            # Straight path for testing
            path_control_points = [
                (0.0,   100),
                (0, 200),
                (0, 300),
            ]
            path = densify_polyline(path_control_points, spacing=400.0)

            robot._nav_follow_pp_path(
                #Distance bot looks ahead on the path to detect objects
                lookahead_distance=100.0,
                #moving speed
                max_linear_speed=140.0,
                #angluar velocity that keeps rover along curved path
                max_angular_speed=1.5,
                #if bot is within this distance to the goal, it will stop and consider the goal reached
                goal_tolerance=20.0,
                #filters out obstacles further than this range
                obstacles_range=450.0,
                #angle of view for obstacle detection in front of the robot, centered around the forward direction
                view_angle=math.radians(70.0),
                #distance at which the robot considers an obstacle too close and will stop to avoid collision
                safe_dist=250.0,
                # Fixed contol loop number delay where further updates temporarily suspended. So it can commit to current avoidance manuever without changing its target trajectory
                avoidance_delay=150,
                # lookahead distance can be temporarily reduced by scaling factor to help with tracking newly generated path
                alpha_Ld=0.7,
                offset=270.0,
                #bot considers a left and right lane, so total width = land_width*2
                lane_width=500.0,
                obstacle_avoidance=True,
                #x distance of lane center from the origin (where robot starts)
                x_L=300.0,
            )
            robot.planner.set_path(path)
            print("Path is ready, Entering IDLE state.")
            print("[FSM] IDLE - Press BTN_1 to enter MOVING state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            robot._draw_lidar_obstacles()
            if robot.get_button(Button.BTN_1):
                print("Start Moving!")
                print("[FSM] MOVING")
                state = "MOVING"
            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot and saving trajectory.")
                robot.shutdown()

        elif state == "MOVING":
            show_moving_leds(robot)
            # if next_tick % 0.5 < period: # print every half second
            #     robot._draw_lidar_obstacles()
            #     print("Obstacle figure updated.")
            state = robot._nav_follow_pp_path_loop()

        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()