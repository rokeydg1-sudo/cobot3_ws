#!/usr/bin/env python3

import csv
import math
import time
from datetime import datetime
from pathlib import Path

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)


# ============================================================
# 순찰 지점
# ============================================================

PATROL_POINTS = [
    ("P1",  0.204,  1.406),
    ("P2",  6.627,  2.203),
    ("P3", -1.490, -2.555),
]


# ============================================================
# Security Patrol Navigator
# ============================================================

class PatrolNavigator(BasicNavigator):

    def __init__(self):

        super().__init__(
            node_name="security_patrol"
        )

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------

        self.declare_parameter(
            "camera_topic",
            "/front_stereo_camera/left/image_raw"
        )

        self.declare_parameter(
            "output_dir",
            "~/cobot3_ws/patrol_logs"
        )

        self.camera_topic = (
            self.get_parameter("camera_topic")
            .get_parameter_value()
            .string_value
        )

        output_dir = (
            self.get_parameter("output_dir")
            .get_parameter_value()
            .string_value
        )

        # ----------------------------------------------------
        # 저장 경로
        # ----------------------------------------------------

        self.output_dir = (
            Path(output_dir)
            .expanduser()
        )

        self.image_dir = (
            self.output_dir / "images"
        )

        self.output_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        self.image_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        self.csv_path = (
            self.output_dir
            / "patrol_log.csv"
        )

        self.prepare_csv()

        # ----------------------------------------------------
        # Camera
        # ----------------------------------------------------

        self.bridge = CvBridge()

        self.latest_image = None
        self.latest_image_ros_time = None

        self.camera_sub = (
            self.create_subscription(
                Image,
                self.camera_topic,
                self.camera_callback,
                qos_profile_sensor_data
            )
        )

        self.get_logger().info(
            f"Camera topic: {self.camera_topic}"
        )

        self.get_logger().info(
            f"Output directory: {self.output_dir}"
        )

    # ========================================================
    # Camera callback
    # ========================================================

    def camera_callback(self, msg):

        try:

            self.latest_image = (
                self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="bgr8"
                )
            )

            self.latest_image_ros_time = (
                msg.header.stamp.sec
                + msg.header.stamp.nanosec
                * 1e-9
            )

        except Exception as e:

            self.get_logger().error(
                f"Camera conversion error: {e}"
            )

    # ========================================================
    # 카메라 수신 확인
    # ========================================================

    def wait_for_camera(self, timeout_sec=5.0):

        self.get_logger().info(
            "Waiting for camera image..."
        )

        start = time.monotonic()

        while rclpy.ok():

            if self.latest_image is not None:

                self.get_logger().info(
                    "Camera image received!"
                )

                return True

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if (
                time.monotonic() - start
                > timeout_sec
            ):

                self.get_logger().error(
                    "Camera image was not received."
                )

                self.get_logger().error(
                    "Check whether camera_topic "
                    "is sensor_msgs/msg/Image."
                )

                return False

        return False

    # ========================================================
    # CSV 생성
    # ========================================================

    def prepare_csv(self):

        if self.csv_path.exists():
            return

        with open(
            self.csv_path,
            "w",
            newline="",
            encoding="utf-8"
        ) as f:

            writer = csv.writer(f)

            writer.writerow([
                "cycle",
                "point",
                "wall_time",
                "ros_time_sec",
                "camera_time_sec",
                "x",
                "y",
                "image_file",
            ])

    # ========================================================
    # 사진 저장
    # ========================================================

    def save_snapshot(
        self,
        cycle,
        point_name,
        x,
        y
    ):

        if self.latest_image is None:

            self.get_logger().warning(
                f"[{point_name}] "
                "카메라 영상이 없습니다."
            )

            return False

        now = datetime.now().astimezone()

        timestamp = (
            now.strftime(
                "%Y%m%d_%H%M%S_%f"
            )[:-3]
        )

        filename = (
            f"cycle_{cycle:03d}_"
            f"{point_name}_"
            f"{timestamp}.jpg"
        )

        image_path = (
            self.image_dir / filename
        )

        # 최신 영상 복사
        image = self.latest_image.copy()

        success = cv2.imwrite(
            str(image_path),
            image
        )

        if not success:

            self.get_logger().error(
                f"Image save failed: "
                f"{image_path}"
            )

            return False

        wall_time = now.isoformat(
            timespec="milliseconds"
        )

        ros_time_sec = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        # ----------------------------------------------------
        # CSV 기록
        # ----------------------------------------------------

        with open(
            self.csv_path,
            "a",
            newline="",
            encoding="utf-8"
        ) as f:

            writer = csv.writer(f)

            writer.writerow([
                cycle,
                point_name,
                wall_time,
                f"{ros_time_sec:.3f}",
                (
                    f"{self.latest_image_ros_time:.3f}"
                    if self.latest_image_ros_time
                    is not None
                    else ""
                ),
                x,
                y,
                str(image_path),
            ])

        self.get_logger().info(
            f"[{point_name}] 사진 촬영 완료"
        )

        self.get_logger().info(
            f"  Time  : {wall_time}"
        )

        self.get_logger().info(
            f"  Image : {image_path}"
        )

        return True


# ============================================================
# Pose 생성
# ============================================================

def create_pose(
    navigator,
    x,
    y,
    yaw
):

    pose = PoseStamped()

    pose.header.frame_id = "map"

    pose.header.stamp = (
        navigator
        .get_clock()
        .now()
        .to_msg()
    )

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # yaw → quaternion

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0

    pose.pose.orientation.z = (
        math.sin(yaw / 2.0)
    )

    pose.pose.orientation.w = (
        math.cos(yaw / 2.0)
    )

    return pose


# ============================================================
# Waypoint Pose 생성
# ============================================================

def make_patrol_poses(
    navigator,
    patrol_points
):

    poses = []

    count = len(patrol_points)

    for i in range(count):

        name, x, y = (
            patrol_points[i]
        )

        next_index = (
            (i + 1) % count
        )

        _, next_x, next_y = (
            patrol_points[next_index]
        )

        # 다음 waypoint 방향 바라보기

        yaw = math.atan2(
            next_y - y,
            next_x - x
        )

        pose = create_pose(
            navigator,
            x,
            y,
            yaw
        )

        poses.append(pose)

        navigator.get_logger().info(
            f"{name}: "
            f"x={x:.3f}, "
            f"y={y:.3f}, "
            f"yaw="
            f"{math.degrees(yaw):.1f} deg"
        )

    return poses


# ============================================================
# 한 번의 순찰
# ============================================================

def run_patrol_cycle(
    navigator,
    cycle_number
):

    poses = make_patrol_poses(
        navigator,
        PATROL_POINTS
    )

    total_points = len(poses)

    navigator.get_logger().info(
        "=================================="
    )

    navigator.get_logger().info(
        f"Patrol cycle {cycle_number} START"
    )

    navigator.get_logger().info(
        "Using FollowWaypoints"
    )

    navigator.get_logger().info(
        "=================================="
    )

    # ========================================================
    # 핵심 변경
    #
    # goThroughPoses()
    #       ↓
    # followWaypoints()
    # ========================================================

    navigator.followWaypoints(poses)

    captured_count = 0

    last_waypoint = 0

    # ========================================================
    # Waypoint 실행 중
    # ========================================================

    while not navigator.isTaskComplete():

        feedback = navigator.getFeedback()

        if feedback is None:
            continue

        current_waypoint = int(
            feedback.current_waypoint
        )

        # ----------------------------------------------------
        # waypoint index가 증가했다
        #
        # 0 → 1 : P1 완료
        # 1 → 2 : P2 완료
        # ----------------------------------------------------

        if (
            current_waypoint
            != last_waypoint
        ):

            navigator.get_logger().info(
                "----------------------------------"
            )

            navigator.get_logger().info(
                f"Waypoint changed: "
                f"{last_waypoint} "
                f"-> {current_waypoint}"
            )

            # 현재 waypoint 이전까지
            # 도착 완료한 것으로 판단

            while (
                captured_count
                < current_waypoint
            ):

                point_name, x, y = (
                    PATROL_POINTS[
                        captured_count
                    ]
                )

                navigator.get_logger().info(
                    f"{point_name} 도착"
                )

                navigator.save_snapshot(
                    cycle_number,
                    point_name,
                    x,
                    y
                )

                captured_count += 1

            last_waypoint = (
                current_waypoint
            )

    # ========================================================
    # 최종 결과
    # ========================================================

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:

        navigator.get_logger().info(
            "FollowWaypoints SUCCEEDED"
        )

        # ----------------------------------------------------
        # 마지막 waypoint(P3)는
        # current_waypoint가 다음 값으로 바뀌지 않으므로
        # Action 성공 시 촬영
        # ----------------------------------------------------

        while (
            captured_count
            < total_points
        ):

            point_name, x, y = (
                PATROL_POINTS[
                    captured_count
                ]
            )

            navigator.get_logger().info(
                f"{point_name} 최종 도착"
            )

            navigator.save_snapshot(
                cycle_number,
                point_name,
                x,
                y
            )

            captured_count += 1

        return True

    elif result == TaskResult.CANCELED:

        navigator.get_logger().warning(
            "FollowWaypoints CANCELED"
        )

    elif result == TaskResult.FAILED:

        navigator.get_logger().error(
            "FollowWaypoints FAILED"
        )

    else:

        navigator.get_logger().error(
            f"Unknown result: {result}"
        )

    return False


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    navigator = PatrolNavigator()

    try:

        # ----------------------------------------------------
        # 카메라부터 검증
        # ----------------------------------------------------

        if not navigator.wait_for_camera(
            timeout_sec=5.0
        ):

            navigator.get_logger().error(
                "Patrol aborted because "
                "camera is not available."
            )

            return

        # ----------------------------------------------------
        # Nav2
        # ----------------------------------------------------

        navigator.get_logger().info(
            "Waiting for Nav2..."
        )

        navigator.waitUntilNav2Active()

        navigator.get_logger().info(
            "Nav2 ACTIVE"
        )

        # ----------------------------------------------------
        # 순찰 시작
        # ----------------------------------------------------

        run_patrol_cycle(
            navigator,
            cycle_number=1
        )

    except KeyboardInterrupt:

        navigator.get_logger().info(
            "Patrol interrupted."
        )

    except Exception as e:

        navigator.get_logger().error(
            f"Error: {e}"
        )

    finally:

        navigator.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()