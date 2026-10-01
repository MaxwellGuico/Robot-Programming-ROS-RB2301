import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.sub_odom = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.last_scan = None
        self.last_angles = None

        # Store each gap decision beside the can layout for this environment.
        self.declare_parameter("save_debug_plot", True)
        self.declare_parameter("debug_output_root", "ca1_runs")
        self.declare_parameter("lidar_offset_x", 0.11423)
        self.declare_parameter("lidar_offset_y", 0.0)
        self.save_debug_plot = self.get_parameter("save_debug_plot").value
        self.debug_output_root = Path(
            self.get_parameter("debug_output_root").value
        ).expanduser().resolve()
        self.lidar_offset_x = self.get_parameter("lidar_offset_x").value
        self.lidar_offset_y = self.get_parameter("lidar_offset_y").value
        self.debug_run_directory = self.get_debug_run_directory()
        plot_numbers = [
            int(path.stem.rsplit("_", 1)[1])
            for path in self.debug_run_directory.glob("gap_decision_*.jpg")
            if path.stem.rsplit("_", 1)[1].isdigit()
        ]
        self.debug_plot_number = max(plot_numbers, default=0)
        self.last_debug_plot_time = 0.0
        self.debug_plot_interval = 1.0
        self.debug_clusters = []
        self.debug_candidate_gaps = []
        self.get_logger().info(
            f"Environment images: {self.debug_run_directory}"
        )

        # Automatically evaluate every run from the first odometry sample.
        self.declare_parameter("enable_evaluation", True)
        self.declare_parameter("collision_yaw_jump_degrees", 20.0)
        self.declare_parameter("collision_yaw_deviation_degrees", 30.0)
        self.declare_parameter("finish_x_tolerance", 0.0)
        self.declare_parameter("stop_on_evaluation_complete", True)
        self.evaluation_enabled = self.get_parameter("enable_evaluation").value
        self.yaw_jump_threshold = np.deg2rad(
            self.get_parameter("collision_yaw_jump_degrees").value
        )
        self.yaw_deviation_threshold = np.deg2rad(
            self.get_parameter("collision_yaw_deviation_degrees").value
        )
        self.finish_x_tolerance = self.get_parameter("finish_x_tolerance").value
        self.stop_on_evaluation_complete = self.get_parameter(
            "stop_on_evaluation_complete"
        ).value
        self.evaluation_started = False
        self.evaluation_complete = False
        self.evaluation_saved = False
        self.evaluation_start_time = None
        self.evaluation_last_time = None
        self.evaluation_clock_source = None
        self.evaluation_start_position = None
        self.evaluation_last_position = None
        self.evaluation_previous_position = None
        self.total_distance = 0.0
        self.latest_robot_pose = None
        self.robot_trajectory = []
        self.overview_decisions = []
        self.evaluation_start_yaw = None
        self.evaluation_previous_yaw = None
        self.collision_detected = False
        self.collision_reason = None
        self.collision_elapsed_seconds = None
        self.max_yaw_jump = 0.0
        self.max_yaw_deviation = 0.0
        self.environment_metadata = self.load_environment_metadata()
        self.last_can_row_x = None
        if self.environment_metadata is None:
            self.evaluation_enabled = False
        else:
            self.last_can_row_x = float(
                self.environment_metadata["last_can_row_x"]
            )
            self.get_logger().info(
                "Evaluation armed: finish at "
                f"x={self.last_can_row_x:.3f} m; timing starts on first odometry"
            )


        self.robot_width = 0.23
        self.robot_length = 0.23
        self.safety_margin = 0.05
        self.required_gap = self.robot_width + 2 * self.safety_margin
        self.stop_distance = 0.6 # og:0.9
        self.look_ahead = 0.75

        self.avoiding = False
        self.clear_count = 0
        self.gap_heading = None

        self.gap_side = -1   # -1 = start with right, +1 = start with left
        self.side = 0        # emergency sidestep direction

        self.side_lock_count = 0
        self.missing_count = 0
        self.last_command = (0.0, 0.0)

        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

    def get_debug_run_directory(self):
        """Use the folder created by the latest obstacle-generator run."""
        latest_run_file = self.debug_output_root / "latest_run.txt"
        try:
            run_directory = Path(
                latest_run_file.read_text(encoding="utf-8").strip()
            ).expanduser().resolve()
        except (OSError, ValueError):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            run_directory = self.debug_output_root / timestamp

        run_directory.mkdir(parents=True, exist_ok=True)
        return run_directory

    def load_environment_metadata(self):
        """Load the finish row produced with the active can environment."""
        if not self.evaluation_enabled:
            return None

        metadata_path = self.debug_run_directory / "environment.json"
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            float(metadata["last_can_row_x"])
            return metadata
        except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            self.get_logger().warning(
                f"Evaluation disabled; could not read {metadata_path}: {error}"
            )
            return None

    @staticmethod
    def quaternion_to_yaw(orientation):
        """Convert a ROS quaternion to planar yaw in radians."""
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return float(np.arctan2(sin_yaw, cos_yaw))

    @staticmethod
    def wrapped_angle_difference(angle, reference):
        """Return the signed shortest angular difference in radians."""
        return float(np.arctan2(
            np.sin(angle - reference),
            np.cos(angle - reference),
        ))

    def get_evaluation_time(self, msg):
        """Prefer simulation time from odometry, with monotonic time as fallback."""
        stamp = msg.header.stamp
        odometry_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if self.evaluation_clock_source is None:
            self.evaluation_clock_source = (
                "odometry_stamp" if odometry_time > 0.0 else "monotonic"
            )

        if self.evaluation_clock_source == "odometry_stamp":
            return odometry_time
        return time.monotonic()

    def odom_callback(self, msg):
        """Track time, distance, trajectory, progress, and yaw evidence."""
        position = msg.pose.pose.position
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_robot_pose = {
            "x": float(position.x),
            "y": float(position.y),
            "yaw": yaw,
        }
        if (
            not self.robot_trajectory
            or np.hypot(
                position.x - self.robot_trajectory[-1][0],
                position.y - self.robot_trajectory[-1][1],
            ) >= 0.02
        ):
            self.robot_trajectory.append((float(position.x), float(position.y)))

        if not self.evaluation_enabled or self.evaluation_complete:
            return

        current_time = self.get_evaluation_time(msg)
        self.evaluation_last_time = current_time
        self.evaluation_last_position = {
            "x": float(position.x),
            "y": float(position.y),
        }
        if self.evaluation_previous_position is not None:
            delta_x = (
                self.evaluation_last_position["x"]
                - self.evaluation_previous_position["x"]
            )
            delta_y = (
                self.evaluation_last_position["y"]
                - self.evaluation_previous_position["y"]
            )
            self.total_distance += float(np.hypot(delta_x, delta_y))
        self.evaluation_previous_position = dict(self.evaluation_last_position)

        if not self.evaluation_started:
            self.evaluation_started = True
            self.evaluation_start_time = current_time
            self.evaluation_start_position = dict(self.evaluation_last_position)
            self.evaluation_start_yaw = yaw
            self.evaluation_previous_yaw = yaw
            self.get_logger().info(
                "Evaluation started at "
                f"x={position.x:.3f}, y={position.y:.3f}, "
                f"yaw={np.rad2deg(yaw):.1f} deg"
            )
        else:
            yaw_jump = abs(self.wrapped_angle_difference(
                yaw, self.evaluation_previous_yaw
            ))
            yaw_deviation = abs(self.wrapped_angle_difference(
                yaw, self.evaluation_start_yaw
            ))
            self.max_yaw_jump = max(self.max_yaw_jump, yaw_jump)
            self.max_yaw_deviation = max(
                self.max_yaw_deviation, yaw_deviation
            )

            if not self.collision_detected and (
                yaw_jump >= self.yaw_jump_threshold
                or yaw_deviation >= self.yaw_deviation_threshold
            ):
                self.collision_detected = True
                self.collision_reason = (
                    "yaw_jump"
                    if yaw_jump >= self.yaw_jump_threshold
                    else "yaw_deviation"
                )
                self.collision_elapsed_seconds = max(
                    0.0, current_time - self.evaluation_start_time
                )
                self.get_logger().warning(
                    "Possible can collision detected: "
                    f"yaw jump={np.rad2deg(yaw_jump):.1f} deg, "
                    f"yaw deviation={np.rad2deg(yaw_deviation):.1f} deg"
                )

            self.evaluation_previous_yaw = yaw

        finish_x = self.last_can_row_x - self.finish_x_tolerance
        if position.x >= finish_x:
            self.finish_evaluation(
                reached_last_row=True,
                reason="reached_last_can_row",
            )
            if self.stop_on_evaluation_complete:
                self.move_2D()

    def finish_evaluation(self, reached_last_row, reason):
        """Write one evaluation result for this node run."""
        if not self.evaluation_enabled or self.evaluation_saved:
            return

        self.evaluation_complete = True
        elapsed_seconds = None
        if (
            self.evaluation_started
            and self.evaluation_start_time is not None
            and self.evaluation_last_time is not None
        ):
            elapsed_seconds = max(
                0.0, self.evaluation_last_time - self.evaluation_start_time
            )

        self.save_can_layout_overview()

        success = bool(reached_last_row and not self.collision_detected)
        result = {
            "status": "completed" if reached_last_row else "incomplete",
            "success": success,
            "reached_last_can_row": bool(reached_last_row),
            "elapsed_seconds": (
                round(elapsed_seconds, 6)
                if elapsed_seconds is not None
                else None
            ),
            "total_distance_meters": round(self.total_distance, 6),
            "collision_detected": bool(self.collision_detected),
            "collision_reason": self.collision_reason,
            "collision_elapsed_seconds": self.collision_elapsed_seconds,
            "max_yaw_jump_degrees": round(
                float(np.rad2deg(self.max_yaw_jump)), 3
            ),
            "max_yaw_deviation_degrees": round(
                float(np.rad2deg(self.max_yaw_deviation)), 3
            ),
            "yaw_jump_threshold_degrees": round(
                float(np.rad2deg(self.yaw_jump_threshold)), 3
            ),
            "yaw_deviation_threshold_degrees": round(
                float(np.rad2deg(self.yaw_deviation_threshold)), 3
            ),
            "last_can_row_x": self.last_can_row_x,
            "finish_x_tolerance": self.finish_x_tolerance,
            "start_position": self.evaluation_start_position,
            "finish_position": self.evaluation_last_position,
            "clock_source": self.evaluation_clock_source,
            "reason": reason,
            "scenario": self.environment_metadata.get("scenario"),
            "saved_at": datetime.now().astimezone().isoformat(),
        }

        result_numbers = [
            int(path.stem.rsplit("_", 1)[1])
            for path in self.debug_run_directory.glob("evaluation_*.json")
            if path.stem.rsplit("_", 1)[1].isdigit()
        ]
        result_number = max(result_numbers, default=0) + 1
        result_path = self.debug_run_directory / (
            f"evaluation_{result_number:04d}.json"
        )
        try:
            result_path.write_text(
                json.dumps(result, indent=2) + "\n",
                encoding="utf-8",
            )
            self.evaluation_saved = True
        except OSError as error:
            self.get_logger().warning(
                f"Could not save evaluation result to {result_path}: {error}"
            )
            return

        verdict = "PASS" if success else (
            "FAIL" if reached_last_row else "INCOMPLETE"
        )
        elapsed_text = (
            f"{elapsed_seconds:.3f} s"
            if elapsed_seconds is not None
            else "not available"
        )
        self.get_logger().info(
            f"Evaluation {verdict}: time={elapsed_text}, "
            f"distance={self.total_distance:.3f} m, "
            f"collision={self.collision_detected}; saved to {result_path}"
        )


    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        self.last_scan = np.array(msg.ranges)[::4]  # 2 degree given N = 720, step =4 
       
    """
    LiDAR index 0       approximately -180  robot front
    LiDAR index 45      approximately  -90  robot left
    LiDAR index 90                      0   robot back
    LiDAR index 135     approximately  +90  robot right
    LiDAR index 179     approximately +178  robot front
    
    """
    def timer_callback(self):
        
        if self.evaluation_complete and self.stop_on_evaluation_complete:
            self.move_2D()
            return

        if self.last_scan is None:
            return

        scan = np.asarray(self.last_scan, dtype=float).reshape(-1)
        if len(scan) < 4:
            self.move_2D()
            return

        points, indices, angles, ranges = self.get_front_scan(scan)
        targets = []

        # Check the space directly in front of the robot.
        front_clearance = 0.12
        blocked = np.any(
            (points[:, 0] > 0)
            & (points[:, 0] < self.stop_distance)
            & (np.abs(points[:, 1]) < front_clearance)
        )

        if not blocked:
            if self.avoiding:
                self.clear_count += 1

            if self.avoiding and self.clear_count < 8:
                # Wait for a few clear readings before going straight again.
                vx, vy = self.last_command
                target = np.array([vx, vy])
            else:
                self.avoiding = False
                self.clear_count = 0
                self.gap_heading = None
                
                self.side = 0
                self.side_lock_count = 0
                self.missing_count = 0
                self.last_command = (0.0, 0.0)
                vx, vy = 0.3, 0.0
                target = np.array([self.look_ahead, 0.0])

        else:
            self.avoiding = True
            self.clear_count = 0
            targets = self.find_gap_targets(points, indices)
            target = self.choose_gap(targets)

            if target is not None:
                self.save_gap_decision_plot(points, target)

                # Move towards the middle of the selected gap.
                direction = target / np.linalg.norm(target)
                vx = 0.18 * direction[0]
                vy = 0.18 * direction[1]
                self.gap_heading = np.arctan2(target[1], target[0])
                self.missing_count = 0

                if self.gap_heading > np.deg2rad(5):
                    self.side = 1
                elif self.gap_heading < np.deg2rad(-5):
                    self.side = -1
            else:
                vx, vy = self.get_sidestep_command(points, angles, ranges)
                target = np.array([vx, vy])

            self.last_command = (vx, vy)

       
        print(f"Moving to x={vx}, y={vy}")
        self.move_2D(x=vx, y=vy, turn=0.0)

    def get_front_scan(self, scan):
        # Keep the original scan order: right side -> front -> left side.
        quarter = len(scan) // 4
        front = np.concatenate((scan[3 * quarter:], scan[:quarter + 1]))
        angles = np.linspace(-np.pi / 2, np.pi / 2, len(front))

        ranges = np.nan_to_num(front, nan=0.0, posinf=self.look_ahead, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.look_ahead)

        valid = np.isfinite(front) & (front >= 0.05) & (front < self.look_ahead)
        indices = np.flatnonzero(valid)
        x = front[valid] * np.cos(angles[valid])
        y = front[valid] * np.sin(angles[valid])
        points = np.column_stack((x, y))
        return points, indices, angles, ranges
    
    def required_clearance_for_direction(self, direction):
        """Project the fixed-heading chassis across a travel direction."""
        return (
            self.robot_width * abs(direction[0])
            + self.robot_length * abs(direction[1])
            + 2.0 * self.safety_margin
        )



    def find_gap_targets(self, points, indices):
        # Put neighbouring readings from the same can into one group.
        cans = []
        for i in range(len(points)):
            if i == 0:
                cans.append([points[i]])
                continue

            distance = np.linalg.norm(points[i] - points[i - 1])
            if indices[i] != indices[i - 1] + 1 or distance > 0.12:
                cans.append([points[i]])
            else:
                cans[-1].append(points[i])

        self.debug_clusters = [np.asarray(can) for can in cans]
        self.debug_candidate_gaps = []

        targets = []
        for i in range(len(cans) - 1):
            right_edge = cans[i][-1]
            left_edge = cans[i + 1][0]
            gap = left_edge - right_edge
            midpoint = right_edge + 0.5 * gap
            distance = np.linalg.norm(midpoint)

            if midpoint[0] <= 0 or distance == 0:
                continue

            heading = np.arctan2(midpoint[1], midpoint[0])

            # Don't consider gaps that are mostly beside the robot
            # this was the change that helped it to get out of the loop
            if abs(heading) > np.deg2rad(85): # originally 60
                continue

            direction = midpoint / distance
            # The robot keeps its +x heading, so the opening it can pass through
            # is the lateral (+/-y) surface-to-surface separation. The diagonal
            # vector to the midpoint is not the robot heading.
            lateral_width = abs(gap[1])
            if lateral_width < self.required_gap:
                continue

            # Separately check the swept corridor along the diagonal translation.
            corridor_required = self.required_clearance_for_direction(direction)
            along = points @ direction
            across = np.abs(
                direction[0] * points[:, 1]
                - direction[1] * points[:, 0]
            )
            blocked = np.any(
                (along > 0.05)
                & (along < distance - 0.05)
                & (across < corridor_required / 2)
            )
            if not blocked:
                heading = abs(np.arctan2(midpoint[1], midpoint[0]))
                targets.append((heading, midpoint))
                self.debug_candidate_gaps.append(
                    (right_edge.copy(), left_edge.copy(), midpoint.copy())
                )

        return targets

    def robot_to_world(self, point):
        """Transform a LiDAR-origin point in base axes into odom."""
        pose = self.latest_robot_pose
        cosine = np.cos(pose["yaw"])
        sine = np.sin(pose["yaw"])
        # get_front_scan has already expressed scan angles in base axes, but
        # their origin is still the LiDAR, not the robot base.
        local_x = float(point[0]) + self.lidar_offset_x
        local_y = float(point[1]) + self.lidar_offset_y
        return np.array([
            pose["x"] + cosine * local_x - sine * local_y,
            pose["y"] + sine * local_x + cosine * local_y,
        ])

    def record_overview_decision(self, chosen_target):
        """Add one chosen gap and its perpendicular clearance to the overview."""
        if self.latest_robot_pose is None or self.environment_metadata is None:
            return

        chosen_gap = None
        for right_edge, left_edge, midpoint in self.debug_candidate_gaps:
            if np.allclose(midpoint, chosen_target):
                chosen_gap = (right_edge, left_edge, midpoint)
                break
        if chosen_gap is None:
            return

        right_edge, left_edge, midpoint = chosen_gap
        gap_vector = left_edge - right_edge
        measured_width = abs(gap_vector[1])
        width_start = np.array([
            midpoint[0], midpoint[1] - 0.5 * measured_width
        ])
        width_end = np.array([
            midpoint[0], midpoint[1] + 0.5 * measured_width
        ])
        required_width = self.required_gap

        self.overview_decisions.append({
            "robot": [
                self.latest_robot_pose["x"],
                self.latest_robot_pose["y"],
            ],
            "target": self.robot_to_world(midpoint).tolist(),
            "gap_right": self.robot_to_world(right_edge).tolist(),
            "gap_left": self.robot_to_world(left_edge).tolist(),
            "width_start": self.robot_to_world(width_start).tolist(),
            "width_end": self.robot_to_world(width_end).tolist(),
            "width": float(measured_width),
            "required_width": float(required_width),
        })
        self.save_can_layout_overview()

    def save_can_layout_overview(self):
        """Redraw can_layout.jpg with the cumulative path and gap decisions."""
        if self.environment_metadata is None:
            return

        can_positions = self.environment_metadata.get("can_positions", [])
        if not can_positions:
            return

        try:
            import matplotlib
            matplotlib.use("Agg", force=True)
            import matplotlib.pyplot as plt

            can_x = np.array([position["x"] for position in can_positions])
            can_y = np.array([position["y"] for position in can_positions])
            fig, ax = plt.subplots(figsize=(10, 12))
            ax.scatter(
                can_y,
                can_x,
                s=65,
                color="firebrick",
                edgecolors="black",
                linewidths=0.35,
                label="Cans",
                zorder=2,
            )

            all_x = list(can_x)
            all_y = list(can_y)
            if self.robot_trajectory:
                trajectory = np.asarray(self.robot_trajectory)
                ax.plot(
                    trajectory[:, 1],
                    trajectory[:, 0],
                    color="navy",
                    linewidth=2.0,
                    alpha=0.8,
                    label="Robot trajectory",
                    zorder=3,
                )
                ax.scatter(
                    trajectory[0, 1],
                    trajectory[0, 0],
                    marker="o",
                    s=80,
                    color="deepskyblue",
                    edgecolors="black",
                    label="Start",
                    zorder=6,
                )
                ax.scatter(
                    trajectory[-1, 1],
                    trajectory[-1, 0],
                    marker="^",
                    s=100,
                    color="royalblue",
                    edgecolors="black",
                    label="Latest robot pose",
                    zorder=7,
                )
                all_x.extend(trajectory[:, 0])
                all_y.extend(trajectory[:, 1])

            for decision_number, decision in enumerate(
                self.overview_decisions, start=1
            ):
                robot = np.asarray(decision["robot"])
                target = np.asarray(decision["target"])
                gap_right = np.asarray(decision["gap_right"])
                gap_left = np.asarray(decision["gap_left"])
                width_start = np.asarray(decision["width_start"])
                width_end = np.asarray(decision["width_end"])

                ax.plot(
                    [gap_right[1], gap_left[1]],
                    [gap_right[0], gap_left[0]],
                    "--",
                    color="darkorange",
                    linewidth=1.2,
                    alpha=0.75,
                    label=("Detected surface-to-surface gap" if decision_number == 1 else None),
                    zorder=4,
                )
                ax.scatter(
                    [gap_right[1], gap_left[1]],
                    [gap_right[0], gap_left[0]],
                    marker="x",
                    s=38,
                    color="darkorange",
                    zorder=6,
                )
                ax.plot(
                    [width_start[1], width_end[1]],
                    [width_start[0], width_end[0]],
                    color="limegreen",
                    linewidth=4.0,
                    label=(
                        "Measured lateral width (perpendicular to fixed heading)"
                        if decision_number == 1
                        else None
                    ),
                    zorder=5,
                )
                ax.annotate(
                    "",
                    xy=(target[1], target[0]),
                    xytext=(robot[1], robot[0]),
                    arrowprops={
                        "arrowstyle": "-|>",
                        "color": "royalblue",
                        "linewidth": 1.5,
                        "alpha": 0.7,
                    },
                    zorder=5,
                )
                ax.scatter(
                    robot[1],
                    robot[0],
                    marker="s",
                    s=25,
                    color="royalblue",
                    label="Decision pose" if decision_number == 1 else None,
                    zorder=6,
                )
                width_midpoint = 0.5 * (width_start + width_end)
                ax.annotate(
                    f"D{decision_number}: {decision['width']:.2f} ≥ {decision['required_width']:.2f} m",
                    xy=(width_midpoint[1], width_midpoint[0]),
                    xytext=(4, 4),
                    textcoords="offset points",
                    fontsize=7,
                    color="darkgreen",
                    zorder=8,
                )
                for point in (
                    robot, target, gap_right, gap_left, width_start, width_end
                ):
                    all_x.append(point[0])
                    all_y.append(point[1])

            if self.overview_decisions:
                ax.plot(
                    [], [],
                    color="royalblue",
                    linewidth=1.5,
                    label="Chosen travel direction",
                )

            y_min, y_max = min(all_y), max(all_y)
            x_min, x_max = min(all_x), max(all_x)
            ax.plot(
                [y_min - 0.3, y_max + 0.3],
                [self.last_can_row_x, self.last_can_row_x],
                ":",
                color="purple",
                linewidth=2.0,
                label="Last can row / finish",
                zorder=1,
            )
            padding = 0.4
            ax.set_xlim(y_max + padding, y_min - padding)
            ax.set_ylim(min(-padding, x_min - padding), x_max + padding)
            ax.set_aspect("equal", adjustable="box")
            ax.set_title(
                "Cumulative obstacle-avoidance decisions\n"
                "Fixed yaw: labels show lateral measured ≥ required width"
            )
            ax.set_xlabel("y (m, +left)")
            ax.set_ylabel("x (m, +forward)")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right", fontsize=8)
            fig.tight_layout()

            layout_path = self.debug_run_directory / "can_layout.jpg"
            temporary_path = self.debug_run_directory / ".can_layout.tmp.jpg"
            fig.savefig(temporary_path, format="jpeg", dpi=150)
            plt.close(fig)
            temporary_path.replace(layout_path)
        except Exception as error:
            self.get_logger().warning(
                f"Could not update cumulative can layout: {error}"
            )


    def save_gap_decision_plot(self, points, chosen_target):
        '''Save the latest gap-selection decision as a JPEG image.'''
        if not self.save_debug_plot:
            return

        now = time.monotonic()
        if now - self.last_debug_plot_time < self.debug_plot_interval:
            return
        self.last_debug_plot_time = now

        try:
            # The Agg backend writes image files without opening a GUI window.
            import matplotlib
            matplotlib.use("Agg", force=True)
            import matplotlib.pyplot as plt

            self.record_overview_decision(chosen_target)

            fig, ax = plt.subplots(figsize=(8, 8))

            # Grey dots are every valid LiDAR return used by the controller.
            if len(points) > 0:
                ax.scatter(
                    points[:, 1],
                    points[:, 0],
                    s=16,
                    color="0.65",
                    alpha=0.55,
                    label="LiDAR points",
                    zorder=1,
                )

            # Larger coloured dots show the clusters made from those returns.
            colours = plt.get_cmap("tab10")
            for cluster_id, cluster in enumerate(self.debug_clusters):
                if len(cluster) == 0:
                    continue
                ax.scatter(
                    cluster[:, 1],
                    cluster[:, 0],
                    s=38,
                    color=colours(cluster_id % 10),
                    edgecolors="black",
                    linewidths=0.3,
                    label="Clustered points" if cluster_id == 0 else None,
                    zorder=2,
                )

            chosen_gap = None
            for gap_id, (right_edge, left_edge, midpoint) in enumerate(
                self.debug_candidate_gaps
            ):
                # Orange dashed lines are all gaps accepted by the checks.
                ax.plot(
                    [right_edge[1], left_edge[1]],
                    [right_edge[0], left_edge[0]],
                    "--",
                    color="orange",
                    linewidth=1.5,
                    label="Candidate gaps" if gap_id == 0 else None,
                    zorder=3,
                )
                ax.scatter(
                    midpoint[1],
                    midpoint[0],
                    marker="x",
                    s=50,
                    color="orange",
                    zorder=4,
                )
                if np.allclose(midpoint, chosen_target):
                    chosen_gap = (right_edge, left_edge)

            # Green is the lateral opening perpendicular to the fixed +x heading.
            if chosen_gap is not None:
                right_edge, left_edge = chosen_gap
                midpoint = 0.5 * (right_edge + left_edge)
                gap_vector = left_edge - right_edge
                measured_width = abs(gap_vector[1])
                width_start = np.array([
                    midpoint[0], midpoint[1] - 0.5 * measured_width
                ])
                width_end = np.array([
                    midpoint[0], midpoint[1] + 0.5 * measured_width
                ])
                required_width = self.required_gap
                ax.plot(
                    [width_start[1], width_end[1]],
                    [width_start[0], width_end[0]],
                    color="limegreen",
                    linewidth=4.0,
                    label=(
                        f"Measured lateral {measured_width:.2f} m / "
                        f"required {required_width:.2f} m"
                    ),
                    zorder=5,
                )

            ax.scatter(
                chosen_target[1],
                chosen_target[0],
                marker="*",
                s=180,
                color="limegreen",
                edgecolors="black",
                label="Chosen midpoint",
                zorder=6,
            )
            ax.arrow(
                0.0,
                0.0,
                chosen_target[1],
                chosen_target[0],
                width=0.008,
                head_width=0.07,
                length_includes_head=True,
                color="green",
                alpha=0.75,
                zorder=4,
            )
            ax.scatter(
                0.0,
                0.0,
                marker="^",
                s=100,
                color="royalblue",
                label="Robot",
                zorder=7,
            )

            ax.set_title("Obstacle avoidance gap decision")
            ax.set_xlabel("y (m, +left)")
            ax.set_ylabel("x (m, +forward)")
            # Robot-centric view: +x is up and +y is to the left.
            ax.set_xlim(self.look_ahead + 0.05, -self.look_ahead - 0.05)
            ax.set_ylim(-0.05, self.look_ahead + 0.05)
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right")

            self.debug_plot_number += 1
            debug_plot_path = self.debug_run_directory / (
                f"gap_decision_{self.debug_plot_number:04d}.jpg"
            )
            fig.tight_layout()
            fig.savefig(debug_plot_path, format="jpeg", dpi=150)
            plt.close(fig)
            self.get_logger().debug(
                f"Saved gap decision plot to {debug_plot_path}"
            )
        except Exception as error:
            # Plotting is diagnostic only and must never stop the controller.
            self.get_logger().warning(
                f"Could not save gap decision plot: {error}"
            )

    def choose_gap(self, targets):
        if not targets:
            return None

        # First check if there is a gap almost straight ahead
        for _, target in targets:
            if abs(target[1]) < 0.15:
                self.side = 0
                print(f"Straight ahead target: {target}")
                return target

        # If no straight gap, choose the opposite side
        wanted_side = self.gap_side

        possible = []

        for _, target in targets:
            if target[1] * wanted_side > 0:
                possible.append(target)

        if possible:
            # Pick the gap closest to straight ahead
            best_target = possible[0]

            for target in possible:
                if abs(np.arctan2(target[1], target[0])) < abs(
                    np.arctan2(best_target[1], best_target[0])
                ):
                    best_target = target

            #self.gap_side *= -1
            print(f"Best Target: {best_target}")
            return best_target

        return None

    def get_sidestep_command(self, points, angles, ranges):
        right = np.median(ranges[angles < np.deg2rad(-30)])
        left = np.median(ranges[angles > np.deg2rad(30)])

        if self.side_lock_count > 0:
            self.side_lock_count -= 1

        if self.side == 0:
            if left > right:
                self.side = 1
            else:
                self.side = -1
        elif (
            self.side_lock_count == 0
            and self.side == 1
            and right > left + 0.20
        ):
            self.side = -1
        elif (
            self.side_lock_count == 0
            and self.side == -1
            and left > right + 0.20
        ):
            self.side = 1

        # Check the swept corridor in the current sideways direction. This is
        # independent of the generated scenario: any obstacle close to the
        # side of the chassis causes the robot to reverse before contact.
        half_robot_clearance = self.required_gap / 2
        side_stop_distance = 0.30
        distance_in_side_direction = points[:, 1] * self.side
        side_blocked = np.any(
            (distance_in_side_direction > 0.0)
            & (distance_in_side_direction < side_stop_distance)
            & (np.abs(points[:, 0]) < half_robot_clearance)
        )

        if side_blocked:
            opposite_distance = points[:, 1] * -self.side
            opposite_blocked = np.any(
                (opposite_distance > 0.0)
                & (opposite_distance < side_stop_distance)
                & (np.abs(points[:, 0]) < half_robot_clearance)
            )

            if not opposite_blocked:
                self.side *= -1
                # Hold the escape direction long enough to move away from the
                # detected side wall instead of immediately switching back.
                self.side_lock_count = 12
            else:
                return -0.08, 0.0

        if self.missing_count >= 4:
            self.gap_heading = None

        # Reverse slightly if a can is very close in front.
        front_points = points[
            (points[:, 0] > 0)
            & (np.abs(points[:, 1]) < self.required_gap / 2)
        ]
        vx = 0.0
        if len(front_points) > 0 and np.min(front_points[:, 0]) < 0.20:
            vx = -0.08

        vy = 0.25 * self.side
        return vx, vy


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(obstacle_avoidance_node)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_avoidance_node.finish_evaluation(
            reached_last_row=False,
            reason="node_shutdown",
        )
        obstacle_avoidance_node.move_2D()
        obstacle_avoidance_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
