import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Libraries for easier visualisation what the robot is doing in gazebo
from geometry_msgs.msg import Twist, Point 
from visualization_msgs.msg import Marker, MarkerArray

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

        self.last_scan = None
        self.last_angles = None

        self.pub_debug = self.create_publisher(MarkerArray, "debug_markers",10) # Easier visual debugging

        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    # Helper function to publish visualisation points
    def show_decision(self, obstacle_points, possible_targets, chosen_target):
        markers = MarkerArray()

        # Remove markers from the previous decision.
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        stamp = self.get_clock().now().to_msg()

        # Red: LiDAR obstacle points
        obstacles = Marker()
        obstacles.header.frame_id = "base_link"
        obstacles.header.stamp = stamp
        obstacles.ns = "obstacles"
        obstacles.id = 0
        obstacles.type = Marker.SPHERE_LIST
        obstacles.action = Marker.ADD
        obstacles.scale.x = 0.04
        obstacles.scale.y = 0.04
        obstacles.scale.z = 0.04
        obstacles.color.r = 1.0
        obstacles.color.a = 1.0

        for x, y in obstacle_points:
            obstacles.points.append(
                Point(x=float(x), y=float(y), z=0.1)
            )

        markers.markers.append(obstacles)

        # Yellow: gap midpoints considered by the controller
        candidates = Marker()
        candidates.header.frame_id = "base_link"
        candidates.header.stamp = stamp
        candidates.ns = "gap_candidates"
        candidates.id = 1
        candidates.type = Marker.SPHERE_LIST
        candidates.action = Marker.ADD
        candidates.scale.x = 0.08
        candidates.scale.y = 0.08
        candidates.scale.z = 0.08
        candidates.color.r = 1.0
        candidates.color.g = 1.0
        candidates.color.a = 1.0

        for _, midpoint in possible_targets:
            candidates.points.append(
                Point(
                    x=float(midpoint[0]),
                    y=float(midpoint[1]),
                    z=0.12
                )
            )

        markers.markers.append(candidates)

        # Green: direction selected by the controller
        if chosen_target is not None:
            arrow = Marker()
            arrow.header.frame_id = "base_link"
            arrow.header.stamp = stamp
            arrow.ns = "chosen_direction"
            arrow.id = 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            arrow.points = [
                Point(x=0.0, y=0.0, z=0.15),
                Point(
                    x=float(chosen_target[0]),
                    y=float(chosen_target[1]),
                    z=0.15
                ),
            ]

            # Shaft diameter, head diameter and head length
            arrow.scale.x = 0.025
            arrow.scale.y = 0.06
            arrow.scale.z = 0.08

            arrow.color.g = 1.0
            arrow.color.a = 1.0

            markers.markers.append(arrow)

        self.pub_debug.publish(markers)

    def sub_scan_callback(self, msg):
        self.last_scan = np.array(msg.ranges)[::4]  # 2 degree given N = 720, step =4 
       
    """
    LiDAR index 0       approximately -180° → robot front
    LiDAR index 45      approximately  -90° → robot left
    LiDAR index 90                    0° → robot back
    LiDAR index 135     approximately +90° → robot right
    LiDAR index 179     approximately +178° → robot front
    
    """
    def timer_callback(self):
        """Controller loop"""
        if self.last_scan is None:
            return

        robot_width = 0.23
        safety_margin = 0.05
        required_gap = robot_width + (2.0 * safety_margin)
        # The first generated can is only 0.8 m from the spawn point. Start
        # avoiding early enough to account for the LiDAR update interval and
        # the part of the chassis that extends in front of the sensor.
        stop_distance = 0.9
        look_ahead = 1.5
        same_can_distance = 0.12
        release_clear_cycles = 8
        gap_memory_cycles = 4
        gap_match_tolerance = np.deg2rad(30.0)

        # Keep a small amount of state between timer calls. Without this, tiny
        # changes in a noisy scan can make the controller alternate between two
        # gaps (or between avoidance and straight-ahead motion) at 20 Hz.
        if not hasattr(self, "_avoidance_active"):
            self._avoidance_active = False
            self._clear_corridor_cycles = 0
            self._committed_gap_heading = None
            self._committed_side = 0
            self._missing_gap_cycles = 0
            self._last_avoidance_command = (0.0, 0.0)

        scan = np.asarray(self.last_scan, dtype=float).reshape(-1)
        if scan.size < 4:
            self.move_2D()
            return

        # The scan starts behind the robot. Reorder its forward half so that
        # points run continuously from the robot's right side to its left side.
        quarter = scan.size // 4
        front_ranges = np.concatenate((
            scan[3 * quarter:],
            scan[:quarter + 1],
        ))
        front_angles = np.linspace(
            -np.pi / 2.0,
            np.pi / 2.0,
            front_ranges.size,
        )

        # Infinite ranges mean clear space. NaN and negative ranges are invalid,
        # so treat them conservatively when comparing the two side sectors.
        clear_ranges = np.nan_to_num(
            front_ranges,
            nan=0.0,
            posinf=look_ahead,
            neginf=0.0,
        )
        clear_ranges = np.clip(clear_ranges, 0.0, look_ahead)

        hit_mask = (
            np.isfinite(front_ranges)
            & (front_ranges >= 0.05)
            & (front_ranges < look_ahead)
        )
        hit_indices = np.flatnonzero(hit_mask)
        hit_ranges = front_ranges[hit_mask]
        hit_angles = front_angles[hit_mask]
        hit_points = np.column_stack((
            hit_ranges * np.cos(hit_angles),
            hit_ranges * np.sin(hit_angles),
        ))

        # Clear forward corridor: no nearby return lies inside the swept width
        # of the robot (including its safety margin).
        half_required_gap = required_gap / 2.0
        corridor_blocked = np.any(
            (hit_points[:, 0] > 0.0)
            & (hit_points[:, 0] < stop_distance)
            & (np.abs(hit_points[:, 1]) < half_required_gap)
        )
        if not corridor_blocked:
            if self._avoidance_active:
                self._clear_corridor_cycles += 1

                # A single clear scan is often just a noisy or missed return.
                # Continue the committed manoeuvre until the corridor has been
                # clear for several consecutive controller cycles.
                if self._clear_corridor_cycles < release_clear_cycles:
                    command_x, command_y = self._last_avoidance_command
                    chosen_target = np.array([command_x, command_y])
                    self.show_decision(hit_points, [], chosen_target)
                    self.move_2D(
                        x=command_x,
                        y=command_y,
                        turn=0.0,
                    )
                    return

            self._avoidance_active = False
            self._clear_corridor_cycles = 0
            self._committed_gap_heading = None
            self._committed_side = 0
            self._missing_gap_cycles = 0
            self._last_avoidance_command = (0.0, 0.0)
            chosen_target = np.array([look_ahead, 0.0])
            self.show_decision(hit_points, [], chosen_target)
            self.move_2D(x=0.3, y=0.0, turn=0.0)
            return

        self._avoidance_active = True
        self._clear_corridor_cycles = 0

        # Group only consecutive LiDAR returns. Requiring neighbouring beam
        # indices prevents separate objects across an unmeasured region from
        # being accidentally merged into one can.
        cans = []
        previous_index = None
        for index, point in zip(hit_indices, hit_points):
            starts_new_can = (
                previous_index is None
                or index != previous_index + 1
                or np.linalg.norm(point - cans[-1][-1]) > same_can_distance
            )
            if starts_new_can:
                cans.append([point])
            else:
                cans[-1].append(point)
            previous_index = index

        # Measure each opening between neighbouring cans. The first tuple item
        # is the absolute heading, so min() favours the most forward gap.
        possible_targets = []
        for right_can, left_can in zip(cans, cans[1:]):
            right_edge = right_can[-1]
            left_edge = left_can[0]
            gap_vector = left_edge - right_edge
            midpoint = right_edge + (0.5 * gap_vector)

            target_distance = np.linalg.norm(midpoint)
            if midpoint[0] <= 0.0 or target_distance == 0.0:
                continue

            direction = midpoint / target_distance

            # Euclidean endpoint distance alone can report a false opening
            # when one can is much farther away than the other. Measure the
            # usable width perpendicular to the route instead.
            transverse_gap = abs(
                (direction[0] * gap_vector[1])
                - (direction[1] * gap_vector[0])
            )
            if transverse_gap < required_gap:
                continue

            # Reject a midpoint if the straight route to it passes through a
            # nearer can. Do not test the final few centimetres because the two
            # points that define the opening lie at that end of the route.
            projections = hit_points @ direction
            lateral_distances = np.abs(
                (direction[0] * hit_points[:, 1])
                - (direction[1] * hit_points[:, 0])
            )
            route_blocked = np.any(
                (projections > 0.05)
                & (projections < target_distance - 0.05)
                & (lateral_distances < half_required_gap)
            )
            if route_blocked:
                continue

            heading = abs(np.arctan2(midpoint[1], midpoint[0]))
            possible_targets.append((heading, midpoint))

        chosen_target = None
        if possible_targets:
            best_target = min(
                possible_targets,
                key=lambda target: target[0],
            )

            if self._committed_gap_heading is None:
                _, chosen_target = best_target
            else:
                # Track the candidate nearest to the previously selected
                # heading, rather than reranking unrelated gaps every cycle.
                matching_target = min(
                    possible_targets,
                    key=lambda target: abs(
                        np.arctan2(target[1][1], target[1][0])
                        - self._committed_gap_heading
                    ),
                )
                matching_heading = np.arctan2(
                    matching_target[1][1],
                    matching_target[1][0],
                )
                heading_change = abs(
                    matching_heading - self._committed_gap_heading
                )

                if heading_change <= gap_match_tolerance:
                    _, chosen_target = matching_target
                    self._missing_gap_cycles = 0
                else:
                    self._missing_gap_cycles += 1
                    if self._missing_gap_cycles >= gap_memory_cycles:
                        _, chosen_target = best_target

        else:
            self._missing_gap_cycles += 1

        if chosen_target is not None:
            target_distance = np.linalg.norm(chosen_target)
            if target_distance > 0.0:
                direction = chosen_target / target_distance
                chosen_heading = np.arctan2(
                    chosen_target[1],
                    chosen_target[0],
                )
                self._committed_gap_heading = chosen_heading
                if abs(chosen_heading) > np.deg2rad(5.0):
                    self._committed_side = 1 if chosen_heading > 0.0 else -1
                self._missing_gap_cycles = 0
                command_x = 0.18 * direction[0]
                command_y = 0.18 * direction[1]
                self._last_avoidance_command = (command_x, command_y)
                self.show_decision(
                    hit_points,
                    possible_targets,
                    chosen_target,
                )
                self.move_2D(
                    x=command_x,
                    y=command_y,
                    turn=0.0,
                )
                return

        # No traversable gap: sidestep toward the side with more free range.
        right_mask = front_angles < np.deg2rad(-30.0)
        left_mask = front_angles > np.deg2rad(30.0)
        right_clearance = np.median(clear_ranges[right_mask])
        left_clearance = np.median(clear_ranges[left_mask])

        clearer_side = 1 if left_clearance > right_clearance else -1

        # Keep sidestepping in the committed direction unless the other side
        # is meaningfully clearer. The margin prevents left/right toggling when
        # both median clearances differ only by LiDAR noise.
        if self._committed_side == 0:
            self._committed_side = clearer_side
        else:
            committed_clearance = (
                left_clearance
                if self._committed_side > 0
                else right_clearance
            )
            alternative_clearance = (
                right_clearance
                if self._committed_side > 0
                else left_clearance
            )
            if alternative_clearance > committed_clearance + 0.20:
                self._committed_side *= -1

        if self._missing_gap_cycles >= gap_memory_cycles:
            self._committed_gap_heading = None

        lateral_speed = 0.25 * self._committed_side

        # Back away slightly if the robot is already extremely close. This
        # lets it recover instead of continuing to push against a can.
        blocking_distances = hit_points[
            (hit_points[:, 0] > 0.0)
            & (np.abs(hit_points[:, 1]) < half_required_gap),
            0,
        ]
        nearest_blocker = (
            np.min(blocking_distances)
            if blocking_distances.size
            else look_ahead
        )
        forward_speed = -0.08 if nearest_blocker < 0.20 else 0.0

        self._last_avoidance_command = (forward_speed, lateral_speed)
        chosen_target = np.array([forward_speed, lateral_speed])
        self.show_decision(hit_points, possible_targets, chosen_target)
        self.move_2D(x=forward_speed, y=lateral_speed, turn=0.0)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
