import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Libraries for easier visualisation what the robot is doing in gazebo
from geometry_msgs.msg import Twist, Point 

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

        robot_width = 0.23
        safety_margin = 0.05
        self.required_gap = robot_width + 2 * safety_margin
        self.stop_distance = 0.75 # og:0.9
        self.look_ahead = 1.5

        self.avoiding = False
        self.clear_count = 0
        self.gap_heading = None

        self.gap_side = -1   # -1 = start with right, +1 = start with left
        self.side = 0        # emergency sidestep direction

        self.side_lock_count = 0
        self.missing_count = 0
        self.last_command = (0.0, 0.0)

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
            if abs(heading) > np.deg2rad(60):
                continue

            direction = midpoint / distance
            # Measure the gap across the direction we want to travel.
            width = abs(direction[0] * gap[1] - direction[1] * gap[0])
            if width < self.required_gap:
                continue

            # Check that another can is not in the way of this gap.
            along = points @ direction
            across = np.abs(direction[0] * points[:, 1] - direction[1] * points[:, 0])
            blocked = np.any(
                (along > 0.05)
                & (along < distance - 0.05)
                & (across < self.required_gap / 2)
            )
            if not blocked:
                heading = abs(np.arctan2(midpoint[1], midpoint[0]))
                targets.append((heading, midpoint))

        return targets

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
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
