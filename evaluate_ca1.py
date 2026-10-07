"""Run the CA1 Gazebo world and measure an obstacle-avoidance controller.

Usage (after sourcing install/setup.bash):
    python3 evaluate_ca1.py --algo nhien --seeds 1 2 3 4 5
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


ROOT = Path(__file__).resolve().parent
RUNS = ROOT / "ca1_runs"


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Monitor(Node):
    def __init__(self, context):
        super().__init__("ca1_evaluation_monitor", context=context)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.finish_x = None
        self.first_stamp = None
        self.last_stamp = None
        self.first_yaw = None
        self.last_yaw = None
        self.last_position = None
        self.path_length = 0.0
        self.max_yaw_jump = 0.0
        self.max_yaw_deviation = 0.0
        self.collision = False
        self.collision_reason = None
        self.reached_finish = False
        self.odom_count = 0
        self.scan_count = 0
        self.last_progress_bucket = -1

    def on_scan(self, _msg):
        self.scan_count += 1

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0:
            stamp = time.monotonic()
        if self.first_stamp is None:
            self.first_stamp = stamp
            self.first_yaw = yaw
        else:
            self.path_length += math.hypot(
                pos.x - self.last_position[0], pos.y - self.last_position[1]
            )
            jump = abs(angle_delta(yaw, self.last_yaw))
            deviation = abs(angle_delta(yaw, self.first_yaw))
            self.max_yaw_jump = max(self.max_yaw_jump, jump)
            self.max_yaw_deviation = max(self.max_yaw_deviation, deviation)
            if not self.collision and (jump >= math.radians(20) or deviation >= math.radians(30)):
                self.collision = True
                self.collision_reason = "yaw_jump" if jump >= math.radians(20) else "yaw_deviation"
        self.last_stamp = stamp
        self.last_position = (float(pos.x), float(pos.y))
        self.last_yaw = yaw
        self.odom_count += 1
        self.reached_finish = self.finish_x is not None and pos.x >= self.finish_x
        bucket = int(self.elapsed // 10)
        if bucket > self.last_progress_bucket:
            self.last_progress_bucket = bucket
            print(
                f"  sim={self.elapsed:.1f}s x={pos.x:.2f}m "
                f"distance={self.path_length:.2f}m collision={self.collision}",
                flush=True,
            )

    @property
    def elapsed(self):
        if self.first_stamp is None or self.last_stamp is None:
            return None
        return max(0.0, self.last_stamp - self.first_stamp)


def stop_group(process):
    if process is None or process.poll() is not None:
        return
    for sig, delay in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=delay)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def run(seed, algo, batch_dir, sim_limit, wall_limit):
    env = os.environ.copy()
    env.update({
        "RB2301_OBSTACLE_SEED": str(seed),
        "RB2301_HEADLESS": "1",
        "GZ_PARTITION": f"rb2301_ca1_seed_{seed}",
        "ROS_DOMAIN_ID": str(90 + seed),
        "ROS_LOG_DIR": str(batch_dir / "ros_logs"),
        "MPLCONFIGDIR": str(batch_dir / "matplotlib"),
    })
    os.environ["ROS_LOG_DIR"] = env["ROS_LOG_DIR"]
    context = Context()
    rclpy.init(context=context, domain_id=90 + seed)
    monitor = Monitor(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(monitor)
    prior_run = (RUNS / "latest_run.txt").read_text().strip() if (RUNS / "latest_run.txt").exists() else None
    launch = controller = None
    reason = None
    run_dir = None
    start_wall = time.monotonic()
    with (batch_dir / f"seed_{seed}_launch.log").open("w") as launch_log, (
        batch_dir / f"seed_{seed}_{algo}.log"
    ).open("w") as controller_log:
        try:
            launch = subprocess.Popen(
                ["ros2", "launch", "rb2301_gz", "ca1_gazebo.launch.py"],
                env=env, stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            while time.monotonic() - start_wall < 35:
                if launch.poll() is not None:
                    raise RuntimeError("Gazebo launch exited early; see launch log")
                latest = (RUNS / "latest_run.txt").read_text().strip() if (RUNS / "latest_run.txt").exists() else None
                if latest and latest != prior_run and (Path(latest) / "environment.json").exists():
                    run_dir = Path(latest)
                    monitor.finish_x = float(json.loads((run_dir / "environment.json").read_text())["last_can_row_x"])
                    break
                time.sleep(0.2)
            if run_dir is None:
                raise RuntimeError("No generated world metadata found")
            controller = subprocess.Popen(
                ["ros2", "run", "rb2301_ca1", "obstacle_avoidance", "--ros-args",
                 "-p", f"algo:={algo}", "-p", "save_debug_plot:=false"],
                env=env, stdout=controller_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            while True:
                executor.spin_once(timeout_sec=0.2)
                if monitor.reached_finish:
                    reason = "reached_last_can_row"
                    break
                if monitor.elapsed is not None and monitor.elapsed >= sim_limit:
                    reason = "simulation_time_limit"
                    break
                if time.monotonic() - start_wall >= wall_limit:
                    reason = "wall_time_limit"
                    break
                if launch.poll() is not None:
                    reason = "Gazebo launch exited early"
                    break
                if controller.poll() is not None:
                    reason = "controller exited early"
                    break
        except Exception as exc:
            reason = str(exc)
        finally:
            stop_group(controller)
            stop_group(launch)
            executor.shutdown()
            monitor.destroy_node()
            rclpy.shutdown(context=context)

    result = {
        "seed": seed,
        "algo": algo,
        "elapsed_seconds": round(monitor.elapsed, 6) if monitor.elapsed is not None else None,
        "total_distance_meters": round(monitor.path_length, 6),
        "success": bool(monitor.reached_finish and not monitor.collision),
        "reached_last_can_row": monitor.reached_finish,
        "collision_detected": monitor.collision,
        "collision_reason": monitor.collision_reason,
        "max_yaw_jump_degrees": round(math.degrees(monitor.max_yaw_jump), 3),
        "max_yaw_deviation_degrees": round(math.degrees(monitor.max_yaw_deviation), 3),
        "finish_x": monitor.finish_x,
        "finish_position": monitor.last_position,
        "odom_count": monitor.odom_count,
        "scan_count": monitor.scan_count,
        "reason": reason,
        "world_directory": str(run_dir) if run_dir else None,
    }
    (batch_dir / f"seed_{seed}_result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"seed {seed}: {json.dumps(result)}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algo", default="nhien")
    parser.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    parser.add_argument("--sim-limit", type=float, default=120)
    parser.add_argument("--wall-limit", type=float, default=360)
    parser.add_argument("--batch-dir", type=Path)
    args = parser.parse_args()
    batch_dir = args.batch_dir or RUNS / f"{args.algo}_evaluation_{datetime.now():%Y%m%d_%H%M%S}"
    batch_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for seed in args.seeds:
        result_path = batch_dir / f"seed_{seed}_result.json"
        if result_path.exists():
            print(f"Reusing seed {seed} from {result_path}", flush=True)
            results.append(json.loads(result_path.read_text()))
        else:
            print(f"Starting seed {seed}", flush=True)
            results.append(run(seed, args.algo, batch_dir, args.sim_limit, args.wall_limit))
    valid = [r for r in results if r["elapsed_seconds"] is not None]
    summary = {
        "algo": args.algo,
        "seeds": args.seeds,
        "sim_limit_seconds": args.sim_limit,
        "results": results,
        "average_elapsed_seconds": sum(r["elapsed_seconds"] for r in valid) / len(valid) if valid else None,
        "average_distance_meters": sum(r["total_distance_meters"] for r in valid) / len(valid) if valid else None,
        "success_rate": sum(r["success"] for r in results) / len(results),
    }
    (batch_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Summary: {batch_dir / 'summary.json'}", flush=True)
    print(json.dumps({key: summary[key] for key in ("average_elapsed_seconds", "average_distance_meters", "success_rate")}, indent=2), flush=True)


if __name__ == "__main__":
    main()
