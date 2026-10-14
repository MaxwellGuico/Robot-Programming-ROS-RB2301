# CA2 runtime parameters

Build and run from the workspace:

```bash
colcon build --packages-select rb2301_ca2 --symlink-install
./ca2.sh                 # Gazebo simulation
./ca2.sh --run test1     # Real maze, first test
./ca2.sh --run test2     # Real maze, second test
```

Edit `params/ca2_runtime.yaml` to set the PD gains, carrot look-ahead distance,
and goal tolerance for the next run. The script reads this source YAML each time,
so changing these values does not require another build. Use decimal values such
as `3.0` for the numeric parameters. Gains must be finite and non-negative;
look-ahead distance and goal tolerance must be finite and positive.

You can override a setting when starting a run:

```bash
./ca2.sh --run test1 --ros-args -p kp:=2.5
```

Or change a setting from another terminal while the node is running:

```bash
source install/setup.bash
ros2 param set /waypoint kp 2.5
ros2 param set /waypoint look_ahead_dist 0.18
```

Each invocation creates `ca2_runs/<run>_<timestamp>_<unique suffix>/params.yaml`.
This file contains the effective settings, including overrides, and updates after
each accepted parameter change. The node prints `Runtime parameters saved to ...`
after each successful save, including changes made with `ros2 param set`. Reuse a snapshot
with:

```bash
./ca2.sh --run test1 --ros-args --params-file ca2_runs/<previous_run>/params.yaml
```

`test1` and `test2` always disable image creation, PNG saving, and opening the
image viewer. The terminal map still prints. Simulation and `--run full` use the
`generate_image` setting in the YAML. Real-maze speed remains capped at 0.4 m/s.

The default YAML is also installed under the package's `share/rb2301_ca2/params`
directory for use with `ros2 run ... --ros-args --params-file ...`. Direct
`ros2 run` invocations can request parameter saving with `--save-params <file>`.
