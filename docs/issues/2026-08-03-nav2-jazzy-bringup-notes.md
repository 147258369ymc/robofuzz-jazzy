# nav2_jazzy bring-up notes (Task 7)

Integration + environment orchestration for the `nav2_jazzy` fuzzing target:
`run_target.sh nav2_jazzy` launches the TurtleBot4 base (verbatim copy of the
verified `run_slam_toolbox` split-headless bring-up), then rides the nav2
navigable stack on top with ground-truth localization instead of amcl.

## Stack structure (`run_nav2` / `start_nav2_when_ready`)

`run_nav2` is a verbatim copy of `run_slam_toolbox`'s TB4 base bring-up:
- gz sim launch (`ros_gz_sim gz_sim.launch.py`, headless `-s -r`, xvfb + software GL)
- `/clock` bridge (`ros_gz_bridge parameter_bridge`)
- world defaults to `slam_arena`
- trap-based cleanup of sim/clock/nav2 PIDs
- TB4 spawn via `src_jazzy/launch/turtlebot4_fuzz_spawn.launch.py`

The only substitution: `start_slam_toolbox_when_ready` -> `start_nav2_when_ready`.

`start_nav2_when_ready` waits for `/scan`, then launches (instead of slam_toolbox):
1. **Ground-truth map->odom TF** — `python3 src_jazzy/nav2_gt_tf.py --ros-args -p use_sim_time:=true`.
   Subscribes `/sim_ground_truth_pose` + `/odom`, broadcasts `map->odom` at 20 Hz.
   This bypasses amcl and the TB4 scan-frame / map->odom TF blood-debt that
   plagued the slam_toolbox target: nav2 localizes perfectly because we feed it
   the simulator truth.
   - TF math: `T_map_odom = T_gt * inverse(T_odom_base)` so `map->base_link`
     tracks truth even as wheel odom drifts. Falls back to `map->odom = T_gt`
     if no `/odom` yet, or identity before any data (robot spawns at world
     origin x=y=yaw=0, so identity is the correct seed).
2. **map_server** — `nav2_map_server map_server` serving `${NAV2_MAP:-/work/maps/slam_arena.yaml}`,
   configured+activated by a dedicated `nav2_lifecycle_manager lifecycle_manager`
   (`node_names:=[map_server]`, `autostart:=true`). navigation_launch.py does
   NOT include map_server, so it must be started + lifecycle-activated here.
3. **nav2 navigation stack** — `ros2 launch nav2_bringup navigation_launch.py
   use_sim_time:=true [params_file:=$NAV2_PARAMS]` (planner + controller +
   bt_navigator + `/navigate_to_pose` action server). `params_file` is passed
   only when `NAV2_PARAMS` is set and the file exists; otherwise the built-in
   nav2 defaults are used.

Env overrides: `NAV2_MAP`, `NAV2_PARAMS`, `NAV2_RVIZ` (plus the shared
`TURTLEBOT4_*` base overrides).

## Step 1 — building the offline `slam_arena` static map

Run the already-verified slam stack, drive the robot around `slam_arena`, then
save the map with the nav2 map saver:

```bash
docker run --rm -it -v /home/ymc/RoboFuzz-jazzy:/work robofuzz-jazzy:latest \
  bash -lc 'source /opt/ros/jazzy/setup.bash; cd /work; \
    ./run_target.sh slam_toolbox_jazzy & \
    # wait for slam to map, drive/teleop or run a short campaign to traverse the arena, then:
    ros2 run nav2_map_server map_saver_cli -f /work/maps/slam_arena \
      --ros-args -p save_map_timeout:=10000.0'
```

Produces `maps/slam_arena.yaml` + `maps/slam_arena.pgm`.

Verify the map loads with non-zero dimensions:

```bash
python3 -c "import sys; sys.path.insert(0,'src_jazzy'); import nav2_map; \
  m=nav2_map.load_occupancy_map('maps/slam_arena.yaml'); \
  print(m.width, m.height, m.resolution)"
```

## Step 4 — smoke: stack up + TF continuity (the #1 risk)

```bash
docker run --rm -it --name robofuzz-jazzy-nav2-smoke \
  -v /home/ymc/RoboFuzz-jazzy:/work robofuzz-jazzy:latest \
  bash -lc 'source /opt/ros/jazzy/setup.bash; cd /work; ./run_target.sh nav2_jazzy & \
    sleep 90; ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true'
```

Expected: `tf2_echo map odom` prints continuously-updating transforms (not
"frame does not exist"). Because the ground-truth publisher seeds identity
before any data arrives, `map->odom` should exist from the moment
`nav2_gt_tf.py` starts — this is exactly the mitigation for the slam target's
map->odom blood-debt.

## Step 5 — smoke: one fuzzing round end-to-end

```bash
docker run --rm -it --name robofuzz-jazzy-nav2-e2e \
  -v /home/ymc/RoboFuzz-jazzy:/work -e NAV2_MAP=/work/maps/slam_arena.yaml \
  robofuzz-jazzy:latest \
  bash -lc 'source /opt/ros/jazzy/setup.bash; cd /work/src_jazzy; \
    python3 -u fuzzer.py --target-profile nav2_jazzy --schedule sequence \
    --seqlen 1 --maxloop 2 --interval 1.0 --no-cov \
    --logdir /work/logs_jazzy/nav2_smoke'
```

Expected: `/navigate_to_pose` action server ready, goal sent,
`states-0.bag` records `/robofuzz/nav2_goal` + `/robofuzz/nav2_result` +
`/sim_ground_truth_pose`, oracle prints a judgment line, and
`nav2_truth_sample_count > 0`.

## Smoke results

<!-- Filled in by the Part B smoke attempt below. -->

(See "Part B smoke attempt" section appended at the end of this file.)
