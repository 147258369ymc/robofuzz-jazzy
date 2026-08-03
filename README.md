# RoboFuzz Jazzy Modernization

This repository is a modernization fork of
[RoboFuzz](https://github.com/sslab-gatech/RoboFuzz), the ESEC/FSE 2022
fuzzing framework for ROS 2 robotic systems.

The goal of this fork is to port the user's improved RoboFuzz codebase from
the original Ubuntu 20.04 / ROS 2 Foxy / Gazebo Classic baseline to a modern
Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic environment.

## Current Scope

The active modernization targets are:

| Profile | Target | Baseline |
| --- | --- | --- |
| `turtlebot4_jazzy` | TurtleBot4 Standard simulator | ROS 2 Jazzy + Gazebo Harmonic |
| `px4_v117_jazzy` | PX4 SITL x500 | PX4 v1.17.0 + Gazebo Harmonic + ROS 2 bridge |
| `moveit2_jazzy` | MoveIt2 Panda demo stack | ROS 2 Jazzy binary packages |
| `slam_toolbox_jazzy` | slam_toolbox online mapping on TurtleBot4 | ROS 2 Jazzy + Gazebo Harmonic + `slam_arena` |

Each modern profile is a small target contract: it selects the launch adapter,
fuzzing input, readiness conditions, recorded topics, and executable oracle.
`slam_toolbox_jazzy` reuses the TurtleBot4 motion base and `/cmd_vel` input,
then adds the SLAM arena, trajectory-comparison oracle, collision quality gate,
and the official slam_toolbox lifecycle launch with automatic activation.

PX4 on Jazzy is treated as a compatibility spike. PX4 v1.17.0 is the current
stable release target for this fork, but PX4's ROS 2 documentation still
recommends ROS 2 Humble on Ubuntu 22.04 as the supported platform. This
repository intentionally keeps the unified Jazzy image goal while documenting
any PX4 integration failure evidence instead of silently switching to Humble.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `src/` | Existing improved RoboFuzz tree kept for comparison with the prior Foxy workflow. |
| `src_jazzy/` | Working tree for Jazzy target adaptation. New target code should land here. |
| `docker/jazzy/` | Docker build area for `robofuzz-jazzy:latest`. |
| `target_profiles/` | Profile files that describe each modern target's launch, input, watchlist, and oracle assumptions. |
| `targets.lock` | Human-readable lock manifest for image base, target versions, package names, and externally pinned source tags. |
| `docs/superpowers/` | Design and implementation notes for the modernization work. |

## Build

Build the Jazzy image from the repository root:

```bash
docker build -t robofuzz-jazzy:latest -f docker/jazzy/Dockerfile .
```

Quick image sanity check:

```bash
docker run --rm robofuzz-jazzy:latest bash -lc \
  'lsb_release -a && printenv ROS_DISTRO && ros2 --version && gz sim --versions'
```

## Running RoboFuzz Jazzy

All commands below assume this checkout is at `/home/ymc/RoboFuzz-jazzy`.
The host path is mounted into the container as `/work`, so the fuzzer uses the
current working tree rather than the copy baked into the image. Logs are written
under `/work/logs_jazzy/...`; each log directory keeps `metadata/`, `queue/`,
`errors/`, `rosbags/`, and a `latest` symlink.

If a named container already exists, remove it first or choose another
`--name`.

### MoveIt2 Panda with RViz

```bash
xhost +SI:localuser:root
docker run --rm -it --name robofuzz-jazzy-moveit-gui \
  --init \
  --network host --ipc host \
  -e DISPLAY="$DISPLAY" \
  -e MOVEIT_HEADLESS=0 \
  -e MOVEIT_WITH_RVIZ=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest \
  bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd /work/src_jazzy
python3 -u fuzzer.py \
  --target-profile moveit2_jazzy \
  --schedule single \
  --maxloop 100 \
  --no-cov \
  --logdir /work/logs_jazzy/moveit2_gui
```

After a GUI run, restore host X access control:

```bash
xhost -SI:localuser:root
```

### MoveIt2 Panda headless

```bash
docker run --rm -it --name robofuzz-jazzy-moveit-headless \
  --init \
  --network host --ipc host \
  -e MOVEIT_HEADLESS=1 \
  -e MOVEIT_WITH_RVIZ=0 \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest \
  bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd /work/src_jazzy
python3 -u fuzzer.py \
  --target-profile moveit2_jazzy \
  --schedule single \
  --maxloop 1000 \
  --no-cov \
  --logdir /work/logs_jazzy/moveit2_headless_1000
```

### TurtleBot4 with Gazebo GUI

```bash
xhost +SI:localuser:root
docker run --rm -it --name robofuzz-jazzy-tb4-gui \
  --init \
  --network host \
  --shm-size=2g \
  -e DISPLAY="$DISPLAY" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e TURTLEBOT4_HEADLESS=0 \
  -e TURTLEBOT4_USE_XVFB=0 \
  -e TURTLEBOT4_WORLD=simple \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest \
  bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd /work/src_jazzy
python3 -u fuzzer.py \
  --target-profile turtlebot4_jazzy \
  --schedule sequence \
  --seqlen 8 \
  --maxloop 30 \
  --interval 0.2 \
  --no-cov \
  --logdir /work/logs_jazzy/tb4_gui
```

After a GUI run:

```bash
xhost -SI:localuser:root
```

### TurtleBot4 headless

Use this form for longer fuzzing campaigns. `--init` is important for repeated
Gazebo/ROS process cleanup, and a private Docker shared-memory segment avoids
FastDDS shared-memory lock buildup across rounds.

```bash
docker run --rm -it --name robofuzz-jazzy-tb4-headless \
  --init \
  --network host \
  --shm-size=2g \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e TURTLEBOT4_HEADLESS=1 \
  -e TURTLEBOT4_USE_XVFB=1 \
  -e TURTLEBOT4_WORLD=empty \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest \
  bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd /work/src_jazzy
python3 -u fuzzer.py \
  --target-profile turtlebot4_jazzy \
  --schedule sequence \
  --seqlen 8 \
  --maxloop 1000 \
  --interval 0.2 \
  --no-cov \
  --logdir /work/logs_jazzy/tb4_headless_1000
```

### slam_toolbox mapping headless (clean target per round)

The SLAM profile defaults to the repository's `slam_arena` world. For the
normal RoboFuzz workflow, do **not** pass `--persistent` and do not start
`run_target.sh` separately. The fuzzer owns the target lifecycle: every round
starts a fresh Gazebo, TurtleBot4, and slam_toolbox stack, waits for readiness,
records and checks that round, then tears the stack down. This resets the
occupancy map, pose graph, `map -> odom` transform, robot pose, and SLAM node
state before the next mutated sequence.

Verified software versions for this target:

```text
OS:              Ubuntu 24.04 (Noble)
ROS:             ROS 2 Jazzy
Simulator:       Gazebo Harmonic (ros_gz)
World:           slam_arena (in-repo, worlds/slam_arena.sdf)
slam_toolbox:    ros-jazzy-slam-toolbox 2.8.5-1noble  (sync_slam_toolbox_node, mode: mapping)
TurtleBot4 base: ros-jazzy-turtlebot4-simulator, ros-jazzy-irobot-create-nodes
Laser:           simulated RPLidar (frame rplidar_link, 0.164-12.0 m, 640 samples)
Input:           /cmd_vel geometry_msgs/msg/TwistStamped
Oracle:          oracles.slam (estimate-vs-ground-truth RPE / ATE / yaw drift)
```

`seqlen 20` is a workable starting point, but a longer sequence (for example
`--seqlen 40`) keeps the robot moving long enough for slam_toolbox to publish
more time-aligned poses per round, which raises the fraction of rounds the
oracle can actually judge. `SLAM_MAX_LIN` / `SLAM_MAX_ANG` override the SLAM
velocity envelope if you want to push motion harder.

Run one clean end-to-end validation round first. This includes a complete
target launch and can take several minutes on a CPU-bound simulator:

```bash
docker run --rm -it --name robofuzz-jazzy-slam-clean-smoke \
  --init \
  --network host \
  --shm-size=2g \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e TURTLEBOT4_HEADLESS=1 \
  -e TURTLEBOT4_USE_XVFB=1 \
  -e TURTLEBOT4_WORLD=slam_arena \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest \
  bash -lc 'source /opt/ros/jazzy/setup.bash; cd /work/src_jazzy; python3 -u fuzzer.py --target-profile slam_toolbox_jazzy --schedule sequence --seqlen 20 --maxloop 1 --interval 1.0 --no-cov --logdir /work/logs_jazzy/slam_clean_smoke'
```

The same foreground log should show the target readiness markers, one
`[slam] status=...` summary, and final cleanup. A process name alone is not
sufficient: the expected SLAM lifecycle marker is
`[LifecycleLaunch] Slamtoolbox node is activating.`, and the profile also
waits for live odometry and simulator ground truth.

After the one-round result is valid, run a clean-state campaign in a new
container. Every round inside this command still restarts the complete target:

```bash
docker run --rm -it --name robofuzz-jazzy-slam-clean-campaign \
  --init \
  --network host \
  --shm-size=2g \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e TURTLEBOT4_HEADLESS=1 \
  -e TURTLEBOT4_USE_XVFB=1 \
  -e TURTLEBOT4_WORLD=slam_arena \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest \
  bash -lc 'source /opt/ros/jazzy/setup.bash; cd /work/src_jazzy; python3 -u fuzzer.py --target-profile slam_toolbox_jazzy --schedule sequence --seqlen 20 --maxloop 1000 --interval 1.0 --no-cov --logdir /work/logs_jazzy/slam_clean_1000'
```

These are starting values, not a guarantee that every round will be judged. If
many rounds report `status=insufficient`, increase sequence length and/or the
interval so slam_toolbox has more opportunities to accept scans and publish
time-aligned poses. Use the observed status distribution to tune the campaign.
Because clean-state mode relaunches the full simulator every round, a large
campaign can be very slow; run the long command yourself and return the log for
analysis.

### slam_toolbox mapping with Gazebo and RViz

Allow the container to use the host X server, then run one clean GUI round.
The fuzzer launches Gazebo and the TurtleBot4 navigation RViz view itself and
tears them down after the round:

```bash
xhost +SI:localuser:root
docker run --rm -it --name robofuzz-jazzy-slam-clean-gui \
  --init \
  --network host \
  --shm-size=2g \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e TURTLEBOT4_HEADLESS=0 \
  -e TURTLEBOT4_USE_XVFB=0 \
  -e TURTLEBOT4_WORLD=slam_arena \
  -e SLAM_RVIZ=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest \
  bash -lc 'source /opt/ros/jazzy/setup.bash; cd /work/src_jazzy; python3 -u fuzzer.py --target-profile slam_toolbox_jazzy --schedule sequence --seqlen 20 --maxloop 1 --interval 1.0 --no-cov --logdir /work/logs_jazzy/slam_clean_gui'
```

After removing the GUI container, restore host X access control:

```bash
xhost -SI:localuser:root
```

#### SLAM observations and limits

- `/pose` is slam_toolbox's semantic pose estimate
  (`geometry_msgs/msg/PoseWithCovarianceStamped`), not simulator truth. It
  publishes data only after motion crosses the configured distance or heading
  threshold and produces a successful scan match. `/sim_ground_truth_pose` is
  the independent Gazebo ground-truth odometry used by the oracle for
  trajectory comparison.
- `/pose` uses volatile durability, so a late subscriber does not receive an
  earlier sample. For manual diagnosis, start the `/pose` subscriber before
  commanding motion. This profile sets
  `check_min_dist_and_heading_precisely: true`, with distance and heading
  thresholds set to `0.1 m` and `0.1 rad`, so satisfying either threshold can
  admit a scan into matching. This prevents high-angular-velocity fuzzing from
  being rejected only because translation is below its threshold. It does not
  guarantee a fixed number of `/pose` samples in every fuzzing round.
- Collision rounds are rejected before feedback-history evaluation. They may
  still be recorded for diagnosis, but they cannot become interesting seeds or
  shift the SLAM feedback baseline.
- Every SLAM checker call prints one stable summary such as
  `[slam] status=judged valid=1 collision=0 matched_pairs=20 rpe_m=0.012
  ate_m=0.018 yaw_rad=0.009`. The status meanings are:
  - `collision`: contact was observed. This has priority over the other status
    labels, and the round is rejected before feedback-history evaluation.
  - `insufficient`: drift was not judged. This can mean missing trajectories,
    fewer than ten acceptable pairs, or another incomplete observation; it
    does not mean SLAM was correct or incorrect. Its matched-pair growth remains
    useful coverage feedback and may still guide mutation.
  - `judged`: RPE, ATE, and yaw calculations all executed and their feedback
    values were written. Only this status represents a completed drift
    judgment.
- `valid=1` requires at least ten estimate/truth pairs aligned by non-zero ROS
  `header.stamp` time within the oracle tolerance, with each ground-truth sample
  used by at most one estimate. `valid=0` means the round was recorded but did
  not complete drift judgment; it is not a total feedback-quality gate.
- Clean-state fuzzing is the default and omits `--persistent`. Persistent mode
  is reserved for a separate stateful campaign that intentionally studies
  accumulated map/pose-graph behavior across multiple mutation rounds; its
  results must not be mixed with independent clean-round results.
- SLAM commands default to a post-mutation clamp of `0.35 m/s` linear and
  `1.5 rad/s` angular. Set `SLAM_MAX_LIN` and `SLAM_MAX_ANG` on the container to
  choose positive finite values up to the TurtleBot4 physical limits of
  `0.46 m/s` and `1.9 rad/s`; invalid or larger values are rejected. These
  SLAM-specific limits do not change the conservative `turtlebot4_jazzy`
  limits.

In a fresh isolated integration smoke test, the lifecycle node reached
`active [3]`, `/scan` had one slam_toolbox subscriber, and translational motion
produced `/pose` data. The same run produced a `238 x 238` occupancy grid at
`0.05 m` resolution and a three-node pose graph. These observations validate
the current launch and data path; they are not calibrated accuracy or
performance guarantees.

### Target profile shortcuts

The canonical interface is `--target-profile <name>`. Compatibility aliases are
kept conservative:

| Alias | Policy |
| --- | --- |
| `--tb3-sitl` | Legacy/Foxy-only until a separate TurtleBot3 Jazzy profile exists. |
| `--tb4-sitl` | Alias for `--target-profile turtlebot4_jazzy`. |
| `--test-moveit` | Alias for `--target-profile moveit2_jazzy`. |
| PX4 legacy shortcuts | Kept legacy until the PX4 Jazzy adapter is fully validated. |

## Specification Preprocessing and OracleIR

The specification corpus under `system_doc/` is preprocessed into `SpecBlock`
JSON files before OracleIR generation. The generated OracleIR YAML files are
research artifacts under `src_jazzy/oracle_ir/specs/<target>/`; they guide
human/LLM oracle design, while the executable runtime checks live in
`src_jazzy/oracles/`.

### Preprocess the corpus

Run preprocessing from the repository root:

```bash
cd /home/ymc/RoboFuzz-jazzy
PYTHONPYCACHEPREFIX=/tmp/robofuzz_jazzy_pycache \
python3 -m system_doc.preprocessing.run_pipeline --target moveit2_panda

PYTHONPYCACHEPREFIX=/tmp/robofuzz_jazzy_pycache \
python3 -m system_doc.preprocessing.run_pipeline --target turtlebot4_jazzy

PYTHONPYCACHEPREFIX=/tmp/robofuzz_jazzy_pycache \
python3 -m system_doc.preprocessing.run_pipeline --target px4_v117_jazzy
```

Outputs are written to:

```text
system_doc/preprocessed/<target>/blocks/
system_doc/preprocessed/<target>/index.json
```

Use `-v` when debugging extraction or tag assignment:

```bash
PYTHONPYCACHEPREFIX=/tmp/robofuzz_jazzy_pycache \
python3 -m system_doc.preprocessing.run_pipeline --target px4_v117_jazzy -v
```

### Generate OracleIR with the agent

Generate OracleIR in small batches by target and tag:

```bash
cd /home/ymc/RoboFuzz-jazzy
ORACLE_AGENT_MAX_TURNS=10 \
PYTHONPYCACHEPREFIX=/tmp/robofuzz_jazzy_pycache \
python3 -m src_jazzy.oracle_ir.agent.generate \
  --target px4_v117_jazzy \
  --tag velocity_constraint \
  --mode agent
```

Common batch tags:

| Target | Useful tags |
| --- | --- |
| `moveit2_panda` | `oracle_joint_velocity_bound_candidate`, `oracle_joint_acceleration_bound_candidate`, `oracle_joint_jerk_bound_candidate` |
| `turtlebot4_jazzy` | `oracle_velocity_bound_candidate`, `oracle_acceleration_bound_candidate`, `cmd_odom_consistency`, `scan_range_constraint` |
| `px4_v117_jazzy` | `velocity_constraint`, `acceleration_constraint`, `attitude_constraint`, `offboard_constraint`, `failsafe_constraint`, `actuator_constraint`, `sensor_validity` |

Generate a single block when reviewing or repairing one rule:

```bash
ORACLE_AGENT_MAX_TURNS=10 \
PYTHONPYCACHEPREFIX=/tmp/robofuzz_jazzy_pycache \
python3 -m src_jazzy.oracle_ir.agent.generate \
  --target px4_v117_jazzy \
  --block-id px4_v117_jazzy.parameter.MPC_XY_VEL_MAX \
  --mode agent
```

Preview a prompt without calling the API:

```bash
PYTHONPYCACHEPREFIX=/tmp/robofuzz_jazzy_pycache \
python3 -m src_jazzy.oracle_ir.agent.generate \
  --target px4_v117_jazzy \
  --tag velocity_constraint \
  --mode agent \
  --dry-run
```

Optional OracleIR quality check:

```bash
PYTHONPYCACHEPREFIX=/tmp/robofuzz_jazzy_pycache \
python3 -m src_jazzy.oracle_ir.evaluation.run_eval \
  --target px4_v117_jazzy \
  --spec-dir src_jazzy/oracle_ir/specs/px4_v117_jazzy
```

## Upstream Attribution

RoboFuzz was originally published as:

> RoboFuzz: Fuzzing Robotic Systems over Robot Operating System (ROS) for
> Finding Correctness Bugs, ESEC/FSE 2022.

Original project and paper links:

- <https://github.com/sslab-gatech/RoboFuzz>
- <https://seulbae-security.github.io/pubs/robofuzz-fse22.pdf>

This fork keeps the original MIT license and attribution while adding the
Jazzy/Noble/Harmonic modernization work and the user's improved target/oracle
logic.
