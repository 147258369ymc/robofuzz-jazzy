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

### Target profile shortcuts

The canonical interface is `--target-profile <name>`. Compatibility aliases are
kept conservative:

| Alias | Policy |
| --- | --- |
| `--tb3-sitl` | Legacy/Foxy-only until a separate TurtleBot3 Jazzy profile exists. |
| `--tb4-sitl` | Alias for `--target-profile turtlebot4_jazzy`. |
| `--test-moveit` | Alias for `--target-profile moveit2_jazzy`. |
| PX4 legacy shortcuts | Kept legacy until the PX4 Jazzy adapter is fully validated. |

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
