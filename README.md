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

## Planned Public Interfaces

These interfaces are the intended direction for the Jazzy port:

```bash
./run_target.sh turtlebot4_jazzy
./run_target.sh px4_v117_jazzy
./run_target.sh moveit2_jazzy

cd src_jazzy
./fuzzer.py --target-profile turtlebot4_jazzy ...
./fuzzer.py --target-profile px4_v117_jazzy ...
./fuzzer.py --target-profile moveit2_jazzy ...
```

Compatibility aliases will be handled conservatively:

| Alias | Policy |
| --- | --- |
| `--tb3-sitl` | Keep as legacy/Foxy-only until a separate TurtleBot3 Jazzy profile exists. |
| `--tb4-sitl` | New alias for `--target-profile turtlebot4_jazzy`. |
| `--test-moveit` | Keep as an alias for `moveit2_jazzy`. |
| PX4 legacy shortcuts | Map only after the PX4 uXRCE-DDS adapter is working. |

## First Build Goal

The first image target is:

```bash
docker build -t robofuzz-jazzy:latest -f docker/jazzy/Dockerfile .
```

The first image should verify:

```bash
lsb_release -a
printenv ROS_DISTRO
ros2 --version
gz sim --versions
ros2 pkg list
```

Target smoke tests will be added in this order:

1. TurtleBot4 launch and `/cmd_vel` command smoke test.
2. MoveIt2 Panda launch and one reachable pose goal.
3. PX4 v1.17.0 SITL + Micro XRCE-DDS/uXRCE-DDS ROS 2 bridge smoke test.

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
