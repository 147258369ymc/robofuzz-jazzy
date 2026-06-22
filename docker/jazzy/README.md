# Jazzy Docker Image

This directory builds the first `robofuzz-jazzy:latest` image.

## Build

```bash
docker build -t robofuzz-jazzy:latest -f docker/jazzy/Dockerfile .
```

The Dockerfile starts from `ros:jazzy-ros-base-noble`, installs ROS/Gazebo
tooling, TurtleBot4 simulator packages, MoveIt2 Jazzy packages, RoboFuzz Python
dependencies, and a pinned PX4-Autopilot v1.17.0 checkout.

PX4 is intentionally only pinned and checked out in this first scaffold. The
first successful build should be followed by a PX4 SITL build/smoke pass so the
exact missing dependency list can be captured instead of guessed.

## Baseline Verification

```bash
docker run --rm robofuzz-jazzy:latest bash -lc \
  'lsb_release -a && printenv ROS_DISTRO && ros2 --version && gz sim --versions'
```

## Target Smoke Commands

```bash
docker run --rm -it robofuzz-jazzy:latest ./run_target.sh turtlebot4_jazzy
docker run --rm -it robofuzz-jazzy:latest ./run_target.sh moveit2_jazzy
docker run --rm -it robofuzz-jazzy:latest ./run_target.sh px4_v117_jazzy
```

The PX4 command expects `MicroXRCEAgent` to be available before the PX4 profile
is considered usable. If the first image build lacks it, add the source build or
distribution package in the next Docker iteration and record the evidence in
the commit message.
