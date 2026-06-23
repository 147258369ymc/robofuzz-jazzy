# Jazzy Docker Image

This directory builds the first `robofuzz-jazzy:latest` image.

## Build

```bash
docker build -t robofuzz-jazzy:latest -f docker/jazzy/Dockerfile .
```

The Dockerfile starts from `ros:jazzy-ros-base-noble`, installs ROS/Gazebo
tooling, TurtleBot4 simulator packages, MoveIt2 Jazzy packages, RoboFuzz Python
dependencies, a pinned PX4-Autopilot v1.17.0 checkout, `px4_msgs` from
`release/1.17`, and Micro XRCE-DDS Agent `v2.4.3`. The Agent is built against
Jazzy's system FastDDS/Fast-CDR with its logger and P2P profiles disabled, which
keeps the build compatible with Ubuntu 24.04 while preserving the UDP agent path
needed for the first PX4 SITL smoke tests.

PX4 remains the riskiest target on a unified Jazzy image. The image now includes
the ROS 2 message package and the v2.x Micro XRCE-DDS Agent line used by PX4
v1.17, but the PX4 SITL build/smoke pass still needs to be treated as a
compatibility spike rather than assumed to be production-ready.

## Baseline Verification

```bash
docker run --rm robofuzz-jazzy:latest bash -lc \
  'lsb_release -a && printenv ROS_DISTRO && ros2 pkg list >/dev/null && gz sim --versions'
```

```bash
docker run --rm robofuzz-jazzy:latest bash -lc \
  'source /robofuzz/px4_ros2_ws/install/setup.bash && python3 -c "from px4_msgs.msg import VehicleStatus; print(\"px4_msgs ok\")" && command -v MicroXRCEAgent'
```

## Target Smoke Commands

```bash
docker run --rm -it robofuzz-jazzy:latest ./run_target.sh turtlebot4_jazzy
docker run --rm -it robofuzz-jazzy:latest ./run_target.sh moveit2_jazzy
docker run --rm -it robofuzz-jazzy:latest ./run_target.sh px4_v117_jazzy
```

The TurtleBot4 launcher defaults to a split headless mode: it starts
`ros_gz_sim` under `xvfb-run` with `gz_args:="-s -r warehouse.sdf"` and then
starts the `/clock` bridge before launching `turtlebot4_spawn.launch.py`. Set
`TURTLEBOT4_HEADLESS=0` to use the upstream `turtlebot4_gz.launch.py` path, or
`TURTLEBOT4_USE_XVFB=0` to test without a virtual display. The MoveIt launcher
defaults to the binary Panda
resources package, `moveit_resources_panda_moveit_config`, because
`moveit2_tutorials` is not available as a Jazzy apt package in the verified
image.

## MoveIt2 with visible RViz (host X11)

To watch the Panda arm move in RViz while fuzzing, share the host X server with
the container. On the host:

```bash
xhost +local:
docker run --rm -it --name robofuzz-jazzy-moveit-x11 \
  --network host --ipc host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest \
  bash
```

Inside the container, source ROS and run the MoveIt target normally. For
headless runs without an X server, set `MOVEIT_HEADLESS=1` to force the
offscreen Qt backend.

Security note: `xhost +local:` relaxes X access control for local clients on the
host. Restore the stricter default after the run with:

```bash
xhost -local:
```
