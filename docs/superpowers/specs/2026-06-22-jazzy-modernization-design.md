# RoboFuzz Jazzy Modernization Design

## Goal

Modernize the improved RoboFuzz tree for a unified Ubuntu 24.04 Noble, ROS 2
Jazzy, and Gazebo Harmonic image while preserving the existing Foxy-oriented
code for comparison.

## Source Boundaries

The repository keeps two source trees:

| Path | Role |
| --- | --- |
| `src/` | Existing improved RoboFuzz code, kept as the comparison baseline. |
| `src_jazzy/` | Active development area for TurtleBot4, PX4 v1.17.0, and MoveIt2 Jazzy adaptation. |

The first modernization phase does not change fuzzing behavior. It establishes
documentation, target profiles, lock metadata, and image scaffolding so later
changes can be reviewed target by target.

## Environment

The common image target is:

| Layer | Choice |
| --- | --- |
| Base image | `ros:jazzy-ros-base-noble` |
| OS | Ubuntu 24.04 Noble |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Image tag | `robofuzz-jazzy:latest` |

This combination is the shared baseline because ROS REP-2000 lists Jazzy with
Ubuntu Noble and Gazebo Harmonic, and the Gazebo ROS installation guide pairs
ROS 2 Jazzy with Gazebo Harmonic. TurtleBot4 and MoveIt2 both document Jazzy
installation paths. PX4 v1.17.0 is pinned as the PX4 target, but PX4's ROS 2
guide still recommends Humble on Ubuntu 22.04, so PX4/Jazzy remains the risky
compatibility spike.

## Target Profiles

Profiles are data files under `target_profiles/`. They separate launch,
message, topic, and oracle assumptions from the core fuzzing loop.

| Profile | Default launch | Input |
| --- | --- | --- |
| `turtlebot4_jazzy` | `ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=standard` | Prefer `geometry_msgs/msg/TwistStamped`, fall back to `Twist` if required by the graph. |
| `px4_v117_jazzy` | `HEADLESS=1 make px4_sitl gz_x500` plus Micro XRCE-DDS Agent | Current `/fmu/in/*` ROS 2 topics from `px4_msgs`. |
| `moveit2_jazzy` | Jazzy MoveIt2 Panda demo resources | `moveit_msgs/msg/MotionPlanRequest`. |

## Adapter Direction

The later source changes should add profile-backed launch adapters instead of
hardcoding target behavior in `harness.py` and `fuzzer.py`.

Initial adapter responsibilities:

| Unit | Responsibility |
| --- | --- |
| Profile loader | Read YAML profile data and expose normalized target settings. |
| Launch adapter | Start and stop the target process group for a profile. |
| Input adapter | Select message type, topic, and initial seed behavior for a profile. |
| Watch adapter | Load profile watch topics and feed `state_monitor.py`. |
| Oracle adapter | Route a profile to the correct oracle and profile-specific thresholds. |

The first code migration must preserve the user's current MoveIt improvements:
error-signature deduplication, process-group cleanup, endpoint checking via the
last `MotionPlanRequest`, controller-state FK, velocity/acceleration tolerance
ratios, reachable-region mutation, and orientation mutation.

## Acceptance Gates

Each target is accepted only after one RoboFuzz execution creates `metadata`,
`queue`, `errors`, and `rosbags` output without manual edits after container
start.

Oracle thresholds must be justified by current target documentation, runtime
parameters, or measured baseline behavior. Old TurtleBot3 or PX4 v1.12 numeric
thresholds must not be copied into the Jazzy profiles without evidence.
