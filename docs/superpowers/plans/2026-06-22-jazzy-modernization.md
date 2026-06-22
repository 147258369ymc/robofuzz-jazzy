# RoboFuzz Jazzy Modernization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a maintainable Jazzy/Noble/Harmonic modernization branch with target profiles, Docker scaffolding, and a future path for TurtleBot4, PX4 v1.17.0, and MoveIt2 Jazzy.

**Architecture:** Keep `src/` stable as the comparison baseline and adapt targets in `src_jazzy/`. Add profile files first, then Docker image support, then profile-backed source adapters.

**Tech Stack:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Docker, Python 3.12, colcon, rosdep, PX4 v1.17.0, TurtleBot4 simulator, MoveIt2.

---

### Task 1: Repository Boundary and Documentation

**Files:**
- Modify: `README.md`
- Modify: `.gitignore`
- Create: `docs/superpowers/specs/2026-06-22-jazzy-modernization-design.md`
- Create: `docs/superpowers/plans/2026-06-22-jazzy-modernization.md`

- [ ] **Step 1: Replace the public README with Jazzy fork context**

Write a README that says this repository is a RoboFuzz modernization fork, lists the three active profiles, explains the `src/` versus `src_jazzy/` boundary, and keeps upstream attribution.

- [ ] **Step 2: Allow tracked modernization docs**

Change `.gitignore` from ignoring all `docs/` content to ignoring `docs/*` while unignoring `docs/superpowers/**`. Also ignore `src_jazzy/logs/`, `src_jazzy/out`, and `src_jazzy/err`.

- [ ] **Step 3: Verify repository state**

Run:

```bash
git status --short
```

Expected: only README, `.gitignore`, and new `docs/superpowers/` files are changed.

- [ ] **Step 4: Commit**

```bash
git add README.md .gitignore docs/superpowers
git commit -m "docs: describe Jazzy modernization fork"
```

### Task 2: Target Profile and Lock Scaffold

**Files:**
- Create: `target_profiles/turtlebot4_jazzy.yaml`
- Create: `target_profiles/px4_v117_jazzy.yaml`
- Create: `target_profiles/moveit2_jazzy.yaml`
- Create: `targets.lock`
- Create: `run_target.sh`

- [ ] **Step 1: Add TurtleBot4 profile**

Create a profile with `model: standard`, a `model` override parameter, `ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py`, preferred `TwistStamped` command input, and initial watch topics for odometry, scan, hazard, slip, stall, kidnap, and Create3 status topics.

- [ ] **Step 2: Add PX4 profile**

Create a profile pinned to `PX4-Autopilot` tag `v1.17.0` and commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`. Use `HEADLESS=1 make px4_sitl gz_x500`, Micro XRCE-DDS Agent on UDP port 8888, `/fmu/in/*` input topics, and `/fmu/out/*` watch topics.

- [ ] **Step 3: Add MoveIt2 profile**

Create a profile for `ros-jazzy-moveit` and Panda resources, preserving the watch topics `/motion_plan_request`, `/joint_states`, `/panda_arm_controller/state`, and `/move_action/_action/status`.

- [ ] **Step 4: Add `targets.lock`**

Record the image base, ROS/Gazebo baseline, apt package names, and pinned PX4 tag/SHA. Mark Debian package versions as resolved by the first successful image build, with the exact command that will generate the installed package lock.

- [ ] **Step 5: Add `run_target.sh`**

Create a conservative shell wrapper that accepts `turtlebot4_jazzy`, `px4_v117_jazzy`, or `moveit2_jazzy`, sources `/opt/ros/jazzy/setup.bash`, and prints the launch command it will run. Keep the script as a smoke helper, not a replacement for the later Python launch adapter.

- [ ] **Step 6: Verify shell syntax**

Run:

```bash
bash -n run_target.sh
```

Expected: exit code 0.

- [ ] **Step 7: Commit**

```bash
git add target_profiles targets.lock run_target.sh
git commit -m "chore: add Jazzy target scaffold"
```

### Task 3: Docker Jazzy Baseline

**Files:**
- Create: `docker/jazzy/Dockerfile`
- Create: `docker/jazzy/README.md`

- [ ] **Step 1: Add Dockerfile**

Create a Dockerfile based on `ros:jazzy-ros-base-noble` that installs build tools, Python tooling, `ros-jazzy-ros-gz`, `ros-jazzy-moveit`, `ros-jazzy-turtlebot4-simulator`, `ros-jazzy-irobot-create-nodes`, and PX4 build prerequisites.

- [ ] **Step 2: Add Docker README**

Document build, environment verification, and the first smoke-test commands.

- [ ] **Step 3: Build image**

Run:

```bash
docker build -t robofuzz-jazzy:latest -f docker/jazzy/Dockerfile .
```

Expected: image build succeeds, or the failure is captured with the failing package name and command.

- [ ] **Step 4: Verify image baseline**

Run:

```bash
docker run --rm robofuzz-jazzy:latest bash -lc 'lsb_release -a && printenv ROS_DISTRO && ros2 --version && gz sim --versions'
```

Expected: Ubuntu 24.04, ROS_DISTRO `jazzy`, and Gazebo Harmonic tools are available.

### Task 4: Source Profile Adapter Spike

**Files:**
- Create: `src_jazzy/target_profiles.py`
- Modify: `src_jazzy/fuzzer.py`
- Modify: `src_jazzy/harness.py`
- Test: `src_jazzy/tests/test_target_profiles.py`

- [ ] **Step 1: Write profile loader tests**

Add tests that load all three YAML profiles and assert each profile has `name`, `input`, `launch`, `watch`, and `oracle` sections.

- [ ] **Step 2: Implement profile loader**

Implement a small YAML loader that reads `../target_profiles/<name>.yaml` and returns a dictionary.

- [ ] **Step 3: Add CLI flag**

Add `--target-profile` to `src_jazzy/fuzzer.py` without changing legacy aliases yet.

- [ ] **Step 4: Run tests**

Run:

```bash
python3 -m pytest -q src_jazzy/tests/test_target_profiles.py
```

Expected: profile loader tests pass after `pytest` is installed in the Jazzy image or host environment.
