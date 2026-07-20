import hashlib
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
OFFICIAL_PARAMS = (
    REPO_ROOT
    / "system_doc"
    / "turtlebot4_clean"
    / "params"
    / "turtlebot4_navigation.slam.yaml"
)
PROFILE_PARAMS = (
    REPO_ROOT / "target_profiles" / "slam_toolbox_jazzy.params.yaml"
)
RUN_TARGET = REPO_ROOT / "run_target.sh"
OFFICIAL_PARAMS_SHA256 = (
    "c0e42df72f08af87272e5f71d67f1b664393810534298d2f50dc4d51698863d1"
)


def _parameters(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)["slam_toolbox"]["ros__parameters"]


def _run_target_function_source(function_name, end_marker):
    text = RUN_TARGET.read_text(encoding="utf-8")
    start = text.index(f"{function_name}() {{")
    end = text.index(end_marker, start)
    return text[start:end]


def _resolve_params(env=None):
    function_source = _run_target_function_source(
        "slam_params_file", "start_slam_toolbox_when_ready() {"
    )
    script = "\n".join(
        (
            "set -euo pipefail",
            f"ROOT_DIR={shlex.quote(str(REPO_ROOT))}",
            function_source,
            "slam_params_file",
        )
    )
    completed = subprocess.run(
        ["bash", "-c", script],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    return completed.stdout.strip()


class SlamProfileParamsTests(unittest.TestCase):
    def test_profile_params_preserve_mapping_frames_and_enable_precise_gate(self):
        self.assertTrue(PROFILE_PARAMS.is_file())
        params = _parameters(PROFILE_PARAMS)

        self.assertEqual("base_link", params["base_frame"])
        self.assertEqual("/scan", params["scan_topic"])
        self.assertEqual(0.1, params["minimum_travel_distance"])
        self.assertEqual(0.1, params["minimum_travel_heading"])
        self.assertIs(True, params["check_min_dist_and_heading_precisely"])

    def test_profile_params_are_official_snapshot_plus_only_precise_gate(self):
        self.assertTrue(PROFILE_PARAMS.is_file())
        official = _parameters(OFFICIAL_PARAMS)
        profile = _parameters(PROFILE_PARAMS)

        self.assertNotIn("check_min_dist_and_heading_precisely", official)
        self.assertIs(True, profile.pop("check_min_dist_and_heading_precisely"))
        self.assertEqual(official, profile)

    def test_official_snapshot_remains_byte_for_byte_unchanged(self):
        digest = hashlib.sha256(OFFICIAL_PARAMS.read_bytes()).hexdigest()

        self.assertEqual(OFFICIAL_PARAMS_SHA256, digest)

    def test_run_target_defaults_to_slam_profile_params(self):
        env = os.environ.copy()
        env.pop("SLAM_PARAMS_FILE", None)

        self.assertEqual(str(PROFILE_PARAMS), _resolve_params(env))

    def test_slam_params_file_override_has_highest_priority(self):
        env = os.environ.copy()
        with tempfile.NamedTemporaryFile() as override:
            env["SLAM_PARAMS_FILE"] = override.name

            self.assertEqual(override.name, _resolve_params(env))

    def test_ordinary_tb4_launch_does_not_resolve_slam_params(self):
        function_source = _run_target_function_source(
            "run_turtlebot4", "run_px4() {"
        )

        self.assertNotIn("slam_params_file", function_source)
        self.assertNotIn("slam_toolbox_jazzy.params.yaml", function_source)


if __name__ == "__main__":
    unittest.main()
