"""
Mutation profile configuration for domain-constrained, feedback-adaptive fuzzing.

Provides reusable profiles that define per-field value ranges, mutation strategy
weights, and feedback-to-strategy mappings. Designed for extensibility across
different targets and control modes.
"""

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Tuple


@dataclass
class FieldRange:
    """Domain constraint for a single message field."""
    name: str
    low: float
    high: float
    extreme_prob: float = 0.15  # P(sample an out-of-range extreme value)

    def sample(self) -> float:
        """Generate a random value within the domain range, with
        `extreme_prob` chance of producing an out-of-range extreme value
        that can stress the controller into undefined behavior."""
        if random.random() < self.extreme_prob:
            return self._sample_extreme()
        return random.uniform(self.low, self.high)

    def _sample_extreme(self) -> float:
        """Generate values outside normal range to trigger controller edge cases.
        Includes very large values, near-zero denorms, and boundary values."""
        roll = random.random()
        span = self.high - self.low
        if roll < 0.3:
            # 2x-5x beyond range
            multiplier = random.uniform(2.0, 5.0)
            return random.choice([self.high * multiplier, self.low * multiplier])
        elif roll < 0.6:
            # Exact boundary values (often trigger off-by-one in controllers)
            return random.choice([self.low, self.high, 0.0])
        else:
            # Very large values that may overflow controller integrators
            return random.choice([1e6, -1e6, 1e3, -1e3])

    def sample_high_magnitude(self) -> float:
        """Sample from the upper/lower 30% of the range (extreme values)."""
        span = self.high - self.low
        if random.random() < 0.5:
            return random.uniform(self.high - 0.3 * span, self.high)
        else:
            return random.uniform(self.low, self.low + 0.3 * span)

    def clamp(self, value: float) -> float:
        return max(self.low, min(self.high, value))


STRATEGY_MULTI_AXIS = "multi_axis"
STRATEGY_FLIP = "flip"
STRATEGY_SINGLE_BLOCK = "single_block"
STRATEGY_RANDOM = "random"

# MoveIt-specific strategies
STRATEGY_BOUNDARY_PUSH = "boundary_push"
STRATEGY_REVERSAL = "reversal"
STRATEGY_TRAJECTORY_ARC = "trajectory_arc"
STRATEGY_SINGLE_EXTREME = "single_extreme"

# Default feedback-to-strategy mappings for PX4
_PX4_FEEDBACK_STRATEGY_MAP = {
    "max_jerk": STRATEGY_FLIP,
    "max_angular_rate": STRATEGY_FLIP,
    "combined_tilt_velocity": STRATEGY_MULTI_AXIS,
    "max_tilt_angle": STRATEGY_MULTI_AXIS,
    "max_xy_velocity": STRATEGY_SINGLE_BLOCK,
    "actuator_saturation": STRATEGY_SINGLE_BLOCK,
    "vel_pos_inconsistency": STRATEGY_FLIP,
    "max_altitude": STRATEGY_SINGLE_BLOCK,
}

# MoveIt feedback-to-strategy mappings
_MOVEIT_FEEDBACK_STRATEGY_MAP = {
    "workspace_boundary_distance": STRATEGY_BOUNDARY_PUSH,
    "trajectory_tracking_rms": STRATEGY_REVERSAL,
    "max_joint_jerk": STRATEGY_REVERSAL,
    "planning_duration": STRATEGY_RANDOM,
    "abort_joint_drift": STRATEGY_SINGLE_EXTREME,
    "velocity_roughness": STRATEGY_TRAJECTORY_ARC,
    "joint_motion_range": STRATEGY_BOUNDARY_PUSH,
    "end_point_deviation": STRATEGY_TRAJECTORY_ARC,
    "max_velocity_margin": STRATEGY_BOUNDARY_PUSH,
    "execution_sample_count": STRATEGY_TRAJECTORY_ARC,
    "max_joint_pos_error": STRATEGY_REVERSAL,
    "max_joint_vel_error": STRATEGY_REVERSAL,
    "mean_joint_pos_error": STRATEGY_SINGLE_EXTREME,
    "mean_joint_vel_error": STRATEGY_SINGLE_EXTREME,
}


@dataclass
class MutationProfile:
    """
    Encapsulates mutation configuration for a specific target/control mode.

    Attributes:
        field_ranges: Per-field domain constraints keyed by field name.
        strategy_weights: Base probability weights for each strategy.
        block_len_range: (min, max) block length for sustained mutations.
        feedback_strategy_map: Maps feedback metric names to the strategy
            that should be boosted when that metric triggers interesting.
        adaptation_boost: How much to boost a strategy's weight on feedback hit.
    """
    field_ranges: Dict[str, FieldRange]
    strategy_weights: Dict[str, float]
    block_len_range: Tuple[int, int]
    feedback_strategy_map: Dict[str, str]
    adaptation_boost: float = 0.15

    def select_strategy(self, recent_feedback: str = None) -> str:
        """
        Select a mutation strategy based on weights, optionally adapted
        by the most recently triggered feedback metric.
        """
        weights = dict(self.strategy_weights)

        if recent_feedback and recent_feedback in self.feedback_strategy_map:
            boosted = self.feedback_strategy_map[recent_feedback]
            weights[boosted] += self.adaptation_boost
            # Normalize
            total = sum(weights.values())
            weights = {k: v / total for k, v in weights.items()}

        roll = random.random()
        cumulative = 0.0
        for strategy, weight in weights.items():
            cumulative += weight
            if roll < cumulative:
                return strategy
        return STRATEGY_SINGLE_BLOCK  # fallback

    def get_range(self, field_name: str) -> FieldRange:
        """Get the domain range for a field, or a default [-1, 1] range."""
        return self.field_ranges.get(
            field_name,
            FieldRange(field_name, -1.0, 1.0)
        )

    @classmethod
    def px4_ros_velocity(cls) -> "MutationProfile":
        """Profile for PX4 OFFBOARD velocity control mode (ROS)."""
        return cls(
            field_ranges={
                "vx": FieldRange("vx", -12.0, 12.0),
                "vy": FieldRange("vy", -12.0, 12.0),
                "vz": FieldRange("vz", -5.0, 1.0),
                "yaw": FieldRange("yaw", -math.pi, math.pi),
                "yawspeed": FieldRange("yawspeed", -3.49, 3.49),
            },
            strategy_weights={
                STRATEGY_MULTI_AXIS: 0.35,
                STRATEGY_FLIP: 0.20,
                STRATEGY_SINGLE_BLOCK: 0.30,
                STRATEGY_RANDOM: 0.15,
            },
            block_len_range=(5, 30),
            feedback_strategy_map=_PX4_FEEDBACK_STRATEGY_MAP,
        )

    @classmethod
    def px4_mavlink_posctl(cls) -> "MutationProfile":
        """Profile for PX4 POSCTL mode (MAVLink manual control)."""
        return cls(
            field_ranges={
                "x": FieldRange("x", -1.0, 1.0),
                "y": FieldRange("y", -1.0, 1.0),
                "z": FieldRange("z", 0.0, 1.0),
                "r": FieldRange("r", -1.0, 1.0),
            },
            strategy_weights={
                STRATEGY_MULTI_AXIS: 0.30,
                STRATEGY_FLIP: 0.20,
                STRATEGY_SINGLE_BLOCK: 0.50,
            },
            block_len_range=(5, 30),
            feedback_strategy_map=_PX4_FEEDBACK_STRATEGY_MAP,
        )

    @classmethod
    def moveit_panda(cls) -> "MutationProfile":
        """Profile for MoveIt2 Panda arm goal-position fuzzing.

        Workspace: sphere of ~0.855m radius centered at base.
        Ranges are tightened to the reliably-reachable region (most samples
        land within ~0.8m of the base) so planning budget is spent on goals
        MoveIt can actually execute, rather than on unreachable goals that get
        silently skipped. A small extreme_prob still probes the boundary/beyond.
        """
        return cls(
            field_ranges={
                "x": FieldRange("x", -0.6, 0.6, extreme_prob=0.05),
                "y": FieldRange("y", -0.6, 0.6, extreme_prob=0.05),
                "z": FieldRange("z", 0.1, 0.9, extreme_prob=0.05),
            },
            strategy_weights={
                STRATEGY_BOUNDARY_PUSH: 0.30,
                STRATEGY_REVERSAL: 0.15,
                STRATEGY_TRAJECTORY_ARC: 0.25,
                STRATEGY_SINGLE_EXTREME: 0.15,
                STRATEGY_RANDOM: 0.15,
            },
            block_len_range=(3, 8),
            feedback_strategy_map=_MOVEIT_FEEDBACK_STRATEGY_MAP,
        )
