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

    def sample(self) -> float:
        """Generate a random value within the domain range."""
        return random.uniform(self.low, self.high)

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
                "vz": FieldRange("vz", -1.0, 5.0),
                "yaw": FieldRange("yaw", -math.pi, math.pi),
                "yawspeed": FieldRange("yawspeed", -3.49, 3.49),
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
