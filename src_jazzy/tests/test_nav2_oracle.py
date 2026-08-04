import math, os, sys, unittest
from types import SimpleNamespace as NS

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from oracles import nav2  # noqa: E402


def _quat(yaw):
    return NS(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

def make_pose_stamped(x=0.0, y=0.0, yaw=0.0, sec=1):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=0), frame_id="map"),
              pose=NS(position=NS(x=x, y=y, z=0.0), orientation=_quat(yaw)))

def make_truth(x=0.0, y=0.0, yaw=0.0, sec=1):
    m = NS(header=NS(stamp=NS(sec=sec, nanosec=0), frame_id="odom"),
           pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0),
                           orientation=_quat(yaw))))
    m.child_frame_id = "base_link"
    return m

def make_int32(v):
    return NS(data=int(v))

def _fbk(names):
    return [NS(name=n, update_value=(lambda self=NS(): None)) for n in names]

class _FB:
    def __init__(self, name): self.name = name; self.value = 0.0
    def update_value(self, v): self.value = v

def fb_list():
    return [_FB(n) for n in nav2.NAV2_FEEDBACK_NAMES]

def truth_track(points):
    # points: list of (x,y,yaw,sec)
    return [(float(s), make_truth(x, y, yaw, s)) for (x, y, yaw, s) in points]

class TestArrival(unittest.TestCase):
    def test_arrival_ok_no_error(self):
        goal = [(1.0, make_pose_stamped(2.0, 0.0, 0.0))]
        truth = [(float(s), make_truth(2.0 + 0.01 * (s % 2), 0.0, 0.0, s))
                 for s in range(1, 21)]
        state = {nav2.GOAL_TOPIC: goal, nav2.RESULT_TOPIC:
                 [(20.0, make_int32(nav2.NAV2_STATUS_SUCCEEDED))],
                 nav2.GROUND_TRUTH_TOPIC: truth}
        errs = nav2.check(NS(), [], state, fb_list())
        self.assertEqual([e for e in errs if "ARRIVAL" in e], [])

    def test_arrival_lie_reports_error(self):
        goal = [(1.0, make_pose_stamped(5.0, 0.0, 0.0))]
        truth = [(float(s), make_truth(1.0, 0.0, 0.0, s))
                 for s in range(1, 21)]  # truth ends 4m from goal
        state = {nav2.GOAL_TOPIC: goal, nav2.RESULT_TOPIC:
                 [(20.0, make_int32(nav2.NAV2_STATUS_SUCCEEDED))],
                 nav2.GROUND_TRUTH_TOPIC: truth}
        errs = nav2.check(NS(), [], state, fb_list())
        self.assertTrue(any("NAV2_ARRIVAL_ERROR" in e for e in errs))

    def test_missing_topics_no_error(self):
        errs = nav2.check(NS(), [], {}, fb_list())
        self.assertEqual(errs, [])
