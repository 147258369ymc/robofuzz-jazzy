import math
import os
import unittest
import xml.etree.ElementTree as ET

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
WORLD = os.path.join(REPO, "worlds", "slam_arena.sdf")


class TestSlamArenaWorld(unittest.TestCase):

    EXPECTED_WALLS = {
        "wall_north": ((0.0, 6.0, 0.25, 0.0, 0.0, 0.0),
                       (12.2, 0.1, 0.5)),
        "wall_south": ((0.0, -6.0, 0.25, 0.0, 0.0, 0.0),
                       (12.2, 0.1, 0.5)),
        "wall_east": ((6.0, 0.0, 0.25, 0.0, 0.0, 0.0),
                      (0.1, 12.2, 0.5)),
        "wall_west": ((-6.0, 0.0, 0.25, 0.0, 0.0, 0.0),
                      (0.1, 12.2, 0.5)),
    }
    EXPECTED_PILLARS = {
        "pillar_a": (4.5, 0.0, 0.3, 0.0, 0.0, 0.0),
        "pillar_b": (-2.25, 3.9, 0.3, 0.0, 0.0, 0.0),
        "pillar_c": (-2.25, -3.9, 0.3, 0.0, 0.0, 0.0),
    }

    def setUp(self):
        self.assertTrue(os.path.isfile(WORLD), f"missing {WORLD}")
        self.root = ET.parse(WORLD).getroot()
        self.world = self.root.find("world")
        self.assertIsNotNone(self.world)
        self.models = {
            model.get("name"): model for model in self.world.findall("model")
        }

    @staticmethod
    def _floats(text):
        return tuple(float(value) for value in text.split())

    def test_world_name(self):
        self.assertEqual("slam_arena", self.world.get("name"))

    def test_contact_plugin_present(self):
        plugins = {
            plugin.get("filename") for plugin in self.world.findall("plugin")
        }
        self.assertIn("gz-sim-contact-system", plugins)

    def test_four_walls_have_exact_geometry_and_rendering(self):
        walls = {
            name: model
            for name, model in self.models.items()
            if name.startswith("wall_")
        }
        self.assertEqual(set(self.EXPECTED_WALLS), set(walls))

        for name, (expected_pose, expected_size) in self.EXPECTED_WALLS.items():
            with self.subTest(wall=name):
                model = walls[name]
                self.assertEqual("true", model.findtext("static"))
                self.assertEqual(expected_pose, self._floats(model.findtext("pose")))
                link = model.find("link")
                self.assertIsNotNone(link.find("collision"))
                self.assertIsNotNone(link.find("visual"))
                self.assertEqual(
                    expected_size,
                    self._floats(link.findtext("collision/geometry/box/size")),
                )
                self.assertEqual(
                    expected_size,
                    self._floats(link.findtext("visual/geometry/box/size")),
                )

    def test_three_pillars_have_exact_geometry_and_rendering(self):
        pillars = {
            name: model
            for name, model in self.models.items()
            if name.startswith("pillar_")
        }
        self.assertEqual(set(self.EXPECTED_PILLARS), set(pillars))

        for name, expected_pose in self.EXPECTED_PILLARS.items():
            with self.subTest(pillar=name):
                model = pillars[name]
                self.assertEqual("true", model.findtext("static"))
                self.assertEqual(expected_pose, self._floats(model.findtext("pose")))
                link = model.find("link")
                self.assertIsNotNone(link.find("collision"))
                self.assertIsNotNone(link.find("visual"))
                for element in ("collision", "visual"):
                    self.assertEqual(
                        0.3,
                        float(link.findtext(
                            f"{element}/geometry/cylinder/radius"
                        )),
                    )
                    self.assertEqual(
                        0.6,
                        float(link.findtext(
                            f"{element}/geometry/cylinder/length"
                        )),
                    )

    def test_ground_plane_present(self):
        ground = self.models.get("ground_plane")
        self.assertIsNotNone(ground)
        self.assertEqual("true", ground.findtext("static"))
        self.assertIsNotNone(ground.find("link/collision"))
        self.assertIsNotNone(ground.find("link/visual"))

    def test_model_set_contains_only_ground_perimeter_and_pillars(self):
        expected_models = {
            "ground_plane",
            *self.EXPECTED_WALLS,
            *self.EXPECTED_PILLARS,
        }
        self.assertEqual(expected_models, set(self.models))

    def test_nearest_obstacle_surface_is_more_than_four_metres_from_spawn(self):
        surface_distances = []
        for name in self.EXPECTED_WALLS:
            model = self.models[name]
            x, y = self._floats(model.findtext("pose"))[:2]
            size = self._floats(
                model.findtext("link/collision/geometry/box/size")
            )
            width, depth = size[:2]
            dx = max(abs(x) - width / 2.0, 0.0)
            dy = max(abs(y) - depth / 2.0, 0.0)
            surface_distances.append(math.hypot(dx, dy))

        for name in self.EXPECTED_PILLARS:
            model = self.models[name]
            x, y = self._floats(model.findtext("pose"))[:2]
            radius = float(model.findtext(
                "link/collision/geometry/cylinder/radius"
            ))
            surface_distances.append(math.hypot(x, y) - radius)

        self.assertGreater(min(surface_distances), 4.0)

    def test_physics_profile_is_named_for_three_millisecond_step(self):
        physics = self.world.find("physics")
        self.assertIsNotNone(physics)
        self.assertEqual("3ms", physics.get("name"))

    def test_physics_step_is_three_milliseconds(self):
        physics = self.world.find("physics")
        self.assertIsNotNone(physics)
        self.assertEqual(0.003, float(physics.findtext("max_step_size")))


if __name__ == "__main__":
    unittest.main()
