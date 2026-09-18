import re
import struct
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
ASSETS = ROOT / "assets"
sys.path.insert(0, str(TOOLS))

from svg_diagrams import OUTPUT_STEMS, SvgCanvas
from generate_diagrams import BUILDERS


class DiagramTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run([sys.executable, str(TOOLS / "generate_diagrams.py")], check=True)

    def test_svg_and_png_outputs_exist(self):
        self.assertEqual(set(BUILDERS), set(OUTPUT_STEMS))
        for stem in OUTPUT_STEMS:
            self.assertTrue((ASSETS / f"{stem}.svg").is_file())
            png_path = ASSETS / f"{stem}.png"
            self.assertTrue(png_path.is_file())
            with png_path.open("rb") as image:
                self.assertEqual(image.read(8), b"\x89PNG\r\n\x1a\n")
                self.assertEqual(image.read(4), b"\x00\x00\x00\r")
                self.assertEqual(image.read(4), b"IHDR")
                width, height = struct.unpack(">II", image.read(8))
                self.assertEqual((width, height), (2400, 800))

    def test_svg_canvas_and_square_corners(self):
        for stem in OUTPUT_STEMS:
            root = ET.parse(ASSETS / f"{stem}.svg").getroot()
            self.assertEqual(root.attrib.get("viewBox"), "0 0 2400 800")
            for rect in root.findall(".//{http://www.w3.org/2000/svg}rect"):
                self.assertNotIn("rx", rect.attrib)
                self.assertNotIn("ry", rect.attrib)

    def test_text_uses_explicit_readable_font_sizes(self):
        for stem in OUTPUT_STEMS:
            root = ET.parse(ASSETS / f"{stem}.svg").getroot()
            for node in root.findall(".//{http://www.w3.org/2000/svg}text"):
                self.assertIn("font-size", node.attrib)
                self.assertGreaterEqual(float(node.attrib["font-size"]), 42)
        canvas = SvgCanvas()
        canvas.text(42, 42, "&<>")
        rendered = canvas.render()
        self.assertIn("&amp;&lt;&gt;", rendered)
        ET.fromstring(rendered)

    def test_polylines_are_orthogonal(self):
        for stem in OUTPUT_STEMS:
            root = ET.parse(ASSETS / f"{stem}.svg").getroot()
            for line in root.findall(".//{http://www.w3.org/2000/svg}polyline"):
                points = [tuple(map(float, pair.split(","))) for pair in line.attrib["points"].split()]
                for first, second in zip(points, points[1:]):
                    self.assertTrue(first[0] == second[0] or first[1] == second[1])

    def test_key_labels_are_present(self):
        expected = {
            "01_三任务总体链路": ("规范分块", "反馈调度", "候选证据包"),
            "02_规范到Oracle链路": ("SpecBlock", "自动化边界", "不可判定"),
            "03_语义反馈与种子调度": ("容量 ≤ 50", "边界压力 → 边界推进", "组合优化结果"),
            "04_偏轨证据与根因归并": ("不可判定", "去重异常签名", "非一一对应"),
        }
        for stem, labels in expected.items():
            svg = (ASSETS / f"{stem}.svg").read_text(encoding="utf-8")
            plain = re.sub(r"<[^>]+>", "", svg)
            for label in labels:
                self.assertIn(label, plain)


if __name__ == "__main__":
    unittest.main()
