import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import content


class ContentTests(unittest.TestCase):
    def test_seven_pages_and_six_metrics(self):
        self.assertEqual(len(content.SLIDES), 7)
        self.assertEqual(len(content.METRICS), 6)

    def test_metric_states_match_design(self):
        states = [row["status"] for row in content.METRICS]
        self.assertEqual(
            states,
            ["达到数量规模", "部分达到", "达到：8类", "仿真侧达到", "待形成正式证据", "待形成正式证据"],
        )

    def test_three_tasks_only(self):
        self.assertEqual([item["id"] for item in content.TASKS], [1, 2, 3])

    def test_natural_method_copy_is_preserved(self):
        joined = "\n".join(
            line for slide in content.TECH_SLIDES for line in slide["method_lines"]
        )
        self.assertIn("先按文件类型分别处理", joined)
        self.assertIn("测试输入不再局限于修改一条消息的数值", joined)
        self.assertIn("发现偏轨后，同时保存测试输入", joined)
        self.assertNotIn("检索—生成—校验", joined)
        self.assertNotIn("错误签名—共现—级联去重", joined)

    def test_forbidden_claims_are_absent(self):
        corpus = content.all_visible_text()
        forbidden = (
            "34个漏洞",
            "34 个漏洞已发现",
            "151条Oracle全部执行",
            "151 条 Oracle 全部执行",
            "覆盖率提升33.9%",
            "覆盖率提升 33.9%",
            "已完成虚实差分",
            "四类输入全部完成",
        )
        for phrase in forbidden:
            self.assertNotIn(phrase, corpus)

    def test_required_boundaries_are_present(self):
        corpus = content.all_visible_text()
        for phrase in (
            "34 条 Issue 不等于 34 个漏洞",
            "组合优化结果，不等同于语义约束覆盖率",
            "成熟度不一",
            "下一阶段｜尚未开展",
        ):
            self.assertIn(phrase, corpus)


if __name__ == "__main__":
    unittest.main()
