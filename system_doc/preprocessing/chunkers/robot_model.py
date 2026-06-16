"""机器人模型分块器 — 处理 SDF/URDF/SRDF 文件，提取传感器参数和物理约束"""

from __future__ import annotations
import re
from pathlib import Path
from xml.etree import ElementTree as ET

from .base import BaseChunker, RawChunk


class RobotModelChunker(BaseChunker):
    """
    处理 .sdf/.urdf/.srdf/.xacro 机器人模型文件。

    提取目标:
    - sensor 定义（IMU、LiDAR、Camera 的参数：噪声、范围、频率）
    - joint 限制（effort、velocity、lower/upper）
    - plugin 配置（diff_drive 的 max_torque、wheel_separation 等）
    - link 物理属性（mass、inertia）
    - SRDF 语义信息（规划组、碰撞对、预定义姿态、末端执行器）
    """

    @property
    def supported_doc_type(self) -> str:
        return "robot_model"

    def chunk(self, content: str, file_path: Path) -> list[RawChunk]:
        # 处理 XML namespace（SDF 常带 namespace）
        content_clean = re.sub(r'\sxmlns[^"]*"[^"]*"', '', content)
        try:
            root = ET.fromstring(content_clean)
        except ET.ParseError:
            # 尝试不清理 namespace
            try:
                root = ET.fromstring(content)
            except ET.ParseError:
                return []

        chunks = []

        # 统一运行所有提取器 — 每个提取器内部自行判断是否有匹配元素
        # URDF/SDF 提取器
        self._extract_sensors(root, chunks, file_path)
        self._extract_joints(root, chunks, file_path)
        self._extract_links(root, chunks, file_path)
        self._extract_plugins(root, chunks, file_path)
        # SRDF 提取器
        self._extract_groups(root, chunks, file_path)
        self._extract_group_states(root, chunks, file_path)
        self._extract_end_effectors(root, chunks, file_path)
        self._extract_virtual_joints(root, chunks, file_path)
        self._extract_collision_matrix(root, chunks, file_path)
        self._extract_passive_joints(root, chunks, file_path)

        return chunks

    def _extract_sensors(self, root: ET.Element, chunks: list, file_path: Path):
        """提取所有 sensor 元素的参数"""
        for sensor in root.iter("sensor"):
            sensor_name = sensor.get("name", "unknown_sensor")
            sensor_type = sensor.get("type", "unknown")
            params = self._collect_leaf_values(sensor)
            if not params:
                continue

            raw_xml = ET.tostring(sensor, encoding="unicode", method="xml")
            chunks.append(RawChunk(
                content=raw_xml[:2000],  # 截断过长的 XML
                name=sensor_name,
                chunk_type="sensor",
                index=len(chunks),
                parent_heading=f"sensor/{sensor_type}",
                location=f"//sensor[@name='{sensor_name}']",
                extra={
                    "name": sensor_name,
                    "sensor_type": sensor_type,
                    **params,
                },
            ))

    def _extract_joints(self, root: ET.Element, chunks: list, file_path: Path):
        """提取 joint 限制和几何约束"""
        for joint in root.iter("joint"):
            joint_name = joint.get("name", "unknown_joint")
            joint_type = joint.get("type", "")
            limit = joint.find("limit") or joint.find(".//limit")
            axis = joint.find("axis") or joint.find(".//axis")
            origin = joint.find("origin")

            extra = {"name": joint_name, "joint_type": joint_type}
            has_constraint = False

            if limit is not None:
                for attr in ("lower", "upper", "effort", "velocity"):
                    val = limit.get(attr) or limit.findtext(attr)
                    if val:
                        extra[attr] = self._try_float(val)
                        has_constraint = True

            if axis is not None:
                for child in axis:
                    if child.text and child.text.strip():
                        extra[f"axis_{child.tag}"] = child.text.strip()

            if origin is not None:
                xyz = origin.get("xyz")
                rpy = origin.get("rpy")
                if xyz:
                    extra["origin_xyz"] = xyz
                    has_constraint = True
                if rpy:
                    extra["origin_rpy"] = rpy

            # continuous/revolute joints are always constraints
            if joint_type in ("continuous", "revolute", "prismatic"):
                has_constraint = True

            if has_constraint:
                chunks.append(RawChunk(
                    content=ET.tostring(joint, encoding="unicode")[:1000],
                    name=joint_name,
                    chunk_type="joint_limit",
                    index=len(chunks),
                    parent_heading="joints",
                    location=f"//joint[@name='{joint_name}']",
                    extra=extra,
                ))

    def _extract_links(self, root: ET.Element, chunks: list, file_path: Path):
        """提取 link 物理属性（mass、collision geometry）"""
        for link in root.iter("link"):
            link_name = link.get("name", "unknown_link")
            inertial = link.find("inertial") or link.find(".//inertial")
            collision = link.find("collision") or link.find(".//collision")

            extra = {"name": link_name}
            has_data = False

            if inertial is not None:
                mass_elem = inertial.find("mass")
                if mass_elem is not None:
                    val = mass_elem.get("value") or mass_elem.text
                    if val:
                        extra["mass"] = self._try_float(val.strip())
                        has_data = True

            if collision is not None:
                geom = collision.find("geometry") or collision.find(".//geometry")
                if geom is not None:
                    for shape in geom:
                        tag = shape.tag
                        if tag == "cylinder":
                            r = shape.get("radius")
                            l = shape.get("length")
                            if r:
                                extra["collision_radius"] = self._try_float(r)
                            if l:
                                extra["collision_length"] = self._try_float(l)
                            has_data = True
                        elif tag == "box":
                            s = shape.get("size")
                            if s:
                                extra["collision_box_size"] = s
                            has_data = True
                        elif tag == "sphere":
                            r = shape.get("radius")
                            if r:
                                extra["collision_radius"] = self._try_float(r)
                            has_data = True

            if has_data:
                chunks.append(RawChunk(
                    content=ET.tostring(link, encoding="unicode")[:1500],
                    name=link_name,
                    chunk_type="link_physics",
                    index=len(chunks),
                    parent_heading="links",
                    location=f"//link[@name='{link_name}']",
                    extra=extra,
                ))

    def _extract_plugins(self, root: ET.Element, chunks: list, file_path: Path):
        """提取 Gazebo plugin 配置参数"""
        for plugin in root.iter("plugin"):
            plugin_name = plugin.get("name", "")
            plugin_file = plugin.get("filename", "")
            params = self._collect_leaf_values(plugin)
            if not params:
                continue

            # 只保留含数值参数的 plugin
            numeric_params = {
                k: v for k, v in params.items()
                if isinstance(v, (int, float))
            }
            if not numeric_params:
                continue

            chunks.append(RawChunk(
                content=ET.tostring(plugin, encoding="unicode")[:2000],
                name=plugin_name or plugin_file,
                chunk_type="plugin_config",
                index=len(chunks),
                parent_heading="plugins",
                location=f"//plugin[@name='{plugin_name}']",
                extra={
                    "name": plugin_name,
                    "filename": plugin_file,
                    **params,
                },
            ))

    def _collect_leaf_values(self, elem: ET.Element) -> dict:
        """递归收集元素下所有叶节点的文本值"""
        result = {}
        for child in elem.iter():
            if child.text and child.text.strip() and len(list(child)) == 0:
                tag = child.tag.split("}")[-1] if "}" in child.tag else child.tag
                val = child.text.strip()
                result[tag] = self._try_float(val)
        return result

    def _try_float(self, s: str):
        """尝试转为数值"""
        try:
            if "." in s or "e" in s.lower():
                return float(s)
            return int(s)
        except (ValueError, TypeError):
            return s

    # =================================================================
    # SRDF 提取方法
    # =================================================================

    def _extract_groups(self, root: ET.Element, chunks: list, file_path: Path):
        """提取规划组定义（chain、joints、links、subgroups）"""
        for group in root.iter("group"):
            group_name = group.get("name", "unknown_group")
            extra = {"name": group_name}

            # chain 定义
            chain = group.find("chain")
            if chain is not None:
                extra["base_link"] = chain.get("base_link", "")
                extra["tip_link"] = chain.get("tip_link", "")

            # 显式 joint 列表
            joints = [j.get("name", "") for j in group.findall("joint")]
            if joints:
                extra["joints"] = joints

            # 显式 link 列表
            links = [l.get("name", "") for l in group.findall("link")]
            if links:
                extra["links"] = links

            # 被动关节
            passive = [p.get("name", "") for p in group.findall("passive_joint")]
            if passive:
                extra["passive_joints"] = passive

            chunks.append(RawChunk(
                content=ET.tostring(group, encoding="unicode")[:1500],
                name=group_name,
                chunk_type="planning_group",
                index=len(chunks),
                parent_heading="groups",
                location=f"//group[@name='{group_name}']",
                extra=extra,
            ))

    def _extract_group_states(self, root: ET.Element, chunks: list, file_path: Path):
        """提取预定义姿态（named states，如 'ready', 'home', 'extended'）"""
        for state in root.iter("group_state"):
            state_name = state.get("name", "unknown_state")
            group_name = state.get("group", "")
            joint_values = {}
            for jv in state.findall("joint"):
                jname = jv.get("name", "")
                jval = jv.get("value", "")
                if jname and jval:
                    joint_values[jname] = self._try_float(jval)

            extra = {
                "name": state_name,
                "group": group_name,
                "joint_values": joint_values,
            }

            chunks.append(RawChunk(
                content=ET.tostring(state, encoding="unicode")[:1000],
                name=f"{group_name}.{state_name}",
                chunk_type="predefined_pose",
                index=len(chunks),
                parent_heading="group_states",
                location=f"//group_state[@name='{state_name}']",
                extra=extra,
            ))

    def _extract_end_effectors(self, root: ET.Element, chunks: list, file_path: Path):
        """提取末端执行器定义"""
        for ee in root.iter("end_effector"):
            ee_name = ee.get("name", "unknown_ee")
            extra = {
                "name": ee_name,
                "group": ee.get("group", ""),
                "parent_link": ee.get("parent_link", ""),
                "parent_group": ee.get("parent_group", ""),
            }
            chunks.append(RawChunk(
                content=ET.tostring(ee, encoding="unicode"),
                name=ee_name,
                chunk_type="end_effector",
                index=len(chunks),
                parent_heading="end_effectors",
                location=f"//end_effector[@name='{ee_name}']",
                extra=extra,
            ))

    def _extract_virtual_joints(self, root: ET.Element, chunks: list, file_path: Path):
        """提取虚拟关节（机器人与世界坐标系的连接方式）"""
        for vj in root.iter("virtual_joint"):
            vj_name = vj.get("name", "unknown_vj")
            extra = {
                "name": vj_name,
                "type": vj.get("type", ""),
                "parent_frame": vj.get("parent_frame", ""),
                "child_link": vj.get("child_link", ""),
            }
            chunks.append(RawChunk(
                content=ET.tostring(vj, encoding="unicode"),
                name=vj_name,
                chunk_type="virtual_joint",
                index=len(chunks),
                parent_heading="virtual_joints",
                location=f"//virtual_joint[@name='{vj_name}']",
                extra=extra,
            ))

    def _extract_collision_matrix(self, root: ET.Element, chunks: list, file_path: Path):
        """提取碰撞对禁用矩阵（ACM - Allowed Collision Matrix）

        将所有 disable_collisions 汇总为一个 block，而不是每对一个 block，
        因为碰撞矩阵的语义是整体的。
        """
        pairs = []
        for dc in root.iter("disable_collisions"):
            link1 = dc.get("link1", "")
            link2 = dc.get("link2", "")
            reason = dc.get("reason", "")
            pairs.append({"link1": link1, "link2": link2, "reason": reason})

        if not pairs:
            return

        # 按 reason 分组汇总
        adjacent_pairs = [p for p in pairs if p["reason"] == "Adjacent"]
        never_pairs = [p for p in pairs if p["reason"] == "Never"]
        default_pairs = [p for p in pairs if p["reason"] == "Default"]
        other_pairs = [p for p in pairs
                       if p["reason"] not in ("Adjacent", "Never", "Default")]

        extra = {
            "name": "allowed_collision_matrix",
            "total_pairs": len(pairs),
            "adjacent_pairs": len(adjacent_pairs),
            "never_pairs": len(never_pairs),
            "default_pairs": len(default_pairs),
            "other_pairs": len(other_pairs),
            "pairs": pairs,
        }

        # 生成可读内容
        content_lines = ["# Allowed Collision Matrix (disable_collisions)"]
        content_lines.append(f"Total disabled pairs: {len(pairs)}")
        content_lines.append(f"Adjacent: {len(adjacent_pairs)}, "
                             f"Never: {len(never_pairs)}, "
                             f"Default: {len(default_pairs)}")
        content_lines.append("")
        for p in pairs[:30]:  # 最多展示 30 对
            content_lines.append(
                f"  {p['link1']} <-> {p['link2']} ({p['reason']})")
        if len(pairs) > 30:
            content_lines.append(f"  ... and {len(pairs) - 30} more pairs")

        chunks.append(RawChunk(
            content="\n".join(content_lines),
            name="allowed_collision_matrix",
            chunk_type="collision_matrix",
            index=len(chunks),
            parent_heading="collision_pairs",
            location="//disable_collisions",
            extra=extra,
        ))

    def _extract_passive_joints(self, root: ET.Element, chunks: list, file_path: Path):
        """提取顶层被动关节声明（不参与规划的关节）"""
        passive_joints = []
        for pj in root.findall("passive_joint"):
            name = pj.get("name", "")
            if name:
                passive_joints.append(name)

        if not passive_joints:
            return

        chunks.append(RawChunk(
            content=f"Passive joints (excluded from planning): {passive_joints}",
            name="passive_joints",
            chunk_type="config",
            index=len(chunks),
            parent_heading="passive_joints",
            location="//passive_joint",
            extra={"name": "passive_joints", "joints": passive_joints},
        ))
