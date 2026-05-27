"""机器人模型分块器 — 处理 SDF/URDF 文件，提取传感器参数和物理约束"""

from __future__ import annotations
import re
from pathlib import Path
from xml.etree import ElementTree as ET

from .base import BaseChunker, RawChunk


class RobotModelChunker(BaseChunker):
    """
    处理 .sdf/.urdf 机器人模型文件。

    提取目标:
    - sensor 定义（IMU、LiDAR、Camera 的参数：噪声、范围、频率）
    - joint 限制（effort、velocity、lower/upper）
    - plugin 配置（diff_drive 的 max_torque、wheel_separation 等）
    - link 物理属性（mass、inertia）
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
        self._extract_sensors(root, chunks, file_path)
        self._extract_joints(root, chunks, file_path)
        self._extract_links(root, chunks, file_path)
        self._extract_plugins(root, chunks, file_path)
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
