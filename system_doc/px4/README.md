# PX4 System Documentation Index

本目录包含 PX4 自动驾驶系统的规约文档，用于 RoboFuzz 项目的参数约束提取和 fuzzing 输入生成。

**PX4 版本: v1.12.0** (commit a264541861ee876fe4a7d13dad430f69439799a4)

## 文件列表

| 文件 | 内容 | 来源 |
|------|------|------|
| [parameter_reference_v1.12.md](parameter_reference_v1.12.md) | **完整参数参考** (1450参数, 52组) | 容器内 make parameters_metadata 生成 |
| [parameters.json](parameters.json) | 参数元数据 JSON 原始文件 | build/px4_sitl_rtps/parameters.json |
| [parameters.xml](parameters.xml) | 参数元数据 XML 原始文件 | build/px4_sitl_rtps/parameters.xml |
| [parameters_and_configurations.md](parameters_and_configurations.md) | PX4 参数系统架构、C/C++ API、元数据定义方式 | docs.px4.io |
| [uorb_message_reference.md](uorb_message_reference.md) | uORB 消息列表及关键消息字段定义 | docs.px4.io |
| [vehicle_command_reference.md](vehicle_command_reference.md) | VehicleCommand 消息及所有命令 ID 定义 | docs.px4.io |
| [mavlink_common_messages.md](mavlink_common_messages.md) | MAVLink 通用消息定义（HEARTBEAT, COMMAND_LONG 等） | mavlink.io |
| [mavlink_parameter_protocol.md](mavlink_parameter_protocol.md) | MAVLink 参数协议（读写流程、PX4 实现细节） | mavlink.io |

## 数据来源

参数规约数据直接从 robofuzz_original 容器中的 PX4-Autopilot v1.12.0 构建产物提取：

```
容器: robofuzz_original
路径: /robofuzz/targets/PX4-Autopilot
Commit: a264541861ee876fe4a7d13dad430f69439799a4
构建目标: px4_sitl_rtps
参数文件: build/px4_sitl_rtps/parameters.json (1450 parameters)
```
