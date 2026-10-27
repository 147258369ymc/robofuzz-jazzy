# Nav2 新增 Oracle 设计方案

## 背景
- 当前：5 个粗粒度 oracle（到达/碰撞/超时/卡死/假失败），722 轮发现 0 bug
- 问题：只检测任务级失败，检测不到过程级违规
- 目标：基于现有 bag 数据，新增 3-4 个更深层次 oracle

## 可用数据（当前 bag 录制）
- `/robofuzz/nav2_goal`: 目标位姿
- `/robofuzz/nav2_result`: 导航结果码
- `/sim_ground_truth_pose`: 真值轨迹（~30Hz）
- `/bumper_contact`: 碰撞事件
- **预处理 params**: `nav2_jazzy.params.yaml`（goal_checker/progress_checker配置）

## 新增 Oracle 候选（从 42 个 OracleIR 规格中筛选）

### Oracle 6: **路径效率异常** (NAV2_PATH_EFFICIENCY_ERROR)
**来源**: OracleIR 未明确定义，但隐含在性能预期中
**检测**: 实际路径长度 vs 理论最短距离（起点到终点直线）
**逻辑**:
```python
actual_path_length = sum(轨迹相邻点距离)  # 从 truth 计算
straight_line = hypot(goal - start)
efficiency = actual_path_length / straight_line
if result == SUCCEEDED and efficiency > EFFICIENCY_THRESHOLD:
    # 绕了太远的路（可能陷入局部最优、震荡、重复规划）
    NAV2_PATH_EFFICIENCY_ERROR
```
**阈值标定**: 
- Clean campaign 测量 efficiency 分布
- 取 95th percentile × 1.2 作为阈值（初始值 2.5 = 绕路2.5倍）

**能发现的 bug**:
- Planner 陷入局部最优，绕远路
- Costmap 膨胀过度，把直路标记为不可通行
- Recovery behavior 触发导致反复后退/转向

---

### Oracle 7: **导航时间异常** (NAV2_NAVIGATION_TIME_ERROR)
**来源**: OracleIR `progress_checker.movement_time_allowance` (10s)
**检测**: duration vs goal 距离，检测异常缓慢
**逻辑**:
```python
expected_time = straight_line / EXPECTED_SPEED  # 0.3 m/s 保守估计
actual_time = duration
if result == SUCCEEDED and actual_time > expected_time * TIME_TOLERANCE_FACTOR:
    # 到达了但耗时异常长（卡顿、反复retry、速度过慢）
    NAV2_NAVIGATION_TIME_ERROR
```
**阈值标定**:
- Clean campaign 测 time/distance 比值分布
- 取 95th percentile × 1.3（初始值：期望时间 × 3）

**能发现的 bug**:
- Progress checker 过松，卡顿未检测
- Controller 速度过慢（vx 参数配置错误）
- Recovery 频繁触发浪费时间

---

### Oracle 8: **姿态稳定性违规** (NAV2_POSE_OSCILLATION_ERROR)
**来源**: OracleIR `rotational_acc_lim` / `wz_max`（间接）
**检测**: 真值 yaw 抖动/振荡（高频往复）
**逻辑**:
```python
yaw_changes = [abs(wrap_pi(truth[i+1].yaw - truth[i].yaw)) for i in range(len(truth)-1)]
# 检测连续方向反转（振荡）
oscillations = sum(1 for i in range(len(yaw_changes)-1) if yaw_changes[i] > 0.1 and sign(yaw_changes[i+1]) != sign(yaw_changes[i]))
if result == SUCCEEDED and oscillations > OSCILLATION_THRESHOLD:
    # 到达了但姿态剧烈抖动（controller tuning 差、costmap 噪声、动态障碍物反应过激）
    NAV2_POSE_OSCILLATION_ERROR
```
**阈值标定**:
- Clean campaign 测 oscillation count 分布
- 取 max + 5（初始值：10 次振荡）

**能发现的 bug**:
- MPPI controller 参数不当（vx_std/wz_std 过大）
- Costmap 频繁更新导致路径抖动
- Transform tolerance 过大导致姿态估计抖动

---

### Oracle 9: **到达精度退化** (NAV2_ARRIVAL_PRECISION_DEGRADED)
**来源**: OracleIR `xy_goal_tolerance`/`yaw_goal_tolerance` + `stateful`
**检测**: SUCCEEDED 但到达误差显著大于参数配置容差
**逻辑**:
```python
# 从 params 读取 goal_checker 配置
xy_tolerance_config = params['controller_server']['general_goal_checker']['xy_goal_tolerance']  # 默认 0.25
yaw_tolerance_config = params['...']['yaw_goal_tolerance']  # 默认 0.25

arr_xy, arr_yaw = _arrival_error(...)
if result == SUCCEEDED:
    if arr_xy > xy_tolerance_config * PRECISION_MARGIN:
        # 到达了但精度显著差于配置（goal_checker 实现bug、stateful=True未生效、控制器超调）
        NAV2_ARRIVAL_PRECISION_DEGRADED
```
**阈值标定**:
- PRECISION_MARGIN = 1.5（到达误差 > 配置容差 × 1.5 算退化）
- 需从 params.yaml 动态读取配置值

**能发现的 bug**:
- goal_checker 的 stateful=True 未生效（到达后立即漂移）
- xy_goal_tolerance 配置被覆盖/未加载
- Controller 终止条件与 goal_checker 不一致

---

## 实现优先级排序

| Oracle | 价值 | 复杂度 | 推荐 |
|---|---|---|---|
| 6. 路径效率异常 | 高（能发现规划问题）| 低（只需轨迹计算）| ✅ **优先** |
| 7. 导航时间异常 | 高（能发现性能退化）| 低（duration/distance）| ✅ **优先** |
| 8. 姿态稳定性违规 | 中（能发现controller tuning问题）| 中（需yaw时间序列分析）| ✅ 次要 |
| 9. 到达精度退化 | 中（能发现参数不一致）| 中（需读params文件）| ⚠️ 可选 |

**建议先实现 6+7（高价值、低复杂度），再实现 8**。Oracle 9 需要参数读取机制，可后续迭代。

## 集成策略
1. 在 `oracles/nav2.py` 新增 3 个判据（Criterion 5/6/7）
2. 新增对应 feedback（path_efficiency, time_efficiency, pose_oscillation_count）
3. 阈值初始值硬编码，标注"需 clean campaign 标定"
4. 在当前 1000 轮长跑重启后验证（预期能发现新类型 bug）

## 预期收益
- 当前 5 个 oracle：0 bug / 722 轮
- 新增 3 个过程级 oracle 后：预期发现 **路径规划质量问题**、**性能退化**、**controller tuning 缺陷**
- 对比：slam 1000 轮发现 18 个 bug（地图一致性问题）；nav2 应能发现类似数量的导航质量问题
