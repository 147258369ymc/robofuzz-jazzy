# OracleIR 生成模板

本模板指导 LLM Agent 从 SpecBlock 生成规范形式的 OracleIR YAML。

## 输入

Agent 接收以下 SpecBlock 信息：

```json
{
  "block_id": "{system}.parameter.{PARAM_NAME}",
  "block_type": "parameter",
  "structured_fields": {
    "name": "{PARAM_NAME}",
    "default": 12.0,
    "min": 0.0,
    "max": 20.0,
    "units": "m/s",
    "shortDesc": "Maximum horizontal velocity",
    "group": "Position Control"
  },
  "tags": ["velocity_constraint"],
  "references": ["RELATED_PARAM"]
}
```

## 输出模板

```yaml
# ─── 必填：身份标识 ───
id: "{system}.{category}.{name}"
type: "{range_bound|validity|norm_constraint|temporal_consistency|cross_sensor}"
system: "{system}"
version: "{version}"

# ─── 必填：适用条件 ───
scope:
  flight_modes: []          # 留空表示所有模式
  require_airborne: true    # 是否仅在空中检查

# ─── 必填：观测变量 ───
observations:
  - name: "{var_name}"
    topic: "/{TopicName}"       # 使用目标系统的实际 topic 名称
    field: "{field_name}"
    unit: "{unit}"
    index: null             # 数组字段填索引，标量填 null

# ─── 可选：参数依赖 ───
parameters:
  - name: "{PARAM_NAME}"        # 必须与 SpecBlock.name 完全一致
    source: "{system}.parameters"
    default: 12.0               # 必须与 SpecBlock.structured_fields.default 一致
    unit: "{units}"             # 必须与 SpecBlock.structured_fields.units 一致

# ─── 可选：常量 ───
constants:
  - name: "{CONST_NAME}"
    value: 9.81
    unit: "m/s2"

# ─── 可选：派生量 ───
derived:
  - name: "{derived_var}"
    expr: "{受限表达式}"         # 只允许: 算术 + sqrt/abs/min/max/norm/mean/degrees/acos
    unit: "{unit}"

# ─── 必填（或有 feedback）：断言 ───
assertions:
  - expr: "{derived_var} <= param({PARAM_NAME}) + tolerance"
    tolerance: 0.5              # 容差值
    severity: "error"           # error | warning
    message: "{描述}: {derived_var:.2f} {unit}"

# ─── 必填：时间窗口 ───
window:
  type: "every_sample"          # every_sample | sequential_pairs | aggregation

# ─── 可选：语义反馈 ───
feedback:
  - name: "{feedback_name}"
    metric: "{derived_var}"
    direction: "maximize"       # maximize | minimize | zero | target

# ─── 必填：来源追溯 ───
provenance:
  - chunk_id: "{block_id}"      # 必须是 SpecIndex 中存在的 block_id
    evidence: "{shortDesc}"     # 从 SpecBlock 中摘录的证据文本
```

## 生成规则

### 规则 1：类型选择

| SpecBlock 特征 | 选择的 type |
|---------------|-------------|
| shortDesc 含 "Maximum/Minimum/Limit" | `range_bound` |
| shortDesc 含 "valid/NaN/Inf" | `validity` |
| shortDesc 含 "norm/quaternion/unit" | `norm_constraint` |
| 需要前后采样对比（导数/积分） | `temporal_consistency` |
| 涉及多个不同 topic 交叉验证 | `cross_sensor` |

### 规则 2：断言方向

| SpecBlock.shortDesc 语义 | assertion 比较符 | feedback.direction |
|-------------------------|-----------------|-------------------|
| "Maximum X" / "X limit" | `<=` | `maximize` |
| "Minimum X" | `>=` | `minimize` |

### 规则 3：窗口选择

| 场景 | window.type | 说明 |
|------|-------------|------|
| 单采样点即可判断 | `every_sample` | 速度/角度/加速度限制 |
| 需要 Δt 计算导数或积分 | `sequential_pairs` | jerk、vel→pos 一致性 |
| 需要整段统计特征 | `aggregation` | 悬停漂移、传感器统计 |

### 规则 4：数值来源

- `parameters.default` **必须**从 SpecBlock.structured_fields.default 抄写，禁止凭记忆填写
- `parameters.unit` **必须**从 SpecBlock.structured_fields.units 抄写
- `provenance.chunk_id` **必须**是 SpecIndex 中存在的 block_id

### 规则 5：受限表达式

允许：`+ - * / % ** sqrt abs min max norm mean degrees radians acos is_valid param()`

禁止：循环、条件、函数定义、import、字符串操作

## 生成流程

```
1. 接收 SpecBlock 列表（按 tag 过滤：velocity/attitude/altitude_constraint）
2. 对每个约束类参数：
   a. 确定 type（根据规则 1）
   b. 确定关联的 observation topic/field（从 SpecIndex.reference_graph 查找）
   c. 填写 parameters（从 structured_fields 精确抄写）
   d. 编写 derived 表达式
   e. 编写 assertion（根据规则 2 确定方向）
   f. 选择 window（根据规则 3）
   g. 填写 provenance（chunk_id = block_id）
3. 输出 YAML
```
