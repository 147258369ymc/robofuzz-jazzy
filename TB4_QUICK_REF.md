# TurtleBot4 Jazzy - 快速参考

## 🚀 快速启动

### Headless模式（生产推荐）
```bash
docker run --rm -it \
  --name robofuzz-jazzy-tb4 \
  --network host --ipc host \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest bash

source /opt/ros/jazzy/setup.bash && cd /work/src_jazzy
TURTLEBOT4_HEADLESS=1 python3 fuzzer.py \
  --target-profile turtlebot4_jazzy \
  --schedule sequence --seqlen 3 \
  --maxloop 100 --interval 1.0 --no-cov \
  --logdir /work/logs_jazzy/tb4_prod
```

### GUI模式（调试推荐）
```bash
# 宿主机
xhost +SI:localuser:root

# 容器
docker run --rm -it \
  --name robofuzz-jazzy-tb4-gui \
  --network host --ipc host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ymc/RoboFuzz-jazzy:/work \
  -w /work \
  robofuzz-jazzy:latest bash

source /opt/ros/jazzy/setup.bash && cd /work/src_jazzy
TURTLEBOT4_HEADLESS=0 TURTLEBOT4_WORLD=empty \
  python3 fuzzer.py \
  --target-profile turtlebot4_jazzy \
  --schedule sequence --seqlen 3 \
  --maxloop 10 --interval 1.0 --no-cov \
  --logdir /work/logs_jazzy/tb4_gui
```

## 📊 状态

- ✅ **Profile**: `tested`
- ✅ **Oracle**: `verified`
- ✅ **测试**: 29/29 通过
- ✅ **生产就绪**: Headless 100%, GUI 100%

## 🎯 环境变量

| 变量 | 默认值 | 选项 |
|------|--------|------|
| `TURTLEBOT4_HEADLESS` | `1` | `0`=GUI, `1`=headless |
| `TURTLEBOT4_WORLD` | `warehouse` | `empty`, `depot`, `warehouse`, `maze` |
| `TURTLEBOT4_MODEL` | `standard` | `standard`, `lite` |

## 📈 性能

| 模式 | 场景 | 成功率 | 耗时/轮 |
|------|------|--------|---------|
| Headless | 任意 | 100% | ~45s |
| GUI | empty | 100% | ~28s |
| GUI | warehouse | 20-40% | ~45s |

## 💡 推荐

- **调试**: GUI + empty
- **生产**: Headless + 任意场景
- **真实环境**: Headless + warehouse

## 📚 文档

- `docs/TB4_ADAPTATION_SUMMARY.md` — 完整适配
- `docs/TB4_GUI_USAGE.md` — GUI使用
- `TB4_GUI_IMPROVEMENT.md` — GUI改进
- `TB4_WORK_SUMMARY.md` — 工作总结
