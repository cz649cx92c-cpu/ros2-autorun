# autorun_final

`autorun_final` 把两个现有工程接在一起：

- `autorun2` / Odin 全局定位与任务回放
- `linerun` / 行间局部中线约束与障碍物约束

## 设计原则

- 行间正常前进时：
  - 以 `linerun` 的局部跟踪为主
  - 同时保留一部分全局定位回放权重
- 换行、横移、倒车、到路尽头、找不到中线时：
  - 切到全局定位主导
  - 局部权重降为 0
- 如果 `linerun` 判断前方通道被障碍物堵住：
  - 局部约束优先，先停住，避免硬闯

## 目录

- 任务与日志写入 `autorun_final/`
- Mission 录制仍复用 `autorun` 的录制逻辑，但定位会走 Odin

## 常用命令

### 建图

```bash
cd /root/ugv/autorun_final
python3 main.py map --map-name lab_map
```

### 纯定位

```bash
python3 main.py localization \
  --db /root/ugv/autorun_final/maps/lab_map/lab_map.bin
```

### 录制全局任务

```bash
python3 main.py record \
  --db /root/ugv/autorun_final/maps/lab_map/lab_map.bin \
  --mission-name mission_a
```

### 混合自动运行

```bash
python3 main.py autorun \
  --db /root/ugv/autorun_final/maps/lab_map/lab_map.bin \
  --mission /root/ugv/autorun_final/missions/mission_a.json \
  --line-model /root/ugv/line/models/your_model.rknn \
  --line-require-npu \
  --line-source /dev/video0 \
  --line-classes 1 \
  --line-target-class 0
```

## 关键参数

- `--local-weight-in-row`: 行间局部控制权重，默认 `0.75`
- `--global-weight-in-row`: 行间全局控制权重，默认 `0.25`
- `--line-*`: 直接传给 `linerun` 运行所需的主要参数

## 当前实现说明

- 局部控制通过 `linerun` 的 `ROS control` 模式运行
- `autorun_final` 订阅 `linerun` 的状态和 `cmd_vel`
- 最终底盘 CAN 命令由 `autorun_final` 统一下发
- 行间前进段优先采用 `linerun` 输出；换行/倒车/crab 段只走全局
