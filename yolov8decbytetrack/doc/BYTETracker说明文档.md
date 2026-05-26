# BYTETracker 多目标跟踪器说明文档

## 概述

BYTETracker 是基于 [ByteTrack](https://arxiv.org/abs/2110.06864) 论文的 C++ 多目标跟踪（MOT）实现，用于对 YOLOv8 检测到的车辆进行逐帧跟踪，为每个目标分配稳定且唯一的 ID。

在项目 pipeline 中的位置：

```
海康相机采集帧 → YOLOv8 TensorRT 检测(车辆) → BYTETracker 跟踪 → 装甲板检测 → ROS2 发布
```

## 核心问题：为什么需要跟踪器

检测器逐帧输出目标框，但**不提供帧间关联**。例如第 N 帧检测到车辆 A、B，第 N+1 帧检测到车辆 B、C——检测器不会告诉你第 N 帧的 B 和第 N+1 帧的 B 是同一个目标。

跟踪器解决的就是这个**帧间目标关联（Data Association）**问题，给每个物理目标分配一个跨帧不变的 ID。

## 模块架构

```
BYTETracker (跟踪主控)
├── STrack     (单条轨迹 - 每个被跟踪的目标)
│   ├── KalmanFilter (卡尔曼滤波器 - 运动预测/更新)
│   └── Rect         (边界框 - TLWH格式)
├── lapjv      (匈牙利算法 - 最优匹配求解)
└── Object     (检测结果输入)
```

## 各模块详述

### 1. BYTETracker — 跟踪主控

**文件**: `include/BYTETracker.h`, `src/BYTETracker.cpp`

**职责**: 每帧接收检测结果，维护轨迹的完整生命周期。

**构造参数**:

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `frame_rate` | 30 | 视频帧率 |
| `track_buffer` | 90 | 丢失后保留帧数（`max_time_lost = frame_rate / 30 * track_buffer`） |
| `track_thresh` | 0.4 | 高低分检测的分界线 |
| `high_thresh` | 0.6 | 新建轨迹所需的最低置信度 |
| `match_thresh` | 0.75 | IoU 匹配阈值 |

**内部维护的轨迹状态**:
- `tracked_stracks_`: 活跃跟踪轨迹
- `lost_stracks_`: 暂时丢失的轨迹
- `removed_stracks_`: 已删除的轨迹

**核心方法**: `update(const std::vector<Object>& objects)` → 返回当前活跃的轨迹列表

### 2. STrack — 单条轨迹

**文件**: `include/STrack.h`, `src/STrack.cpp`

**职责**: 表示一个被跟踪的目标个体，封装其运动状态和身份信息。

**状态机**:

```
New → Tracked → Lost → Removed
              ↑   ↓
              └───┘ (reActivate)
```

- **New**: 刚创建，尚未确认
- **Tracked**: 正在被跟踪，每帧更新
- **Lost**: 暂时丢失（遮挡/漏检），仍保留一段时间
- **Removed**: 永久删除

**关键属性**:
- `track_id_`: 唯一标识 ID（跨帧不变）
- `frame_id_`: 最后更新的帧号
- `start_frame_id_`: 轨迹起始帧号
- `tracklet_len_`: 已跟踪的帧数
- `score_`: 当前检测置信度
- `mean_ / covariance_`: 卡尔曼滤波器的状态均值和协方差

**关键方法**:
- `activate()`: 首次确认轨迹，初始化卡尔曼滤波器
- `predict()`: 用卡尔曼滤波器预测下一帧位置
- `update()`: 用新的检测结果更新轨迹和卡尔曼滤波器
- `reActivate()`: 重新激活一条 Lost 轨迹

### 3. KalmanFilter — 卡尔曼滤波器

**文件**: `include/KalmanFilter.h`, `src/KalmanFilter.cpp`

**职责**: 对目标运动进行建模和预测，是跟踪器的数学核心。

**状态向量** (8维): `[x, y, a, h, vx, vy, va, vh]`
- `x, y`: 边界框中心坐标
- `a`: 宽高比 (width/height)
- `h`: 高度
- `vx, vy, va, vh`: 上述四个量的速度

**观测向量** (4维): `[x, y, a, h]`（检测器输出）

**运动模型**: 匀速线性运动（8x8 状态转移矩阵，位移 = 速度 × dt）

**三个核心操作**:

1. **initiate** (初始化): 用第一次检测初始化状态均值和协方差矩阵
2. **predict** (预测): `x' = F * x`，协方差 `P' = F * P * F^T + Q`（过程噪声）
3. **update** (更新): 用新检测值通过卡尔曼增益修正状态估计

过程噪声由 `std_weight_position`（默认 0.1）和 `std_weight_velocity`（默认 1/80 ≈ 0.0125）控制，噪声大小正比于目标高度。

### 4. lapjv — 匈牙利算法

**文件**: `include/lapjv.h`, `src/lapjv.cpp`

**职责**: 求解**线性分配问题**，即给定代价矩阵，找到使总代价最小的匹配方案。

这是 Jonker-Volgenant 算法的稠密矩阵实现，用于替代经典的 Hungarian 算法，时间复杂度 O(n³)。

在跟踪器中，代价矩阵 = `1 - IoU`（IoU 越高代价越小），阈值限制最大允许代价。

### 5. Rect — 边界框

**文件**: `include/Rect.h`, `src/Rect.cpp`

**职责**: 表示检测框，支持三种格式互转。

| 格式 | 表示 | 用途 |
|------|------|------|
| TLWH | `[x, y, width, height]` | 内部存储 / 检测器输出 |
| TLBR | `[tl_x, tl_y, br_x, br_y]` | IoU 计算 |
| XYAH | `[cx, cy, aspect, height]` | 卡尔曼滤波器状态 |

## update() 主流程详解

每帧调用 `tracker_.update(objects)` 时的内部流程：

```
Step 1: 分离检测
   ├── prob ≥ track_thresh(0.4) → det_stracks (高分检测)
   └── prob <  track_thresh(0.4) → det_low_stracks (低分检测)

Step 2: 第一次关联（高分检测 ↔ 已有轨迹）
   ├── 对 strack_pool 中每条轨迹做卡尔曼预测
   ├── 计算 IoU 距离矩阵
   ├── 匈牙利匹配 (threshold = match_thresh)
   ├── 匹配成功 → update 轨迹
   ├── 未匹配的轨迹 → remain_tracked_stracks
   └── 未匹配的检测 → remain_det_stracks

Step 3: 第二次关联（低分检测 ↔ 剩余轨迹）
   ├── 用更低阈值 0.5 匹配
   ├── 匹配成功 → update 轨迹
   └── 未匹配的轨迹 → markAsLost()

Step 4: 处理新轨迹
   ├── 未确认轨迹匹配剩余检测（阈值 0.7）
   ├── 未匹配的未确认轨迹 → markAsRemoved()
   └── 剩余高分检测 → 新建轨迹 activate()

Step 5: 状态清理
   ├── 超时 Lost 轨迹 → markAsRemoved()
   ├── 合并列表，去重
   └── 返回 activated 的 tracked_stracks
```

## BYTE 算法的核心思想

传统的 MOT 方法通常**丢弃低分检测**，只在高分检测上做匹配。BYTE 的关键创新是：

1. **高分检测**优先匹配（更可靠，优先建立关联）
2. **低分检测**用于"挽救"未匹配的轨迹（这些轨迹对应的目标可能被遮挡导致分数下降）

这样既保证了跟踪精度（高分匹配），又降低了 ID 切换和漏跟（低分挽救）。

## 在项目中的使用方式

```cpp
// 初始化（在 inference_node 构造函数中）
BYTETracker tracker_(30, 90, 0.3f, 0.6f, 0.7f);

// 每帧调用
std::vector<Object> objects = detectionsToObjects(detections);  // 检测结果转换
std::vector<std::shared_ptr<STrack>> tracked = tracker_.update(objects);

// 使用跟踪结果
for (const auto& track_ptr : tracked) {
    auto rect = track_ptr->getRect();          // 边界框
    size_t id = track_ptr->getTrackId();       // 唯一ID
    float score = track_ptr->getScore();       // 置信度
}
```

实际调参中 `track_thresh` 被设为 0.3（比默认 0.4 更低），意味着更多检测被当作"高分"进入第一轮匹配。

## 关键参数调优指南

| 参数 | 作用 | 增大影响 | 减小影响 |
|------|------|----------|----------|
| `track_thresh` | 高分/低分检测分界 | 更多检测进高分匹配 | 更多检测进低分挽救 |
| `high_thresh` | 新建轨迹最低分 | 更少新轨迹(更可靠) | 更多新轨迹(可能更多误检) |
| `match_thresh` | IoU 匹配阈值 | 更宽松的匹配 | 更严格的匹配 |
| `track_buffer` | 轨迹丢失存活时间 | 更久保留丢失轨迹 | 更快删除丢失轨迹 |
