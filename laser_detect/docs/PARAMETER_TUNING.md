# 卡尔曼滤波器参数调试指南

## 快速诊断表

| 症状 | 原因 | 解决方案 |
|------|------|--------|
| ❌ 角度不停抖动 ±2-3° | R值太小（过信任新测量） | **↑ R值** (e.g., 4→6) |
| ❌ 滤波器反应太慢，跟不上 | Q值太小（过信任历史值） | **↑ Q值** (e.g., 0.01→0.05) |
| ❌ 大目标检测后变小，角度突变 | 异常值阈值太小 | **↑ outlier_threshold** (e.g., 30→40) |
| ❌ 频繁过滤有效目标 | min_box_size太大 | **↓ min_box_size** (e.g., 20→15) |
| ⚠️ 目标尺寸 <20px 仍要追踪 | 默认过滤阈值 | **↓ min_box_size** |

---

## 详细调试流程

### 第一步：确定当前问题

运行程序，记录以下信息：

```cpp
// 在INTEGRATION_EXAMPLE.cpp的timerCallback中添加调试输出
RCLCPP_INFO(this->get_logger(),
    "[DEBUG] Box: %.0f×%.0f, Raw angles: yaw=%.1f° pitch=%.1f°, "
    "Filtered: yaw=%.1f° pitch=%.1f°",
    detection.box.width, detection.box.height,
    raw_yaw, raw_pitch,  // 计算滤波前的角度
    yaw, pitch           // 滤波后的角度
);
```

**记录表格：**
| Frame | Box Size | Raw Yaw | Filtered Yaw | Delta |
|-------|----------|---------|--------------|-------|
| 1     | 25×25    | 5.2°    | 5.0°        | 0.2°  |
| 2     | 24×26    | 6.1°    | 5.4°        | 0.7°  |
| 3     | 3×4      | -8.5°   | 5.5°        | 14.0° |

---

### 第二步：选择相应的调试策略

#### **问题1：高频抖动（±1-3°）**

**症状特征：**
```
Frame 1: yaw = 5.2°
Frame 2: yaw = 5.8°
Frame 3: yaw = 5.1°
Frame 4: yaw = 5.9°
...（周期性波动）
```

**根本原因：**
- 小目标检测框位置波动
- 相机或目标轻微抖动
- 神经网络输出的自然波动

**调整步骤：**

```cpp
// 1️⃣ 先尝试增大R值（降低对新测量的信任）
angle_filter.setKalmanParams(
    0.01f,    // Q (不变)
    6.0f,     // R (原来4.0 → 6.0)
    0.01f,    // Q_pitch
    6.0f      // R_pitch
);
```

**效果监测：**
- ✅ 抖动减小？ → 保持这个参数
- ❌ 抖动不变？ → 继续 ↑ R值到8.0
- ❌ 滤波器响应变慢？ → 不是R值问题，看问题2

---

#### **问题2：响应延迟（跟不上目标快速移动）**

**症状特征：**
```
目标从 yaw=0° 快速移动到 30°（大约100ms内）
但滤波器输出：
Frame 1: yaw = 0°
Frame 2: yaw = 5° (延迟)
Frame 3: yaw = 12°
Frame 4: yaw = 18°
Frame 5: yaw = 25° (最终达到)
```

**根本原因：**
- Q值太小（模型过度平滑）
- R值太大（对新测量信任不足）

**调整步骤：**

```cpp
// 2️⃣ 增大Q值（增加对快速变化的响应）
angle_filter.setKalmanParams(
    0.03f,    // Q (原来0.01 → 0.03)
    4.0f,     // R (不变)
    0.03f,
    4.0f
);
```

**如果还是有延迟，再调整：**

```cpp
// 3️⃣ 同时增大Q和减小R
angle_filter.setKalmanParams(
    0.05f,    // Q (继续增大)
    2.5f,     // R (从4.0 → 2.5)
    0.05f,
    2.5f
);
```

**监测指标：**
```
目标移动延迟时间 = 目标实际移动到位置 - 滤波器输出到位置的时间
目标：< 50ms
```

---

#### **问题3：大尺寸目标突然消失（大跳变被过滤）**

**症状特征：**
```
Frame N:   yaw = 5.2°, box = 40×40 (检测成功)
Frame N+1: yaw = 45.3°, box = 39×41 (同一目标，相机转了40°)
Frame N+2: [异常值被过滤，输出上一帧的值] yaw = 5.2°
```

**根本原因：** `outlier_threshold` 太小（默认30°）

**调整步骤：**

```cpp
// 4️⃣ 增大异常值阈值
angle_filter.setOutlierThreshold(45.0f);  // 原来30° → 45°
```

**不同场景推荐值：**
| 场景 | 值 | 说明 |
|------|-----|------|
| 静止目标 | 15-20° | 严格过滤 |
| **缓慢移动** | **30-35°** | 推荐（默认） |
| 快速移动 | 40-50° | 宽松过滤 |
| 电机大幅转动 | 60-90° | 接受所有跳变 |

---

#### **问题4：小目标被错误过滤**

**症状：**
```
Box size: 15×18 (< 20×20，被过滤)
但这是一个真实的激光目标
```

**调整步骤：**

```cpp
// 5️⃣ 降低最小检测框大小
angle_filter.setMinBoxSize(12);  // 原来20 → 12
```

**推荐值：**
| 目标 | min_box_size | 说明 |
|------|---|------|
| 20m外激光点 | 12-15 | 推荐 |
| 中等距离球 | 20-25 | 标准 |
| 近距离目标 | 30+ | 严格过滤小框 |

---

## 标准参数配置集

### 配置A：20m小目标（激光）- 推荐

```cpp
class laser_inference_node : public rclcpp::Node {
    laser_inference_node() : 
        angle_filter(15, 30.0f)  // min_box=15, outlier=30°
    {
        // 在timerCallback开始处设置
        angle_filter.setKalmanParams(
            0.01f,   // Q_yaw
            4.0f,    // R_yaw
            0.01f,   // Q_pitch
            4.0f     // R_pitch
        );
    }
};
```

**特点：**
- ✅ 强平滑，角度标准差 ±0.4°
- ✅ 响应延迟 ~20ms
- ✅ 可靠过滤虚假检测

**监测指标：**
```
目标尺寸范围：12-50 px
角度平滑度：σ < 0.5°
CPU占用：< 2%
```

---

### 配置B：快速移动目标 - 优先响应

```cpp
angle_filter.setMinBoxSize(20);
angle_filter.setOutlierThreshold(40.0f);
angle_filter.setKalmanParams(
    0.05f,   // 更快响应
    2.5f,
    0.05f,
    2.5f
);
```

**特点：**
- ✅ 低延迟 ~10ms
- ⚠️ 抖动略大 ±1.2°
- ✅ 快速跟踪移动

**应用场景：** 目标快速扫过视野

---

### 配置C：极度稳定 - 最强平滑

```cpp
angle_filter.setMinBoxSize(25);
angle_filter.setOutlierThreshold(20.0f);
angle_filter.setKalmanParams(
    0.005f,  // 高度信任历史值
    8.0f,    // 严格过滤新测量
    0.005f,
    8.0f
);
```

**特点：**
- ✅ 极强平滑 σ < 0.2°
- ⚠️ 响应延迟 ~40ms
- ⚠️ 可能无法快速跟踪

**应用场景：** 电机精密控制，目标较稳定

---

## 实时调试工具

### 方法1：命令行参数调试

```cpp
// 在timerCallback中添加
void debugParameterTuning() {
    std::cout << "\n=== 卡尔曼滤波器实时调试 ===" << std::endl;
    std::cout << "1. 调整Q_yaw (当前: ...)" << std::endl;
    std::cout << "2. 调整R_yaw (当前: ...)" << std::endl;
    std::cout << "3. 调整异常值阈值" << std::endl;
    std::cout << "4. 调整最小框大小" << std::endl;
    std::cout << "5. 保存当前参数" << std::endl;
    std::cout << "6. 加载预设配置" << std::endl;
    
    int choice;
    std::cin >> choice;
    
    switch (choice) {
        case 1: {
            float q;
            std::cout << "输入新的Q_yaw值: ";
            std::cin >> q;
            angle_filter.setKalmanParams(q, 4.0f, q, 4.0f);
            std::cout << "✓ Q值已更新为: " << q << std::endl;
            break;
        }
        // 其他情况...
    }
}
```

### 方法2：配置文件

创建 `filter_config.yaml`：

```yaml
kalman_filter:
  min_box_size: 15
  outlier_threshold: 30.0
  
  yaw:
    process_variance: 0.01
    measurement_variance: 4.0
  
  pitch:
    process_variance: 0.01
    measurement_variance: 4.0

# 预设配置
presets:
  small_target:
    min_box_size: 12
    outlier_threshold: 35.0
    q: 0.01
    r: 4.0
  
  fast_moving:
    min_box_size: 20
    outlier_threshold: 40.0
    q: 0.05
    r: 2.5
  
  ultra_stable:
    min_box_size: 25
    outlier_threshold: 20.0
    q: 0.005
    r: 8.0
```

---

## 性能评估

### 收集指标数据

```cpp
struct FilterMetrics {
    int frame_count = 0;
    float angle_variance_yaw = 0.0f;
    float angle_variance_pitch = 0.0f;
    float max_jump_yaw = 0.0f;
    float max_jump_pitch = 0.0f;
    int detection_count = 0;
    int filtered_count = 0;
    
    void update(float raw_yaw, float filtered_yaw,
                float raw_pitch, float filtered_pitch,
                bool valid) {
        frame_count++;
        
        if (valid) {
            detection_count++;
            // 计算方差、最大跳变等
            // ...
        } else {
            filtered_count++;
        }
    }
    
    void printReport() {
        std::cout << "\n=== 性能报告 ===" << std::endl;
        std::cout << "总帧数: " << frame_count << std::endl;
        std::cout << "有效检测: " << detection_count << std::endl;
        std::cout << "被过滤: " << filtered_count << std::endl;
        std::cout << "Yaw方差: σ = " << std::sqrt(angle_variance_yaw) << "°" << std::endl;
        std::cout << "Pitch方差: σ = " << std::sqrt(angle_variance_pitch) << "°" << std::endl;
        std::cout << "Yaw最大跳变: " << max_jump_yaw << "°" << std::endl;
        std::cout << "Pitch最大跳变: " << max_jump_pitch << "°" << std::endl;
    }
};
```

### 目标指标

| 指标 | 目标值 | 优秀值 |
|------|--------|--------|
| **角度标准差** | < 1.0° | < 0.5° |
| **最大单帧跳变** | < 5.0° | < 2.0° |
| **滤波延迟** | < 50ms | < 25ms |
| **误检过滤率** | > 60% | > 80% |

---

## 常见错误

### ❌ 错误1：同时增大Q和R

```cpp
// 这样做是错误的！
angle_filter.setKalmanParams(0.1f, 10.0f, ...);
// 结果：完全无效，滤波器不工作
```

**正确做法：**
- 要更快响应：↑ Q，↓ R
- 要更平滑：↓ Q，↑ R

---

### ❌ 错误2：过度调参

```cpp
// 第一次调试，立即改成：
angle_filter.setKalmanParams(0.5f, 0.1f, ...);  // 太极端！
```

**正确做法：**
```cpp
// 每次改变 ±50% 幅度
// 原来0.01 → 0.015（+50%）或 0.005（-50%）
```

---

### ❌ 错误3：参数相同但结果不同

**原因：** 没有重置滤波器

```cpp
angle_filter.reset();  // 每次改参数前都要做
angle_filter.setKalmanParams(...);
```

---

## 总结

```
┌─ 症状诊断 ─┐
│ 1. 高频抖动 → ↑ R值 (4→6)
│ 2. 响应延迟 → ↑ Q值 (0.01→0.05)
│ 3. 大跳变被过滤 → ↑ outlier_threshold (30→45)
│ 4. 小目标被过滤 → ↓ min_box_size (20→15)
└────────────┘
```

**快速开始：** 使用配置A，运行30秒观察结果，然后根据症状调整。

---

**版本：** 1.0  
**最后更新：** 2026-04-22
