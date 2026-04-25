# 🚀 卡尔曼滤波集成检查清单

按照以下步骤逐项完成，确保成功集成。

---

## ✅ 第一阶段：准备工作

- [ ] **读文档（5分钟）**
  - 阅读 `docs/README.md` 了解整体结构
  - 快速扫一遍 `docs/KALMAN_FILTER_GUIDE.md` 的概述部分

- [ ] **验证文件**
  - 确认 `include/kalman_filter.hpp` 存在
  - 确认 `src/kalman_filter.cpp` 存在
  - 确认 `docs/` 目录下有 5 个 markdown 文件

---

## ✅ 第二阶段：编译集成

- [ ] **修改 CMakeLists.txt**
  - [ ] 打开 `CMakeLists.txt`
  - [ ] 找到 `add_executable(inference_node src/main.cpp src/laser_detect.cpp)`
  - [ ] 改为：
    ```cmake
    add_executable(inference_node 
        src/main.cpp 
        src/laser_detect.cpp 
        src/kalman_filter.cpp
    )
    ```
  - [ ] 确保 `target_link_libraries` 包含 `m`（数学库）

- [ ] **编译验证**
  ```bash
  cd /e/fjut-rm2026-radar
  colcon build --packages-select laser_detect
  ```
  - [ ] 编译成功（无错误）
  - [ ] 能看到 `Built target inference_node`

---

## ✅ 第三阶段：代码集成

- [ ] **添加头文件**
  - [ ] 打开 `src/main.cpp`
  - [ ] 在 include 部分添加：
    ```cpp
    #include "kalman_filter.hpp"
    ```
  - [ ] 确认能找到这一行：
    ```cpp
    #include "laser_detect.hpp"
    ```

- [ ] **添加成员变量**
  - [ ] 找到 `laser_inference_node` 类定义
  - [ ] 在 private 部分添加：
    ```cpp
    TargetAngleFilter angle_filter;
    ```

- [ ] **初始化滤波器**
  - [ ] 在构造函数初始化列表中添加：
    ```cpp
    laser_inference_node() : Node("laser_inference_node"),
                             runOnGPU_(true),
                             angle_filter(20, 30.0f)
    ```

- [ ] **重置滤波器**
  - [ ] 在 `timerCallback()` 或 `processHikCameraMode()` 开始处添加：
    ```cpp
    angle_filter.reset();
    ```

- [ ] **替换 pitch/yaw 计算代码**
  - [ ] 找到原来的计算代码（约第178-194行）
  - [ ] 替换为：
    ```cpp
    float yaw = 0.0f, pitch = 0.0f;
    bool valid = angle_filter.processDetection(
        detection.box.x, detection.box.y,
        detection.box.width, detection.box.height,
        camera_focal_length_x, camera_focal_length_y,
        image_center_x, image_center_y,
        yaw, pitch
    );
    
    if (!valid) {
        RCLCPP_WARN(this->get_logger(), 
                    "Detection filtered out");
        continue;  // 跳过此检测
    }
    ```

---

## ✅ 第四阶段：编译和测试

- [ ] **重新编译**
  ```bash
  cd /e/fjut-rm2026-radar
  colcon build --packages-select laser_detect
  ```
  - [ ] 编译无错误

- [ ] **运行程序**
  ```bash
  ros2 run laser_detect inference_node
  ```
  - [ ] 程序启动无崩溃
  - [ ] 能看到初始化信息

- [ ] **基本功能测试**
  - [ ] 选择 test 模式
  - [ ] 加载视频文件
  - [ ] 观察输出是否包含 yaw/pitch 角度
  - [ ] 检查是否有"Detection filtered"的警告（正常现象）

---

## ✅ 第五阶段：性能验证（可选但推荐）

- [ ] **角度平滑性**
  - [ ] 运行 30 秒录制
  - [ ] 观察控制台输出的 yaw/pitch 值
  - [ ] 相同静止目标的角度变化应该 < ±1°
  - ✅ **目标：< ±0.5°**

- [ ] **响应延迟**
  - [ ] 水平缓慢转动相机
  - [ ] 观察角度输出的延迟（应该很小）
  - ✅ **目标：< 50ms**

- [ ] **误检过滤**
  - [ ] 观察过滤掉的检测框数量
  - [ ] 应该过滤掉尺寸 < 20px 的目标
  - ✅ **目标：误检过滤率 > 60%**

---

## ✅ 第六阶段：参数调优（如需要）

- [ ] **评估当前性能**
  - [ ] 运行 1 分钟录制
  - [ ] 记录角度标准差（参考 PARAMETER_TUNING.md）

- [ ] **根据症状调整参数**
  - [ ] 如果有高频抖动 → 参考 PARAMETER_TUNING.md 中的"问题1"
  - [ ] 如果响应太慢 → 参考"问题2"
  - [ ] 如果大目标被过滤 → 参考"问题3"

- [ ] **验证调整效果**
  - [ ] 再次运行 1 分钟录制
  - [ ] 比较参数调整前后的性能指标

---

## 🐛 故障排查

### 编译问题

| 错误信息 | 原因 | 解决 |
|---------|------|------|
| `kalman_filter.cpp: No such file` | 文件不存在 | 检查路径，确认 `src/kalman_filter.cpp` 存在 |
| `undefined reference to 'atan2'` | 没链接 m 库 | 在 CMakeLists.txt 的 target_link_libraries 中添加 `m` |
| `fatal error: kalman_filter.hpp` | 找不到头文件 | 检查 include_directories 包含了 include/ 目录 |

**调试步骤：**
```bash
# 清空编译目录重新编译
rm -rf build/
colcon build --packages-select laser_detect --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

### 运行时问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 程序崩溃 | 内存访问错误 | 确保 angle_filter 已初始化 |
| 没有角度输出 | 没调用 processDetection | 检查代码是否替换完整 |
| 所有检测都被过滤 | min_box_size 太大 | 检查是否设置了合理的值 |

---

## 📋 最终验收标准

程序应该满足以下条件：

- ✅ 编译无错误、无警告（关于kalman_filter的）
- ✅ 运行无崩溃
- ✅ 有效检测输出 yaw/pitch 角度值
- ✅ 角度抖动 < ±1°（理想 < ±0.5°）
- ✅ 能够过滤掉太小的检测框
- ✅ 响应延迟 < 50ms（理想 < 25ms）

---

## 📝 完成标记

| 项目 | 状态 | 备注 |
|------|------|------|
| 文件创建 | ✅ | 所有源文件和文档已创建 |
| CMakeLists.txt 修改 | ⬜ | |
| main.cpp 修改 | ⬜ | |
| 编译成功 | ⬜ | |
| 基本功能测试 | ⬜ | |
| 性能验证 | ⬜ | |
| 参数优化 | ⬜ | |

**总体进度：** 1 / 7 ✅

---

## 🆘 需要帮助？

### 快速参考
- 📖 **代码示例**：见 `docs/INTEGRATION_EXAMPLE.cpp`
- 🔧 **编译步骤**：见 `docs/CMAKE_INTEGRATION.md`
- ⚙️ **参数调整**：见 `docs/PARAMETER_TUNING.md`

### 常见问题
- ❓ 应该用哪个类？→ 使用 `TargetAngleFilter`
- ❓ 参数怎么设？→ 用默认值，根据症状调整
- ❓ 怎么验证效果？→ 运行30秒，观察角度稳定性

---

**预计完成时间：** 30-60 分钟（根据经验水平）

**开始日期：** _____________  
**完成日期：** _____________  
**调整者：** _____________

---

**祝集成顺利！** 🎉
