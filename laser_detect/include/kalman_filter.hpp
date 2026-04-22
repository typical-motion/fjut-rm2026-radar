#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <cmath>
#include <iostream>

/**
 * @brief 一维卡尔曼滤波器 - 用于平滑pitch/yaw角度信号
 * 特别针对小目标长距离检测的角度抖动问题
 *
 * 应用场景：
 * - 20米处的小目标检测
 * - 单帧检测结果不稳定导致的角度波动
 * - 实时电机控制指令平滑
 */
class KalmanFilter1D
{
public:
    /**
     * @brief 初始化卡尔曼滤波器
     * @param process_variance Q - 过程噪声方差(值越大，模型越相信测量值)
     * @param measurement_variance R - 测量噪声方差(值越大，模型越相信历史值)
     * @param initial_value 初始估计值
     * @param initial_estimate_error 初始估计误差
     */
    KalmanFilter1D(float process_variance = 0.01f,
                   float measurement_variance = 4.0f,
                   float initial_value = 0.0f,
                   float initial_estimate_error = 1.0f);

    /**
     * @brief 输入新的测量值并返回滤波后的估计值
     * @param measurement 新的测量值（检测到的角度）
     * @return 滤波后的估计值
     */
    float update(float measurement);

    /**
     * @brief 重置滤波器状态
     * @param initial_value 新的初始值
     */
    void reset(float initial_value = 0.0f);

    /**
     * @brief 设置过程噪声方差
     * 值越大，滤波器对突变响应越快（但抖动越多）
     * 建议范围：0.001f - 0.1f
     */
    void setProcessVariance(float q) { Q = q; }

    /**
     * @brief 设置测量噪声方差
     * 值越大，滤波器对新测量值的信任度越低（平滑效果越强）
     * 建议范围：1.0f - 10.0f
     */
    void setMeasurementVariance(float r) { R = r; }

    /**
     * @brief 获取当前估计值
     */
    float getEstimate() const { return x_estimate; }

    /**
     * @brief 获取当前估计误差方差
     */
    float getEstimateError() const { return P_estimate; }

private:
    float Q;            // 过程噪声方差 (Process variance)
    float R;            // 测量噪声方差 (Measurement variance)
    float x_estimate;   // 状态估计值 (State estimate)
    float P_estimate;   // 估计误差方差 (Estimate error)
    float K_gain;       // 卡尔曼增益 (Kalman gain)
};


/**
 * @brief 二维卡尔曼滤波器 - 同时平滑yaw和pitch
 * 简化版本，假设yaw和pitch相互独立
 */
class KalmanFilter2D
{
public:
    /**
     * @brief 初始化二维卡尔曼滤波器
     * @param process_variance_yaw yaw轴的过程噪声方差
     * @param measurement_variance_yaw yaw轴的测量噪声方差
     * @param process_variance_pitch pitch轴的过程噪声方差
     * @param measurement_variance_pitch pitch轴的测量噪声方差
     */
    KalmanFilter2D(float process_variance_yaw = 0.01f,
                   float measurement_variance_yaw = 4.0f,
                   float process_variance_pitch = 0.01f,
                   float measurement_variance_pitch = 4.0f);

    /**
     * @brief 同时更新yaw和pitch
     * @param yaw_measurement 测量的yaw角度
     * @param pitch_measurement 测量的pitch角度
     * @param yaw_out 输出滤波后的yaw
     * @param pitch_out 输出滤波后的pitch
     */
    void update(float yaw_measurement, float pitch_measurement,
                float& yaw_out, float& pitch_out);

    /**
     * @brief 重置滤波器
     */
    void reset(float yaw_init = 0.0f, float pitch_init = 0.0f);

    /**
     * @brief 设置yaw和pitch的参数（用于实时调试）
     */
    void setYawParams(float q, float r);
    void setPitchParams(float q, float r);

private:
    KalmanFilter1D filter_yaw;
    KalmanFilter1D filter_pitch;
};


/**
 * @brief 专用于目标角度跟踪的高级滤波器
 * 包含检测框大小校验、异常值滤除等功能
 */
class TargetAngleFilter
{
public:
    /**
     * @brief 初始化目标角度滤波器
     * @param min_box_size 最小检测框尺寸（像素），小于此值的目标被过滤
     * @param outlier_threshold 异常值阈值（度），超过此值的跳变被视为异常
     */
    TargetAngleFilter(int min_box_size = 20, float outlier_threshold = 30.0f);

    /**
     * @brief 处理检测结果并返回滤波后的角度
     * @param detection_box_x 检测框左上角x
     * @param detection_box_y 检测框左上角y
     * @param detection_box_width 检测框宽度
     * @param detection_box_height 检测框高度
     * @param camera_focal_x 相机焦距x（像素）
     * @param camera_focal_y 相机焦距y（像素）
     * @param image_center_x 图像中心x
     * @param image_center_y 图像中心y
     * @param yaw_out 输出yaw角度
     * @param pitch_out 输出pitch角度
     * @return 检测是否有效（是否通过大小和异常值检查）
     */
    bool processDetection(float detection_box_x, float detection_box_y,
                          float detection_box_width, float detection_box_height,
                          float camera_focal_x, float camera_focal_y,
                          float image_center_x, float image_center_y,
                          float& yaw_out, float& pitch_out);

    /**
     * @brief 获取最后一次检测的有效性
     */
    bool isLastDetectionValid() const { return last_detection_valid; }

    /**
     * @brief 重置滤波器状态
     */
    void reset();

    /**
     * @brief 设置参数（用于实时调试）
     */
    void setMinBoxSize(int size) { min_box_size = size; }
    void setOutlierThreshold(float deg) { outlier_threshold = deg; }
    void setKalmanParams(float q_yaw, float r_yaw, float q_pitch, float r_pitch);

private:
    bool isOutlier(float value, float last_value);
    bool isBoxTooSmall(float width, float height);

    int min_box_size;
    float outlier_threshold;
    KalmanFilter2D filter;
    bool last_detection_valid;
    float last_yaw;
    float last_pitch;
    int invalid_count;
    static constexpr int MAX_INVALID_FRAMES = 5;
};

#endif // KALMAN_FILTER_HPP
