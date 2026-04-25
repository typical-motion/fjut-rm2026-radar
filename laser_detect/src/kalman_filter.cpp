#include "kalman_filter.hpp"
#include <algorithm>

// ============================================================================
// KalmanFilter1D Implementation
// ============================================================================

KalmanFilter1D::KalmanFilter1D(float process_variance,
                               float measurement_variance,
                               float initial_value,
                               float initial_estimate_error)
    : Q(process_variance),
      R(measurement_variance),
      x_estimate(initial_value),
      P_estimate(initial_estimate_error),
      K_gain(0.0f)
{
}

float KalmanFilter1D::update(float measurement)
{
    // 预测步骤 (Prediction)
    // 假设过程模型为：x(k) = x(k-1)，即目标状态不变
    // P(k|k-1) = P(k-1|k-1) + Q
    P_estimate = P_estimate + Q;

    // 更新步骤 (Update)
    // 计算卡尔曼增益：K = P(k|k-1) / (P(k|k-1) + R)
    K_gain = P_estimate / (P_estimate + R);

    // 状态估计：x(k|k) = x(k|k-1) + K * (z(k) - x(k|k-1))
    x_estimate = x_estimate + K_gain * (measurement - x_estimate);

    // 更新估计误差：P(k|k) = (1 - K) * P(k|k-1)
    P_estimate = (1.0f - K_gain) * P_estimate;

    return x_estimate;
}

void KalmanFilter1D::reset(float initial_value)
{
    x_estimate = initial_value;
    P_estimate = 1.0f;
    K_gain = 0.0f;
}


// ============================================================================
// KalmanFilter2D Implementation
// ============================================================================

KalmanFilter2D::KalmanFilter2D(float process_variance_yaw,
                               float measurement_variance_yaw,
                               float process_variance_pitch,
                               float measurement_variance_pitch)
    : filter_yaw(process_variance_yaw, measurement_variance_yaw, 0.0f, 1.0f),
      filter_pitch(process_variance_pitch, measurement_variance_pitch, 0.0f, 1.0f)
{
}

void KalmanFilter2D::update(float yaw_measurement, float pitch_measurement,
                            float& yaw_out, float& pitch_out)
{
    yaw_out = filter_yaw.update(yaw_measurement);
    pitch_out = filter_pitch.update(pitch_measurement);
}

void KalmanFilter2D::reset(float yaw_init, float pitch_init)
{
    filter_yaw.reset(yaw_init);
    filter_pitch.reset(pitch_init);
}

void KalmanFilter2D::setYawParams(float q, float r)
{
    filter_yaw.setProcessVariance(q);
    filter_yaw.setMeasurementVariance(r);
}

void KalmanFilter2D::setPitchParams(float q, float r)
{
    filter_pitch.setProcessVariance(q);
    filter_pitch.setMeasurementVariance(r);
}


// ============================================================================
// TargetAngleFilter Implementation
// ============================================================================

TargetAngleFilter::TargetAngleFilter(int min_box_size, float outlier_threshold)
    : min_box_size(min_box_size),
      outlier_threshold(outlier_threshold),
      filter(0.01f, 4.0f, 0.01f, 4.0f),
      last_detection_valid(false),
      last_yaw(0.0f),
      last_pitch(0.0f),
      invalid_count(0)
{
}

bool TargetAngleFilter::isBoxTooSmall(float width, float height)
{
    return (width < min_box_size || height < min_box_size);
}

bool TargetAngleFilter::isOutlier(float value, float last_value)
{
    float delta = std::abs(value - last_value);
    return delta > outlier_threshold;
}

bool TargetAngleFilter::processDetection(float detection_box_x,
                                          float detection_box_y,
                                          float detection_box_width,
                                          float detection_box_height,
                                          float camera_focal_x,
                                          float camera_focal_y,
                                          float image_center_x,
                                          float image_center_y,
                                          float& yaw_out, float& pitch_out)
{
    // 检查检测框大小
    if (isBoxTooSmall(detection_box_width, detection_box_height))
    {
        invalid_count++;
        last_detection_valid = false;

        // 保持上一次的有效估计（若存在）
        if (invalid_count < MAX_INVALID_FRAMES && last_detection_valid)
        {
            yaw_out = last_yaw;
            pitch_out = last_pitch;
            return false;
        }
        return false;
    }

    // 计算检测框中心
    float target_u = detection_box_x + detection_box_width / 2.0f;
    float target_v = detection_box_y + detection_box_height / 2.0f;

    // 计算相对于图像中心的偏移
    float delta_u = target_u - image_center_x;
    float delta_v = target_v - image_center_y;

    // 计算角度（弧度）
    float yaw_rad = std::atan2(delta_u, camera_focal_x);
    float pitch_rad = std::atan2(delta_v, camera_focal_y);

    // 转换为角度（度）
    float yaw_meas = yaw_rad * (180.0f / M_PI);
    float pitch_meas = pitch_rad * (180.0f / M_PI);

    // 检查异常值（大的突变）
    if (last_detection_valid)
    {
        if (isOutlier(yaw_meas, last_yaw) || isOutlier(pitch_meas, last_pitch))
        {
            invalid_count++;
            // 异常值被忽略，返回上一次的有效估计
            if (invalid_count < MAX_INVALID_FRAMES)
            {
                yaw_out = last_yaw;
                pitch_out = last_pitch;
                return false;
            }
            // 连续多次异常，强制接受新值
        }
    }

    // 通过卡尔曼滤波器
    filter.update(yaw_meas, pitch_meas, yaw_out, pitch_out);

    // 限制角度范围
    yaw_out = std::max(-90.0f, std::min(90.0f, yaw_out));
    pitch_out = std::max(-45.0f, std::min(45.0f, pitch_out));

    // 更新状态
    last_yaw = yaw_out;
    last_pitch = pitch_out;
    last_detection_valid = true;
    invalid_count = 0;

    return true;
}

void TargetAngleFilter::reset()
{
    filter.reset(0.0f, 0.0f);
    last_detection_valid = false;
    last_yaw = 0.0f;
    last_pitch = 0.0f;
    invalid_count = 0;
}

void TargetAngleFilter::setKalmanParams(float q_yaw, float r_yaw,
                                        float q_pitch, float r_pitch)
{
    filter.setYawParams(q_yaw, r_yaw);
    filter.setPitchParams(q_pitch, r_pitch);
}
