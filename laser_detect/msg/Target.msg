# MotorControl.msg
# 用于控制云台电机的消息定义

# 电机控制模式
# 0: 绝对角度控制 (直接设置到指定角度)
# 1: 相对角度控制 (在当前角度基础上增加指定角度)
# 2: 速度控制 (设置电机旋转速度)
uint8 MODE_ABSOLUTE = 0
uint8 MODE_RELATIVE = 1
uint8 MODE_VELOCITY = 2
uint8 mode

# Pitch轴角度 (度)
# 范围通常为 -45 到 45 度
float32 pitch

# Yaw轴角度 (度)
# 范围通常为 -90 到 90 度
float32 yaw

# 电机速度 (仅在速度模式下有效)
float32 velocity_pitch
float32 velocity_yaw

# 时间戳
builtin_interfaces/Time stamp
