# 自定义IMU消息
std_msgs/Header header   # 标准消息头：时间戳、坐标系
float64[3] angular_velocity  # 三轴角速度 rad/s
float64[3] linear_acceleration # 三轴加速度 m/s²
float64[4] orientation   # 四元数姿态 x y z w