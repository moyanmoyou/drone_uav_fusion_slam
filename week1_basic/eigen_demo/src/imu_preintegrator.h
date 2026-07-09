#ifndef IMU_PREINTEGRATOR_H
#define IMU_PREINTEGRATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "geometry_math.h"

// ====================== 核心数据结构定义 ======================

// 单帧IMU观测数据
struct ImuData {
    double dt;               // 两帧时间间隔，单位：秒
    Eigen::Vector3d gyro;    // 陀螺仪角速度测量值，单位：rad/s
    Eigen::Vector3d acc;     // 加速度计测量值，单位：m/s²
};

// IMU噪声参数（用于协方差递推）
struct ImuNoiseParam {
    double gyro_noise = 0.01;       // 陀螺仪白噪声标准差
    double acc_noise = 0.1;         // 加速度计白噪声标准差
    double gyro_random_walk = 0.001; // 陀螺仪随机游走
    double acc_random_walk = 0.01;   // 加速度计随机游走
};

// 预积分累积结果：增量旋转、增量速度、增量位移
struct PreintegrationState {
    Eigen::Matrix3d dR;   // 两帧间增量旋转矩阵
    Eigen::Vector3d dv;   // 两帧间增量速度
    Eigen::Vector3d dp;   // 两帧间增量位移
    double sum_dt;        // 累计积分总时长
};

// ====================== 预积分核心类 ======================
class ImuPreintegrator {
public:
    // 构造函数：传入噪声参数，自动初始化状态
    ImuPreintegrator(const ImuNoiseParam& noise_param);

    // 重置所有预积分状态和协方差，重新开始积分
    void reset();

    // 核心功能：输入单帧IMU数据，执行一步离散积分（中值积分法，对标VINS-Mono）
    void pushImuData(const ImuData& data);

    // 批量积分：传入一整段IMU数组，自动循环累积
    void batchIntegrate(const std::vector<ImuData>& imu_list);

    // 获取预积分结果
    PreintegrationState getState() const { return state_; }

    // 获取15维误差协方差矩阵
    Eigen::MatrixXd getCovariance() const { return cov_; }

private:
    PreintegrationState state_;  // 预积分累积状态量
    Eigen::MatrixXd cov_;        // 15×15误差协方差矩阵
    ImuNoiseParam noise_;        // IMU噪声配置参数
    ImuData last_imu_;           // 保存上一帧IMU数据，用于中值积分
    bool is_first_imu_ = true;   // 标记是否为第一帧IMU数据

    // 私有函数：单步递推误差协方差矩阵
    void propagateCovariance(const ImuData& data);
};

#endif // IMU_PREINTEGRATOR_H