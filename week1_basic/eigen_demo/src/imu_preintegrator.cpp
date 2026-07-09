#include "imu_preintegrator.h"

// ====================== 构造函数：初始化状态和协方差 ======================
ImuPreintegrator::ImuPreintegrator(const ImuNoiseParam& noise_param) : noise_(noise_param) {
    reset();
}

// ====================== 重置函数：清空所有状态，重新开始积分 ======================
void ImuPreintegrator::reset() {
    // 预积分状态初始化
    state_.dR = Eigen::Matrix3d::Identity(); // 初始旋转为单位矩阵
    state_.dv.setZero();                     // 初始速度增量为0
    state_.dp.setZero();                     // 初始位移增量为0
    state_.sum_dt = 0.0;                     // 累计时间为0

    // 15维误差协方差初始化为0（初始状态无误差）
    // 15维顺序：旋转误差(3) + 速度误差(3) + 位置误差(3) + 陀螺零偏误差(3) + 加计零偏误差(3)
    cov_ = Eigen::MatrixXd::Zero(15, 15);

    // 重置帧标记
    is_first_imu_ = true;
}

// ====================== 核心：单帧IMU中值积分 ======================
// 对标VINS-Mono中值积分法，用前后两帧IMU的平均值计算，精度高于欧拉积分
void ImuPreintegrator::pushImuData(const ImuData& data) {
    // 第一帧只保存，不积分
    if (is_first_imu_) {
        last_imu_ = data;
        is_first_imu_ = false;
        return;
    }

    double dt = data.dt;
    // 中值：前后两帧角速度、加速度取平均
    Eigen::Vector3d gyro_avg = 0.5 * (last_imu_.gyro + data.gyro);
    Eigen::Vector3d acc_avg = 0.5 * (last_imu_.acc + data.acc);

    // ---------------------- 1. 旋转增量更新 ----------------------
    // 旋转向量 = 平均角速度 * 时间间隔
    Eigen::Vector3d delta_rot_vec = gyro_avg * dt;
    // 旋转向量转旋转矩阵，右乘到当前增量旋转上
    state_.dR = state_.dR * GeometryMath::rotVec2Rot(delta_rot_vec);

    // ---------------------- 2. 速度增量更新 ----------------------
    // 把body系下的平均加速度转到世界系（用当前dR），乘以时间
    Eigen::Vector3d acc_world = state_.dR * acc_avg;
    state_.dv += acc_world * dt;

    // ---------------------- 3. 位置增量更新 ----------------------
    // 匀加速公式：dp += dv*dt + 0.5*a*dt²
    state_.dp += state_.dv * dt + 0.5 * acc_world * dt * dt;

    // ---------------------- 4. 累计时间 ----------------------
    state_.sum_dt += dt;

    // ---------------------- 5. 递推误差协方差 ----------------------
    propagateCovariance(data);

    // 保存当前帧，作为下一帧的上一帧
    last_imu_ = data;
}

// ====================== 批量积分：循环处理多帧IMU数据 ======================
void ImuPreintegrator::batchIntegrate(const std::vector<ImuData>& imu_list) {
    for (const auto& imu : imu_list) {
        pushImuData(imu);
    }
}

// ====================== 私有：协方差矩阵递推 ======================
// 误差状态传播方程：P_k+1 = F * P_k * F^T + G * V * G^T
// F：状态转移矩阵；G：噪声输入矩阵；V：噪声对角阵
void ImuPreintegrator::propagateCovariance(const ImuData& data) {
    double dt = data.dt;
    Eigen::Vector3d acc_avg = 0.5 * (last_imu_.acc + data.acc);

    // 构造15x15状态转移矩阵F，初始化为单位矩阵
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(15, 15);

    // 反对称矩阵：加速度的反对称（用于旋转误差对速度/位置的影响）
    Eigen::Matrix3d acc_hat;
    acc_hat << 0, -acc_avg(2), acc_avg(1),
               acc_avg(2), 0, -acc_avg(0),
               -acc_avg(1), acc_avg(0), 0;

    // ---------- 填充F矩阵（对应误差状态递推关系） ----------
    // 旋转误差块：dtheta_new = dtheta - gyro_avg^ * dt
    Eigen::Vector3d gyro_avg = 0.5 * (last_imu_.gyro + data.gyro);
    Eigen::Matrix3d gyro_hat;
    gyro_hat << 0, -gyro_avg(2), gyro_avg(1),
                gyro_avg(2), 0, -gyro_avg(0),
                -gyro_avg(1), gyro_avg(0), 0;
    F.block<3,3>(0,0) = Eigen::Matrix3d::Identity() - gyro_hat * dt;
    F.block<3,3>(0,9) = -Eigen::Matrix3d::Identity() * dt; // 陀螺零偏影响旋转

    // 速度误差块：dv_new = dv - R*a^ * dtheta * dt - R * ba * dt
    F.block<3,3>(3,0) = -state_.dR * acc_hat * dt;
    F.block<3,3>(3,12) = -state_.dR * dt; // 加计零偏影响速度
    F.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

    // 位置误差块：dp_new = dp + dv*dt - 0.5*R*a^ * dtheta * dt²
    F.block<3,3>(6,0) = -0.5 * state_.dR * acc_hat * dt * dt;
    F.block<3,3>(6,3) = Eigen::Matrix3d::Identity() * dt;
    F.block<3,3>(6,12) = -0.5 * state_.dR * dt * dt; // 加计零偏影响位置
    F.block<3,3>(6,6) = Eigen::Matrix3d::Identity();

    // 零偏随机游走：零偏误差随时间累积
    F.block<3,3>(9,9) = Eigen::Matrix3d::Identity();
    F.block<3,3>(12,12) = Eigen::Matrix3d::Identity();

    // ---------- 构造噪声输入矩阵G（15x12） ----------
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(15, 12);
    G.block<3,3>(0,0) = -Eigen::Matrix3d::Identity() * dt;  // 陀螺白噪声
    G.block<3,3>(3,3) = -state_.dR * dt;                    // 加计白噪声
    G.block<3,3>(6,3) = -0.5 * state_.dR * dt * dt;         // 加计白噪声影响位置
    G.block<3,3>(9,6) = Eigen::Matrix3d::Identity() * dt;   // 陀螺随机游走
    G.block<3,3>(12,9) = Eigen::Matrix3d::Identity() * dt;  // 加计随机游走

    // ---------- 构造噪声对角阵V（12x12） ----------
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(12, 12);
    double ng2 = noise_.gyro_noise * noise_.gyro_noise;
    double na2 = noise_.acc_noise * noise_.acc_noise;
    double nbg2 = noise_.gyro_random_walk * noise_.gyro_random_walk;
    double nba2 = noise_.acc_random_walk * noise_.acc_random_walk;

    V.block<3,3>(0,0) = ng2 * Eigen::Matrix3d::Identity();
    V.block<3,3>(3,3) = na2 * Eigen::Matrix3d::Identity();
    V.block<3,3>(6,6) = nbg2 * Eigen::Matrix3d::Identity();
    V.block<3,3>(9,9) = nba2 * Eigen::Matrix3d::Identity();

    // ---------- 协方差更新公式 ----------
    cov_ = F * cov_ * F.transpose() + G * V * G.transpose();
}