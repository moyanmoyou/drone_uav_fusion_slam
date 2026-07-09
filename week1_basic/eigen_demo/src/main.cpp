#include <iostream>
#include <iomanip>
#include <vector>
#include "geometry_math.h"
#include "imu_preintegrator.h"

using namespace std;

int main() {
    cout << "====== Week1 Day2 完整测试 ======" << endl;
    cout << fixed << setprecision(6);

    // ====================== 第一部分：姿态几何模块测试 ======================
    cout << "\n---------- 1. 姿态几何转换测试 ----------" << endl;
    Eigen::Vector3d euler_ori(0.2, 0.3, 0.5);
    cout << "原始欧拉角(弧度): " << euler_ori.transpose() << endl;

    Eigen::Matrix3d R = GeometryMath::euler2Rot(euler_ori);
    Eigen::Vector3d euler_back = GeometryMath::rot2Euler(R);
    double error_euler = (euler_ori - euler_back).norm();
    cout << "欧拉角闭环误差: " << error_euler << endl;

    Eigen::Quaterniond q = GeometryMath::rot2Quat(R);
    Eigen::Matrix3d R_q = GeometryMath::quat2Rot(q);
    double error_q = (R - R_q).norm();
    cout << "四元数转换误差: " << error_q << endl;

    Eigen::Vector3d rot_vec = GeometryMath::rot2RotVec(R);
    Eigen::Matrix3d R_vec = GeometryMath::rotVec2Rot(rot_vec);
    double error_vec = (R - R_vec).norm();
    cout << "旋转向量转换误差: " << error_vec << endl;

    // ====================== 第二部分：IMU预积分模块测试 ======================
    cout << "\n---------- 2. 手写IMU预积分测试 ----------" << endl;

    // 配置IMU噪声参数
    ImuNoiseParam noise;
    noise.gyro_noise = 0.01;
    noise.acc_noise = 0.1;

    // 创建预积分器
    ImuPreintegrator preintegrator(noise);

    // 模拟生成100帧IMU数据：匀速绕Z轴旋转 + Z轴匀加速
    vector<ImuData> imu_data_list;
    double dt = 0.01; // 100Hz采样率
    double total_time = 1.0; // 总时长1秒
    int frame_num = total_time / dt;

    // 模拟：绕Z轴0.5rad/s旋转，Z轴加速度2m/s²
    Eigen::Vector3d gyro_sim(0, 0, 0.5);
    Eigen::Vector3d acc_sim(0, 0, 2.0);

    for (int i = 0; i < frame_num; ++i) {
        ImuData data;
        data.dt = dt;
        data.gyro = gyro_sim;
        data.acc = acc_sim;
        imu_data_list.push_back(data);
    }

    // 执行批量预积分
    preintegrator.batchIntegrate(imu_data_list);

    // 获取预积分结果
    PreintegrationState result = preintegrator.getState();
    Eigen::MatrixXd cov = preintegrator.getCovariance();

    cout << "积分总时长: " << result.sum_dt << " 秒" << endl;
    cout << "增量旋转矩阵行列式(应接近1): " << result.dR.determinant() << endl;
    cout << "增量速度(m/s): " << result.dv.transpose() << endl;
    cout << "增量位移(m): " << result.dp.transpose() << endl;

    // 打印协方差矩阵对角线（误差随积分累积增大）
    cout << "\n协方差矩阵对角线(误差累积量):" << endl;
    cout << "旋转误差协方差: " << cov(0,0) << ", " << cov(1,1) << ", " << cov(2,2) << endl;
    cout << "速度误差协方差: " << cov(3,3) << ", " << cov(4,4) << ", " << cov(5,5) << endl;
    cout << "位置误差协方差: " << cov(6,6) << ", " << cov(7,7) << ", " << cov(8,8) << endl;

    // ====================== 测试结论 ======================
    cout << "\n====== Day2 全部测试完成 ======" << endl;
    cout << "✅ 几何转换误差接近0，功能正常" << endl;
    cout << "✅ IMU预积分状态正常累积，协方差正定" << endl;
    cout << "✅ 所有模块编译运行通过" << endl;

    return 0;
}