#ifndef GEOMETRY_MATH_H
#define GEOMETRY_MATH_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

// 姿态几何工具库：全部基于Eigen实现，对标视觉SLAM十四讲第3、4章
class GeometryMath {
public:
    // ---------------------- 1. 欧拉角 <-> 旋转矩阵（ZYX顺序，roll/pitch/yaw） ----------------------
    // 输入：欧拉角 [roll, pitch, yaw] 单位：弧度
    // 输出：3x3旋转矩阵
    static Eigen::Matrix3d euler2Rot(const Eigen::Vector3d& euler);

    // 输入：3x3旋转矩阵
    // 输出：欧拉角 [roll, pitch, yaw] 单位：弧度
    static Eigen::Vector3d rot2Euler(const Eigen::Matrix3d& R);

    // ---------------------- 2. 旋转矩阵 <-> 四元数 ----------------------
    static Eigen::Quaterniond rot2Quat(const Eigen::Matrix3d& R);
    static Eigen::Matrix3d quat2Rot(const Eigen::Quaterniond& q);

    // ---------------------- 3. 旋转向量 <-> 旋转矩阵（罗德里格斯公式） ----------------------
    static Eigen::Matrix3d rotVec2Rot(const Eigen::Vector3d& rot_vec);
    static Eigen::Vector3d rot2RotVec(const Eigen::Matrix3d& R);

    // ---------------------- 4. SO(3) BCH近似公式 ----------------------
    // 一阶近似：左雅可比矩阵
    static Eigen::Matrix3d bchJacobianLeft(const Eigen::Vector3d& phi);
    // 一阶近似：右雅可比矩阵
    static Eigen::Matrix3d bchJacobianRight(const Eigen::Vector3d& phi);

    // ---------------------- 5. 旋转扰动更新 ----------------------
    // 左扰动更新：R_new = exp(delta_phi) * R_old
    static Eigen::Matrix3d updateRotLeft(const Eigen::Matrix3d& R, const Eigen::Vector3d& delta);
    // 右扰动更新：R_new = R_old * exp(delta_phi)
    static Eigen::Matrix3d updateRotRight(const Eigen::Matrix3d& R, const Eigen::Vector3d& delta);

    // 辅助工具：角度归一化，限制在[-pi, pi]
    static double normalizeAngle(double angle);
};

#endif