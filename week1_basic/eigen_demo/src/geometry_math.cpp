#include "geometry_math.h"

Eigen::Matrix3d GeometryMath::euler2Rot(const Eigen::Vector3d& euler){
    double roll = euler(0);
    double pitch = euler(1);
    double yaw = euler(2);

    Eigen::Matrix3d Rx, Ry, Rz;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);

    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;
    
    // ZYX内旋：R = Rz * Ry * Rx
    return Rz * Ry * Rx;
}

Eigen::Vector3d GeometryMath::rot2Euler(const Eigen::Matrix3d& R){
    double roll, pitch, yaw;

    pitch = asin(std::clamp(-R(2,0), -1.0, 1.0));

    // 处理奇异情况（pitch接近±90度）
    if (fabs(fabs(pitch) - M_PI/2.0) < 1e-6) {
        roll = 0;
        yaw = atan2(R(0,1), R(0,0));
    } else {
        roll = atan2(R(2,1), R(2,2));
        yaw = atan2(R(1,0), R(0,0));
    }

    return Eigen::Vector3d(roll, pitch, yaw);
}

// ====================== 2. 旋转矩阵 <-> 四元数 ======================
// 对应SLAM十四讲第3章 3.1.4 四元数
Eigen::Quaterniond GeometryMath::rot2Quat(const Eigen::Matrix3d& R) {
    Eigen::Quaterniond q(R);
    q.normalize(); // 四元数必须归一化
    return q;
}

Eigen::Matrix3d GeometryMath::quat2Rot(const Eigen::Quaterniond& q) {
    return q.normalized().toRotationMatrix();
}

// ====================== 3. 旋转向量 <-> 旋转矩阵（罗德里格斯公式） ======================
// 对应SLAM十四讲第3章 3.1.3 旋转向量与罗德里格斯公式
Eigen::Matrix3d GeometryMath::rotVec2Rot(const Eigen::Vector3d& rot_vec) {
    double theta = rot_vec.norm(); // 旋转角度 = 向量模长
    if (theta < 1e-8) {
        return Eigen::Matrix3d::Identity(); // 角度为0，返回单位矩阵
    }

    Eigen::Vector3d n = rot_vec / theta; // 单位旋转轴

    // 反对称矩阵
    Eigen::Matrix3d n_hat;
    n_hat << 0, -n(2), n(1),
             n(2), 0, -n(0),
             -n(1), n(0), 0;

    // 罗德里格斯公式：R = cosθ*I + (1-cosθ)*n*n^T + sinθ*n^
    return cos(theta) * Eigen::Matrix3d::Identity()
           + (1 - cos(theta)) * n * n.transpose()
           + sin(theta) * n_hat;
}

Eigen::Vector3d GeometryMath::rot2RotVec(const Eigen::Matrix3d& R) {
    double theta = acos(std::clamp((R.trace() - 1) / 2.0, -1.0, 1.0));
    if (theta < 1e-8) {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d rot_vec;
    rot_vec(0) = R(2,1) - R(1,2);
    rot_vec(1) = R(0,2) - R(2,0);
    rot_vec(2) = R(1,0) - R(0,1);
    rot_vec = rot_vec / (2 * sin(theta)) * theta;

    return rot_vec;
}

// ====================== 4. SO(3) BCH近似 左/右雅可比矩阵 ======================
// 对应SLAM十四讲第4章 4.1.6 BCH公式与近似
Eigen::Matrix3d GeometryMath::bchJacobianLeft(const Eigen::Vector3d& phi) {
    double theta = phi.norm();
    if (theta < 1e-8) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Vector3d a = phi / theta;
    Eigen::Matrix3d a_hat;
    a_hat << 0, -a(2), a(1),
             a(2), 0, -a(0),
             -a(1), a(0), 0;

    // 左雅可比：J_l = sinθ/θ * I + (1-sinθ/θ)*a*a^T + (1-cosθ)/θ * a^
    return (sin(theta)/theta) * Eigen::Matrix3d::Identity()
           + (1 - sin(theta)/theta) * a * a.transpose()
           + (1 - cos(theta))/theta * a_hat;
}

Eigen::Matrix3d GeometryMath::bchJacobianRight(const Eigen::Vector3d& phi) {
    // 右雅可比 = 左雅可比的转置
    return bchJacobianLeft(phi).transpose();
}

// ====================== 5. 旋转扰动更新（左扰动、右扰动） ======================
// 对应SLAM十四讲第4章 4.2 李代数求导与扰动模型
Eigen::Matrix3d GeometryMath::updateRotLeft(const Eigen::Matrix3d& R, const Eigen::Vector3d& delta) {
    // 左扰动：R_new = exp(delta^) * R_old
    return rotVec2Rot(delta) * R;
}

Eigen::Matrix3d GeometryMath::updateRotRight(const Eigen::Matrix3d& R, const Eigen::Vector3d& delta) {
    // 右扰动：R_new = R_old * exp(delta^)
    return R * rotVec2Rot(delta);
}

// ====================== 辅助工具：角度归一化到 [-pi, pi] ======================
double GeometryMath::normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}