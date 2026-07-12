#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ctime>

using namespace std;
using namespace Eigen;

// 模板化四元数转旋转矩阵（兼容Ceres自动微分）
template <typename T>
Matrix<T,3,3> quatToRotMat(const T* q) {
    // q顺序: w, x, y, z
    T w = q[0], x = q[1], y = q[2], z = q[3];
    Matrix<T,3,3> R;
    R << T(1)-T(2)*y*y-T(2)*z*z,  T(2)*x*y-T(2)*w*z,        T(2)*x*z+T(2)*w*y,
         T(2)*x*y+T(2)*w*z,        T(1)-T(2)*x*x-T(2)*z*z,  T(2)*y*z-T(2)*w*x,
         T(2)*x*z-T(2)*w*y,        T(2)*y*z+T(2)*w*x,        T(1)-T(2)*x*x-T(2)*y*y;
    return R;
}

// 模板化旋转矩阵转旋转向量（兼容自动微分）
template <typename T>
void rotMatToVec(const Matrix<T,3,3>& R, T* rot_vec) {
    T trace = R.trace();
    T cos_theta = (trace - T(1)) / T(2);
    // 数值截断，防止acos超出定义域
    cos_theta = std::clamp(cos_theta, T(-1), T(1));
    T theta = acos(cos_theta);

    if (theta < T(1e-8)) {
        rot_vec[0] = T(0);
        rot_vec[1] = T(0);
        rot_vec[2] = T(0);
        return;
    }

    T half_sin = T(2) * sin(theta);
    rot_vec[0] = (R(2,1) - R(1,2)) / half_sin * theta;
    rot_vec[1] = (R(0,2) - R(2,0)) / half_sin * theta;
    rot_vec[2] = (R(1,0) - R(0,1)) / half_sin * theta;
}

// ====================== 手眼标定残差函数 ======================
// 约束: A * X = X * B
// 参数: q_x(4维四元数wxyz), t_x(3维平移)
struct HandEyeResidual {
    HandEyeResidual(const Matrix3d& R_A, const Vector3d& t_A,
                    const Matrix3d& R_B, const Vector3d& t_B)
        : R_A_(R_A), t_A_(t_A), R_B_(R_B), t_B_(t_B) {}

    template <typename T>
    bool operator()(const T* const q_x, const T* const t_x, T* residual) const {
        // 1. 外参旋转矩阵
        Matrix<T,3,3> R_x = quatToRotMat(q_x);
        Matrix<T,3,1> t(t_x[0], t_x[1], t_x[2]);

        // 2. 计算 A*X 的旋转和平移
        Matrix<T,3,3> R_AX = R_A_.cast<T>() * R_x;
        Matrix<T,3,1> t_AX = R_A_.cast<T>() * t + t_A_.cast<T>();

        // 3. 计算 X*B 的旋转和平移
        Matrix<T,3,3> R_XB = R_x * R_B_.cast<T>();
        Matrix<T,3,1> t_XB = R_x * t_B_.cast<T>() + t;

        // 4. 旋转误差: R_err = R_AX^T * R_XB
        Matrix<T,3,3> R_err = R_AX.transpose() * R_XB;
        T rot_res[3];
        rotMatToVec(R_err, rot_res);
        residual[0] = rot_res[0];
        residual[1] = rot_res[1];
        residual[2] = rot_res[2];

        // 5. 平移误差
        Matrix<T,3,1> t_err = R_AX.transpose() * (t_XB - t_AX);
        residual[3] = t_err(0);
        residual[4] = t_err(1);
        residual[5] = t_err(2);

        return true;
    }

    Matrix3d R_A_;
    Vector3d t_A_;
    Matrix3d R_B_;
    Vector3d t_B_;
};

int main() {
    // ====================== 1. 生成真值外参 ======================
    Matrix3d R_true = AngleAxisd(M_PI/6, Vector3d::UnitZ())
                    * AngleAxisd(M_PI/18, Vector3d::UnitY())
                    .toRotationMatrix();
    Vector3d t_true(0.1, 0.2, 0.3);
    Quaterniond q_true(R_true);

    cout << "====== 真值外参 ======" << endl;
    cout << "旋转矩阵:\n" << R_true << endl;
    cout << "平移向量: " << t_true.transpose() << endl;

    // ====================== 2. 生成30组配对的A、B运动 ======================
    vector<Matrix3d> R_A_list, R_B_list;
    vector<Vector3d> t_A_list, t_B_list;
    srand(time(0));

    for (int i = 0; i < 30; ++i) {
        // 生成随机相机运动A
        Matrix3d R_A = AngleAxisd( (rand()/double(RAND_MAX)-0.5)*1.0, Vector3d::UnitZ())
                     * AngleAxisd( (rand()/double(RAND_MAX)-0.5)*0.8, Vector3d::UnitY())
                     * AngleAxisd( (rand()/double(RAND_MAX)-0.5)*0.5, Vector3d::UnitX())
                     .toRotationMatrix();
        Vector3d t_A( (rand()/double(RAND_MAX)-0.5)*1.5,
                      (rand()/double(RAND_MAX)-0.5)*1.5,
                      (rand()/double(RAND_MAX)-0.5)*1.5 );

        // 根据 AX=XB 计算 B = X^{-1} A X
        Matrix3d R_B = R_true.transpose() * R_A * R_true;
        Vector3d t_B = R_true.transpose() * (R_A * t_true + t_A - t_true);

        // 加入微小测量噪声
        t_B += Vector3d::Random() * 0.002;

        R_A_list.push_back(R_A);
        t_A_list.push_back(t_A);
        R_B_list.push_back(R_B);
        t_B_list.push_back(t_B);
    }

    // ====================== 3. 构建Ceres优化问题 ======================
    // 初始值：单位四元数 + 零平移
    double q_x[4] = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
    double t_x[3] = {0.0, 0.0, 0.0};

    ceres::Problem problem;

    // 四元数过参数化，使用Ceres官方参数化
    ceres::Manifold* quat_param = new ceres::EigenQuaternionManifold();

    for (int i = 0; i < R_A_list.size(); ++i) {
        ceres::CostFunction* cost =
            new ceres::AutoDiffCostFunction<HandEyeResidual, 6, 4, 3>(
                new HandEyeResidual(R_A_list[i], t_A_list[i], R_B_list[i], t_B_list[i])
            );
        problem.AddResidualBlock(cost, nullptr, q_x, t_x);
    }

    // 给四元数参数块设置局部参数化
    problem.SetManifold(q_x, quat_param);

    // ====================== 4. 求解优化 ======================
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    options.function_tolerance = 1e-10;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);

    // ====================== 5. 结果输出 ======================
    Quaterniond q_opt(q_x[0], q_x[1], q_x[2], q_x[3]);
    q_opt.normalize();
    Matrix3d R_opt = q_opt.toRotationMatrix();
    Vector3d t_opt(t_x[0], t_x[1], t_x[2]);

    cout << "\n====== 优化后外参 ======" << endl;
    cout << "旋转矩阵:\n" << R_opt << endl;
    cout << "平移向量: " << t_opt.transpose() << endl;

    double rot_error = (R_true - R_opt).norm();
    double trans_error = (t_true - t_opt).norm();
    cout << "\n旋转误差: " << rot_error << endl;
    cout << "平移误差: " << trans_error << endl;
    cout << "迭代次数: " << summary.iterations.size() << endl;
    cout << "初始残差: " << summary.initial_cost << endl;
    cout << "最终残差: " << summary.final_cost << endl;

    return 0;
}