#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ctime>

using namespace std;
using namespace Eigen;

// ====================== ICP点到点残差函数 ======================
// 待优化参数：旋转向量(3维) + 平移向量(3维)
struct ICPResidual {
    ICPResidual(const Vector3d& p_src, const Vector3d& p_tgt) 
        : p_src_(p_src), p_tgt_(p_tgt) {}

    template <typename T>
    bool operator()(const T* const rot_vec, const T* const trans, T* residual) const {
        // 1. 旋转向量转旋转矩阵（罗德里格斯公式）
        T theta = sqrt(rot_vec[0]*rot_vec[0] + rot_vec[1]*rot_vec[1] + rot_vec[2]*rot_vec[2]);
        Matrix<T,3,3> R;

        if (theta < T(1e-8)) {
            R = Matrix<T,3,3>::Identity();
        } else {
            T nx = rot_vec[0] / theta;
            T ny = rot_vec[1] / theta;
            T nz = rot_vec[2] / theta;

            Matrix<T,3,3> n_hat;
            n_hat << T(0), -nz, ny,
                     nz, T(0), -nx,
                     -ny, nx, T(0);

            R = cos(theta) * Matrix<T,3,3>::Identity()
                + (T(1) - cos(theta)) * (Matrix<T,3,1>() << nx, ny, nz).finished() 
                                    * (Matrix<T,1,3>() << nx, ny, nz).finished()
                + sin(theta) * n_hat;
        }

        // 2. 平移向量
        Matrix<T,3,1> t(trans[0], trans[1], trans[2]);

        // 3. 源点经过变换后的坐标
        Matrix<T,3,1> p_src_T = p_src_.cast<T>();
        Matrix<T,3,1> p_transformed = R * p_src_T + t;

        // 4. 残差：变换后的点与目标点的坐标差（x/y/z三维残差）
        Matrix<T,3,1> p_tgt_T = p_tgt_.cast<T>();
        residual[0] = p_transformed[0] - p_tgt_T[0];
        residual[1] = p_transformed[1] - p_tgt_T[1];
        residual[2] = p_transformed[2] - p_tgt_T[2];

        return true;
    }

    Vector3d p_src_, p_tgt_;
};

int main() {
    // ====================== 1. 生成模拟点云与真值位姿 ======================
    // 真值变换：绕Z轴旋转30度，平移(0.5, 0.3, 0.1)
    Matrix3d R_true = AngleAxisd(M_PI/6, Vector3d::UnitZ()).toRotationMatrix();
    Vector3d t_true(0.5, 0.3, 0.1);

    cout << "===== Ground Truth Transformation =====" << endl;
    cout << "Rotation matrix:\n" << R_true << endl;
    cout << "Translation: " << t_true.transpose() << endl;

    // 生成100个随机三维点作为源点云
    vector<Vector3d> src_points;
    srand(time(0));
    for (int i = 0; i < 100; ++i) {
        double x = (rand() / double(RAND_MAX) - 0.5) * 10.0;
        double y = (rand() / double(RAND_MAX) - 0.5) * 10.0;
        double z = (rand() / double(RAND_MAX) - 0.5) * 2.0;
        src_points.emplace_back(x, y, z);
    }

    // 用真值变换生成目标点云，加入少量高斯噪声
    vector<Vector3d> tgt_points;
    for (const auto& p : src_points) {
        Vector3d p_tgt = R_true * p + t_true;
        p_tgt += Vector3d::Random() * 0.01; // 1cm噪声
        tgt_points.push_back(p_tgt);
    }

    // ====================== 2. 构建Ceres优化问题 ======================
    // 初始值：单位变换（旋转0，平移0）
    double rot_vec[3] = {1e-6, 1e-6, 1e-6};
    double trans[3] = {0.0, 0.0, 0.0};

    ceres::Problem problem;
    for (int i = 0; i < src_points.size(); ++i) {
        ceres::CostFunction* cost =
            new ceres::AutoDiffCostFunction<ICPResidual, 3, 3, 3>(
                new ICPResidual(src_points[i], tgt_points[i])
            );
        problem.AddResidualBlock(cost, nullptr, rot_vec, trans);
    }

    // ====================== 3. 求解优化 ======================
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);

    // ====================== 4. 结果转换与输出 ======================
    Vector3d rot_opt(rot_vec[0], rot_vec[1], rot_vec[2]);
    double theta = rot_opt.norm();
    Matrix3d R_opt;

    if (theta < 1e-8) {
        R_opt = Matrix3d::Identity();
    } else {
        Vector3d n = rot_opt / theta;
        Matrix3d n_hat;
        n_hat << 0, -n(2), n(1),
                 n(2), 0, -n(0),
                 -n(1), n(0), 0;
        R_opt = cos(theta)*Matrix3d::Identity()
              + (1-cos(theta))*n*n.transpose()
              + sin(theta)*n_hat;
    }
    Vector3d t_opt(trans[0], trans[1], trans[2]);

    cout << "\n===== Optimized Transformation =====" << endl;
    cout << "Rotation matrix:\n" << R_opt << endl;
    cout << "Translation: " << t_opt.transpose() << endl;

    // 计算误差
    double rot_error = (R_true - R_opt).norm();
    double trans_error = (t_true - t_opt).norm();
    cout << "\nRotation error: " << rot_error << endl;
    cout << "Translation error: " << trans_error << endl;
    cout << "Iterations: " << summary.iterations.size() << endl;
    cout << "Final cost: " << summary.final_cost << endl;

    return 0;
}