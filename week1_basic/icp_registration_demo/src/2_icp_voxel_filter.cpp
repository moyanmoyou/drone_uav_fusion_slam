/*
 * 简化版ICP + 体素降采样演示说明
 * 已知问题与工程注意事项：
 * 1. 旋转向量初始值不能为全零，否则Ceres自动微分梯度为0，旋转参数完全不更新
 *    解决：初始值加入1e-6量级微小扰动
 * 2. 本Demo为简化ICP，假设源点与目标点索引一一对应；体素降采样后点序被哈希表打乱，对应关系失效，配准结果会出现偏差
 *    真实工程ICP完整流程：外层循环交替执行「最近邻匹配」+「位姿优化」，并配合初值预测、距离阈值外点剔除、Huber鲁棒核保证稳定性
 * 3. 本文件主要用于理解体素降采样的点数压缩效果，完整工业级ICP推荐使用PCL库
 */

#include <iostream>
#include <vector>
#include <unordered_map>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ctime>
#include <chrono>
#include <cmath>

using namespace std;
using namespace Eigen;

// ====================== 体素降采样函数 ======================
vector<Vector3d> voxelDownsample(const vector<Vector3d>& points, double voxel_size) {
    unordered_map<long long, pair<Vector3d, int>> voxel_map;

    for (const auto& p : points) {
        int ix = static_cast<int>(floor(p.x() / voxel_size));
        int iy = static_cast<int>(floor(p.y() / voxel_size));
        int iz = static_cast<int>(floor(p.z() / voxel_size));
        long long key = (long long)ix * 1000000 + (long long)iy * 1000 + iz;

        if (voxel_map.find(key) == voxel_map.end()) {
            voxel_map[key] = {p, 1};
        } else {
            voxel_map[key].first += p;
            voxel_map[key].second++;
        }
    }

    vector<Vector3d> downsampled;
    downsampled.reserve(voxel_map.size());
    for (auto& item : voxel_map) {
        item.second.first /= item.second.second;
        downsampled.push_back(item.second.first);
    }

    return downsampled;
}

// ====================== 暴力最近邻搜索 ======================
// 输入单个点 + 目标点云，返回最近点的索引和距离
int findNearestNeighbor(const Vector3d& p, const vector<Vector3d>& target, double& min_dist) {
    min_dist = 1e9;
    int min_idx = -1;
    for (int i = 0; i < target.size(); ++i) {
        double dist = (p - target[i]).squaredNorm();
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }
    return min_idx;
}

// ====================== ICP点到点残差函数 ======================
struct ICPResidual {
    ICPResidual(const Vector3d& p_src, const Vector3d& p_tgt) 
        : p_src_(p_src), p_tgt_(p_tgt) {}

    template <typename T>
    bool operator()(const T* const rot_vec, const T* const trans, T* residual) const {
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

        Matrix<T,3,1> t(trans[0], trans[1], trans[2]);
        Matrix<T,3,1> p_src_T = p_src_.cast<T>();
        Matrix<T,3,1> p_transformed = R * p_src_T + t;
        Matrix<T,3,1> p_tgt_T = p_tgt_.cast<T>();

        residual[0] = p_transformed[0] - p_tgt_T[0];
        residual[1] = p_transformed[1] - p_tgt_T[1];
        residual[2] = p_transformed[2] - p_tgt_T[2];

        return true;
    }

    Vector3d p_src_, p_tgt_;
};

// ====================== 旋转向量转旋转矩阵工具函数 ======================
Matrix3d rotVecToMatrix(const Vector3d& rot_vec) {
    double theta = rot_vec.norm();
    if (theta < 1e-8) {
        return Matrix3d::Identity();
    }
    Vector3d n = rot_vec / theta;
    Matrix3d n_hat;
    n_hat << 0, -n(2), n(1),
             n(2), 0, -n(0),
             -n(1), n(0), 0;
    return cos(theta)*Matrix3d::Identity()
         + (1-cos(theta))*n*n.transpose()
         + sin(theta)*n_hat;
}

int main() {
    // ====================== 1. 生成高密度模拟点云 ======================
    Matrix3d R_true = AngleAxisd(M_PI/6, Vector3d::UnitZ()).toRotationMatrix();
    Vector3d t_true(0.5, 0.3, 0.1);

    vector<Vector3d> src_points_raw;
    srand(time(0));
    for (int i = 0; i < 10000; ++i) {
        double x = (rand() / double(RAND_MAX) - 0.5) * 10.0;
        double y = (rand() / double(RAND_MAX) - 0.5) * 10.0;
        double z = (rand() / double(RAND_MAX) - 0.5) * 2.0;
        src_points_raw.emplace_back(x, y, z);
    }

    vector<Vector3d> tgt_points_raw;
    for (const auto& p : src_points_raw) {
        Vector3d p_tgt = R_true * p + t_true;
        p_tgt += Vector3d::Random() * 0.01;
        tgt_points_raw.push_back(p_tgt);
    }

    // ====================== 2. 体素降采样 ======================
    double voxel_size = 0.5;
    auto start_time = chrono::high_resolution_clock::now();

    vector<Vector3d> src_down = voxelDownsample(src_points_raw, voxel_size);
    vector<Vector3d> tgt_down = voxelDownsample(tgt_points_raw, voxel_size);

    auto end_time = chrono::high_resolution_clock::now();
    double downsample_time = chrono::duration<double>(end_time - start_time).count();

    cout << "===== Voxel Downsample Result =====" << endl;
    cout << "Raw points: " << src_points_raw.size() << endl;
    cout << "Downsampled points: " << src_down.size() << endl;
    cout << "Downsample time: " << downsample_time * 1000 << " ms" << endl;

    // ====================== 3. 完整ICP迭代配准 ======================
    // 初始位姿：微小扰动避免零梯度
    Vector3d rot_vec(1e-6, 1e-6, 1e-6);
    Vector3d trans(0.0, 0.0, 0.0);

    int max_icp_iter = 20;
    double converged_thresh = 1e-6;
    auto icp_total_start = chrono::high_resolution_clock::now();

    for (int iter = 0; iter < max_icp_iter; ++iter) {
        Matrix3d R_curr = rotVecToMatrix(rot_vec);

        // 步骤1：变换源点 + 寻找最近邻对应点
        vector<pair<Vector3d, Vector3d>> correspondences;
        double total_dist = 0.0;

        for (const auto& p_src : src_down) {
            Vector3d p_transformed = R_curr * p_src + trans;
            double min_dist;
            int idx = findNearestNeighbor(p_transformed, tgt_down, min_dist);
            if (idx >= 0) {
                correspondences.emplace_back(p_src, tgt_down[idx]);
                total_dist += sqrt(min_dist);
            }
        }

        double avg_dist = total_dist / correspondences.size();

        // 步骤2：构建Ceres优化问题，更新位姿
        double rot_arr[3] = {rot_vec[0], rot_vec[1], rot_vec[2]};
        double trans_arr[3] = {trans[0], trans[1], trans[2]};

        ceres::Problem problem;
        for (const auto& corr : correspondences) {
            ceres::CostFunction* cost =
                new ceres::AutoDiffCostFunction<ICPResidual, 3, 3, 3>(
                    new ICPResidual(corr.first, corr.second)
                );
            problem.AddResidualBlock(cost, nullptr, rot_arr, trans_arr);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // 步骤3：更新位姿，判断收敛
        Vector3d rot_new(rot_arr[0], rot_arr[1], rot_arr[2]);
        Vector3d trans_new(trans_arr[0], trans_arr[1], trans_arr[2]);

        double delta = (rot_new - rot_vec).norm() + (trans_new - trans).norm();
        rot_vec = rot_new;
        trans = trans_new;

        cout << "ICP iter " << iter+1 << ": avg error = " << avg_dist 
             << ", delta = " << delta << endl;

        if (delta < converged_thresh) {
            cout << "ICP converged at iteration " << iter+1 << endl;
            break;
        }
    }

    auto icp_total_end = chrono::high_resolution_clock::now();
    double icp_total_time = chrono::duration<double>(icp_total_end - icp_total_start).count();

    // ====================== 4. 结果输出 ======================
    Matrix3d R_opt = rotVecToMatrix(rot_vec);

    cout << "\n===== Final ICP Result =====" << endl;
    cout << "Total ICP time: " << icp_total_time * 1000 << " ms" << endl;
    cout << "Optimized rotation:\n" << R_opt << endl;
    cout << "Optimized translation: " << trans.transpose() << endl;

    double rot_error = (R_true - R_opt).norm();
    double trans_error = (t_true - trans).norm();
    cout << "\nRotation error: " << rot_error << endl;
    cout << "Translation error: " << trans_error << endl;

    return 0;
}