#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <cmath>
#include <ctime>
#include<ceres/version.h>

using namespace std;

// 代价函数：计算残差 = 观测值y - 拟合值a*x²+b*x+c
struct CurveFittingResidual {
    CurveFittingResidual(double x, double y) : x_(x), y_(y) {}

    // 模板函数，用于Ceres自动微分
    template <typename T>
    bool operator()(const T* const abc, T* residual) const {
        // 拟合模型：y = a*x² + b*x + c
        T y_pred = abc[0] * x_ * x_ + abc[1] * x_ + abc[2];
        residual[0] = T(y_) - y_pred;
        return true;
    }

    double x_, y_;
};

int main() {
    // ====================== 1. 生成带噪声的模拟数据 ======================
    double a_true = 2.0, b_true = -1.0, c_true = 0.5; // 真值
    vector<double> x_data, y_data;
    int point_num = 100;

    // 初始化随机数种子
    srand(time(0));
    for (int i = 0; i < point_num; ++i) {
        double x = i * 0.05;
        double y = a_true * x * x + b_true * x + c_true;
        // 加入高斯噪声
        y += (rand() / double(RAND_MAX) - 0.5) * 0.1;
        x_data.push_back(x);
        y_data.push_back(y);
    }

    // ====================== 2. 构建优化问题 ======================
    double abc[3] = {0.0, 0.0, 0.0}; // 待优化参数初始值，全部设为0

    ceres::Problem problem;
    for (int i = 0; i < point_num; ++i) {
        // 为每个数据点添加残差块：自动微分、残差维度1、参数维度3
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<CurveFittingResidual, 1, 3>(
                new CurveFittingResidual(x_data[i], y_data[i])
            );
        problem.AddResidualBlock(cost_function, nullptr, abc);
    }

    // ====================== 3. 配置求解器并运行优化 ======================
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; // 小型问题用QR分解
    options.minimizer_progress_to_stdout = true;  // 打印迭代过程
    ceres::Solver::Summary summary;               // 优化结果汇总

    ceres::Solve(options, &problem, &summary);

    // ====================== 4. 输出结果 ======================
    cout << "\n====== 优化结果 ======" << endl;
    cout << "真值:      a=" << a_true << "  b=" << b_true << "  c=" << c_true << endl;
    cout << "优化结果:  a=" << abc[0] << "  b=" << abc[1] << "  c=" << abc[2] << endl;
    cout << "迭代次数: " << summary.iterations.size() << endl;
    cout << "最终总残差: " << summary.final_cost << endl;
    cout << "Ceres version: " << CERES_VERSION_STRING << endl;

    return 0;
}