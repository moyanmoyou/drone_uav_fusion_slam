#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <cmath>
#include <ctime>

using namespace std;

// 通用代价函数：二次曲线拟合残差
struct CurveResidual {
    CurveResidual(double x, double y) : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const abc, T* residual) const {
        T y_pred = abc[0] * x_ * x_ + abc[1] * x_ + abc[2];
        residual[0] = T(y_) - y_pred;
        return true;
    }

    double x_, y_;
};

int main() {
    // ====================== 1. 生成带外点的模拟数据 ======================
    double a_true = 2.0, b_true = -1.0, c_true = 0.5;
    vector<double> x_data, y_data;
    int point_num = 100;

    srand(time(0));
    for (int i = 0; i < point_num; ++i) {
        double x = i * 0.05;
        double y = a_true * x * x + b_true * x + c_true;
        // 正常高斯噪声
        y += (rand() / double(RAND_MAX) - 0.5) * 0.1;

        // 故意加入10个明显外点（野值）
        if (i % 10 == 0) {
            y += 5.0; // 偏离真值5个单位的野值
        }

        x_data.push_back(x);
        y_data.push_back(y);
    }

    // ====================== 2. 普通最小二乘拟合（无鲁棒核） ======================
    double abc_normal[3] = {0.0, 0.0, 0.0};
    ceres::Problem problem_normal;
    for (int i = 0; i < point_num; ++i) {
        ceres::CostFunction* cost =
            new ceres::AutoDiffCostFunction<CurveResidual, 1, 3>(
                new CurveResidual(x_data[i], y_data[i])
            );
        problem_normal.AddResidualBlock(cost, nullptr, abc_normal);
    }

    // ====================== 3. Huber鲁棒核拟合 ======================
    double abc_huber[3] = {0.0, 0.0, 0.0};
    ceres::Problem problem_huber;
    for (int i = 0; i < point_num; ++i) {
        ceres::CostFunction* cost =
            new ceres::AutoDiffCostFunction<CurveResidual, 1, 3>(
                new CurveResidual(x_data[i], y_data[i])
            );
        // 关键：加入Huber鲁棒核，阈值设为1.0，残差大于阈值时梯度被抑制
        ceres::LossFunction* loss = new ceres::HuberLoss(1.0);
        problem_huber.AddResidualBlock(cost, loss, abc_huber);
    }

    // ====================== 4. 统一求解配置 ======================
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary_normal, summary_huber;

    ceres::Solve(options, &problem_normal, &summary_normal);
    ceres::Solve(options, &problem_huber, &summary_huber);

    // ====================== 5. 对比输出结果 ======================
    cout << "====== 外点下两种拟合结果对比 ======" << endl;
    cout << "真值:          a=" << a_true << "  b=" << b_true << "  c=" << c_true << endl;
    cout << "普通最小二乘:  a=" << abc_normal[0] << "  b=" << abc_normal[1] << "  c=" << abc_normal[2] << endl;
    cout << "Huber鲁棒核:   a=" << abc_huber[0] << "  b=" << abc_huber[1] << "  c=" << abc_huber[2] << endl;

    cout << "\n结论：普通最小二乘被外点严重带偏，Huber核能有效抵抗野值影响" << endl;

    return 0;
}