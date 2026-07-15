#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <ctime>

using namespace Eigen;

// 构造三维反对称矩阵
template<typename T>
Matrix<T,3,3> skew(const Matrix<T,3,1>& v)
{
    Matrix<T,3,3> mat;
    mat(0,0) = T(0);    mat(0,1) = -v(2); mat(0,2) = v(1);
    mat(1,0) = v(2);    mat(1,1) = T(0);    mat(1,2) = -v(0);
    mat(2,0) = -v(1);   mat(2,1) = v(0);    mat(2,2) = T(0);
    return mat;
}

// 点到面ICP残差项：3维旋转向量 + 3维平移
struct Point2PlaneResidual {
    Point2PlaneResidual(Vector3d src_pt, Vector3d tgt_pt, Vector3d tgt_normal)
        : src_pt_(src_pt), tgt_pt_(tgt_pt), tgt_normal_(tgt_normal) {}

    template <typename T>
    bool operator()(const T* const rot_vec, const T* const trans, T* residual) const {
        Matrix<T,3,1> r(rot_vec[0], rot_vec[1], rot_vec[2]);
        T theta = r.norm();
        Matrix<T,3,3> R;

        if (theta < T(1e-8))
        {
            R.setIdentity();
        }
        else
        {
            Matrix<T,3,1> u = r / theta;
            T cos_t = cos(theta);
            T sin_t = sin(theta);
            Matrix<T,3,3> I = Matrix<T,3,3>::Identity();
            Matrix<T,3,3> uuT = u * u.transpose();
            Matrix<T,3,3> S = skew(u);
            // 罗德里格斯旋转公式
            R = cos_t * I + (T(1) - cos_t) * uuT + sin_t * S;
        }

        Matrix<T,3,1> t(trans[0], trans[1], trans[2]);
        Matrix<T,3,1> p_src(T(src_pt_.x()), T(src_pt_.y()), T(src_pt_.z()));
        Matrix<T,3,1> p_tf = R * p_src + t;

        Matrix<T,3,1> p_tgt(T(tgt_pt_.x()), T(tgt_pt_.y()), T(tgt_pt_.z()));
        Matrix<T,3,1> n_tgt(T(tgt_normal_.x()), T(tgt_normal_.y()), T(tgt_normal_.z()));

        residual[0] = n_tgt.dot(p_tf - p_tgt);
        return true;
    }

    Vector3d src_pt_, tgt_pt_, tgt_normal_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "point2plane_icp_node");
    ros::NodeHandle nh;
    tf2_ros::StaticTransformBroadcaster tf_broadcaster;

    // 真值变换：绕Z轴转30度，平移(0.5, 0.3, 0.1)
    Matrix3d R_true = AngleAxisd(M_PI/6.0, Vector3d::UnitZ()).toRotationMatrix();
    Vector3d t_true(0.5, 0.3, 0.1);

    std::vector<Vector3d> src_points;
    srand(time(0));
    for (int i = 0; i < 200; ++i)
    {
        double x = (rand() / double(RAND_MAX) - 0.5) * 10.0;
        double y = (rand() / double(RAND_MAX) - 0.5) * 10.0;
        double z = 0.0;
        src_points.emplace_back(x, y, z);
    }

    std::vector<Vector3d> tgt_points, tgt_normals;
    for (auto& p : src_points)
    {
        tgt_points.push_back(R_true * p + t_true);
        tgt_normals.push_back(R_true * Vector3d(0,0,1));
    }

    // 优化变量：[rx, ry, rz, tx, ty, tz]
    double rot_vec[3] = {1e-6, 1e-6, 1e-6};
    double trans[3] = {0.0, 0.0, 0.0};

    ceres::Problem problem;
    for (size_t i = 0; i < src_points.size(); ++i)
    {
        ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<Point2PlaneResidual, 1, 3, 3>(
            new Point2PlaneResidual(src_points[i], tgt_points[i], tgt_normals[i])
        );
        problem.AddResidualBlock(cost, nullptr, rot_vec, trans);
    }

    ceres::Solver::Options opts;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);

    // ========== 修复：旋转向量直接计算四元数，避开AngleAxis构造问题 ==========
    Vector3d rv(rot_vec[0], rot_vec[1], rot_vec[2]);
    double angle = rv.norm();
    Quaterniond q_opt;

    if (angle < 1e-8) {
        q_opt.setIdentity();
    } else {
        Vector3d axis = rv / angle;
        double half_angle = angle * 0.5;
        double sin_half = sin(half_angle);
        double cos_half = cos(half_angle);
        q_opt.x() = axis.x() * sin_half;
        q_opt.y() = axis.y() * sin_half;
        q_opt.z() = axis.z() * sin_half;
        q_opt.w() = cos_half;
    }

    Vector3d t_opt(trans[0], trans[1], trans[2]);

    ROS_INFO("==== Point-to-Plane ICP Output ====");
    ROS_INFO("Estimate T: %.3f %.3f %.3f", t_opt.x(), t_opt.y(), t_opt.z());
    ROS_INFO("Ground T:   0.500 0.300 0.100");
    ROS_INFO("Rotation angle: %.3f rad (truth: %.3f rad)", angle, M_PI/6.0);
    ROS_INFO("Quat xyzw: %.3f %.3f %.3f %.3f", q_opt.x(), q_opt.y(), q_opt.z(), q_opt.w());

    // 发布TF
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "icp_aligned";
    tf_msg.transform.translation.x = t_opt.x();
    tf_msg.transform.translation.y = t_opt.y();
    tf_msg.transform.translation.z = t_opt.z();
    tf_msg.transform.rotation.x = q_opt.x();
    tf_msg.transform.rotation.y = q_opt.y();
    tf_msg.transform.rotation.z = q_opt.z();
    tf_msg.transform.rotation.w = q_opt.w();
    tf_broadcaster.sendTransform(tf_msg);

    ROS_INFO("TF world -> icp_aligned published");
    ros::spin();
    return 0;
}