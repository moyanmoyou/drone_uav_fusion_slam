#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <ctime>

using namespace std;
using namespace Eigen;

// ====================== ICP点到点残差函数（复用任务1验证通过版本） ======================
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

// ====================== 工具：Eigen点云转ROS标准PointCloud2 ======================
sensor_msgs::PointCloud2 vectorToPointCloud2(const vector<Vector3d>& points, const string& frame_id) {
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(3,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& p : points) {
        *iter_x = p.x();
        *iter_y = p.y();
        *iter_z = p.z();
        ++iter_x; ++iter_y; ++iter_z;
    }

    return cloud_msg;
}

// ====================== 工具：旋转向量转旋转矩阵 ======================
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "icp_ros_node");
    ros::NodeHandle nh;
    tf2_ros::StaticTransformBroadcaster tf_broadcaster;

    // ====================== 1. 生成模拟点云与真值位姿 ======================
    Matrix3d R_true = AngleAxisd(M_PI/6, Vector3d::UnitZ()).toRotationMatrix();
    Vector3d t_true(0.5, 0.3, 0.1);

    vector<Vector3d> src_points;
    srand(time(0));
    for (int i = 0; i < 200; ++i) {
        double x = (rand() / double(RAND_MAX) - 0.5) * 10.0;
        double y = (rand() / double(RAND_MAX) - 0.5) * 10.0;
        double z = (rand() / double(RAND_MAX) - 0.5) * 2.0;
        src_points.emplace_back(x, y, z);
    }

    vector<Vector3d> tgt_points;
    for (const auto& p : src_points) {
        tgt_points.push_back(R_true * p + t_true);
    }

    // ====================== 2. 发布标准点云话题 ======================
    ros::Publisher src_pub = nh.advertise<sensor_msgs::PointCloud2>("/source_cloud", 1, true);
    ros::Publisher tgt_pub = nh.advertise<sensor_msgs::PointCloud2>("/target_cloud", 1, true);

    sensor_msgs::PointCloud2 src_cloud_msg = vectorToPointCloud2(src_points, "source_aligned_link");
    sensor_msgs::PointCloud2 tgt_cloud_msg = vectorToPointCloud2(tgt_points, "target_link");

    src_pub.publish(src_cloud_msg);
    tgt_pub.publish(tgt_cloud_msg);

    // ====================== 3. ICP配准 ======================
    double rot_arr[3] = {1e-6, 1e-6, 1e-6};
    double trans_arr[3] = {0.0, 0.0, 0.0};

    ceres::Problem problem;
    for (int i = 0; i < src_points.size(); ++i) {
        ceres::CostFunction* cost =
            new ceres::AutoDiffCostFunction<ICPResidual, 3, 3, 3>(
                new ICPResidual(src_points[i], tgt_points[i])
            );
        problem.AddResidualBlock(cost, nullptr, rot_arr, trans_arr);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 结果转换
    Vector3d rot_opt(rot_arr[0], rot_arr[1], rot_arr[2]);
    Matrix3d R_opt = rotVecToMatrix(rot_opt);
    Vector3d t_opt(trans_arr[0], trans_arr[1], trans_arr[2]);
    Quaterniond q_opt(R_opt);

    ROS_INFO("ICP registration finished.");
    ROS_INFO("Translation: %.3f %.3f %.3f", t_opt.x(), t_opt.y(), t_opt.z());
    ROS_INFO("Rotation quaternion: %.3f %.3f %.3f %.3f", q_opt.x(), q_opt.y(), q_opt.z(), q_opt.w());

    // ====================== 4. 发布TF坐标变换 ======================
    // world -> target_link：目标点云固定在世界坐标系
    geometry_msgs::TransformStamped tgt_tf;
    tgt_tf.header.stamp = ros::Time::now();
    tgt_tf.header.frame_id = "world";
    tgt_tf.child_frame_id = "target_link";
    tgt_tf.transform.translation.x = 0;
    tgt_tf.transform.translation.y = 0;
    tgt_tf.transform.translation.z = 0;
    tgt_tf.transform.rotation.x = 0;
    tgt_tf.transform.rotation.y = 0;
    tgt_tf.transform.rotation.z = 0;
    tgt_tf.transform.rotation.w = 1;

    // world -> source_aligned_link：ICP配准后的源点云坐标系
    geometry_msgs::TransformStamped aligned_tf;
    aligned_tf.header.stamp = ros::Time::now();
    aligned_tf.header.frame_id = "world";
    aligned_tf.child_frame_id = "source_aligned_link";
    aligned_tf.transform.translation.x = t_opt.x();
    aligned_tf.transform.translation.y = t_opt.y();
    aligned_tf.transform.translation.z = t_opt.z();
    aligned_tf.transform.rotation.x = q_opt.x();
    aligned_tf.transform.rotation.y = q_opt.y();
    aligned_tf.transform.rotation.z = q_opt.z();
    aligned_tf.transform.rotation.w = q_opt.w();

    tf_broadcaster.sendTransform(tgt_tf);
    tf_broadcaster.sendTransform(aligned_tf);

    ROS_INFO("TF published: world -> target_link, world -> source_aligned_link");
    ROS_INFO("Open RViz, set Fixed Frame to 'world', add PointCloud2 to view clouds.");

    ros::spin();
    return 0;
}