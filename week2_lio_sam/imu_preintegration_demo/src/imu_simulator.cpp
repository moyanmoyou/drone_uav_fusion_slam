#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

using namespace Eigen;

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_simulator");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 100);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/imu_true_pose", 100);

    // IMU频率100Hz
    double imu_rate = 100.0;
    double dt = 1.0 / imu_rate;
    ros::Rate rate(imu_rate);

    // 运动真值：静止开始，匀加速直线运动
    Vector3d linear_acc(0.5, 0.3, 0.0); // 世界坐标系加速度 m/s²
    Vector3d linear_vel(0.0, 0.0, 0.0);
    Vector3d pos(0.0, 0.0, 0.0);
    Quaterniond quat = Quaterniond::Identity(); // 姿态保持不变

    // IMU误差参数
    double acc_noise_std = 0.01;
    double gyro_noise_std = 0.001;
    Vector3d acc_bias(0.02, 0.015, 0.01);
    Vector3d gyro_bias(0.005, 0.003, 0.002);
    Vector3d gravity(0, 0, 9.81);

    // 高斯噪声生成器
    std::default_random_engine generator;
    std::normal_distribution<double> acc_noise(0, acc_noise_std);
    std::normal_distribution<double> gyro_noise(0, gyro_noise_std);

    ROS_INFO("IMU simulator started, 100Hz");

    while (ros::ok()) {
        // 更新真值速度、位置
        linear_vel += linear_acc * dt;
        pos += linear_vel * dt;

        // 机体坐标系下的比力测量 = 加速度反方向 + 重力
        Vector3d acc_meas = quat.inverse() * (gravity + linear_acc);
        // 陀螺仪测量：姿态不变，角速度为0
        Vector3d gyro_meas(0, 0, 0);

        // 叠加零偏和噪声
        acc_meas += acc_bias;
        gyro_meas += gyro_bias;
        for (int i = 0; i < 3; ++i) {
            acc_meas[i] += acc_noise(generator);
            gyro_meas[i] += gyro_noise(generator);
        }

        // 发布IMU消息
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";

        imu_msg.angular_velocity.x = gyro_meas.x();
        imu_msg.angular_velocity.y = gyro_meas.y();
        imu_msg.angular_velocity.z = gyro_meas.z();

        imu_msg.linear_acceleration.x = acc_meas.x();
        imu_msg.linear_acceleration.y = acc_meas.y();
        imu_msg.linear_acceleration.z = acc_meas.z();

        imu_msg.orientation.w = quat.w();
        imu_msg.orientation.x = quat.x();
        imu_msg.orientation.y = quat.y();
        imu_msg.orientation.z = quat.z();

        imu_pub.publish(imu_msg);

        // 发布真值位姿
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = imu_msg.header;
        pose_msg.pose.position.x = pos.x();
        pose_msg.pose.position.y = pos.y();
        pose_msg.pose.position.z = pos.z();
        pose_msg.pose.orientation.w = quat.w();
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_pub.publish(pose_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}