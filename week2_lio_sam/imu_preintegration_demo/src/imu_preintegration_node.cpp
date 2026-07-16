#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>

using namespace Eigen;

class ImuPreintegration {
public:
    ImuPreintegration() {
        // 预标定的零偏值，与模拟器参数对应
        acc_bias_ << 0.02, 0.015, 0.01;
        gyro_bias_ << 0.005, 0.003, 0.002;
        gravity_ << 0, 0, 9.81;
        reset();
    }

    void reset() {
        delta_p_.setZero();
        delta_v_.setZero();
        delta_q_.setIdentity();
        sum_dt_ = 0.0;
        imu_buffer_.clear();
    }

    // 输入一帧IMU数据，执行中值积分
    void pushImu(const sensor_msgs::ImuConstPtr& msg) {
        Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        double dt = 0.01; // 100Hz固定时间间隔

        if (imu_buffer_.empty()) {
            imu_buffer_.push_back({acc, gyro});
            return;
        }

        // 取前后两帧测量值的中值，扣除零偏
        ImuData prev = imu_buffer_.back();
        Vector3d acc_mid = 0.5 * (prev.acc + acc) - acc_bias_;
        Vector3d gyro_mid = 0.5 * (prev.gyro + gyro) - gyro_bias_;

        // 姿态增量：旋转向量转四元数
        double theta = gyro_mid.norm() * dt;
        Quaterniond dq;
        if (theta < 1e-8) {
            dq.setIdentity();
        } else {
            Vector3d axis = gyro_mid.normalized();
            dq = AngleAxisd(theta, axis);
        }

        // 核心修复：比力转世界系后扣除重力，得到真实运动加速度
        Vector3d acc_world = delta_q_ * acc_mid - gravity_;

        // 中值积分更新位置、速度、姿态
        delta_p_ += delta_v_ * dt + 0.5 * acc_world * dt * dt;
        delta_v_ += acc_world * dt;
        delta_q_ = delta_q_ * dq;

        sum_dt_ += dt;
        imu_buffer_.pop_front();
        imu_buffer_.push_back({acc, gyro});
    }

    // 打印结果并与真值对比
    void printResult() {
        double t = sum_dt_;
        // 真值：静止开始的匀加速直线运动，加速度 (0.5, 0.3, 0) m/s²
        Vector3d true_acc(0.5, 0.3, 0.0);
        Vector3d true_pos = 0.5 * true_acc * t * t;
        Vector3d true_vel = true_acc * t;

        ROS_INFO("===== Preintegration Result (%.3fs) =====", sum_dt_);
        ROS_INFO("Delta Position:  %.3f  %.3f  %.3f", delta_p_.x(), delta_p_.y(), delta_p_.z());
        ROS_INFO("True Position:   %.3f  %.3f  %.3f", true_pos.x(), true_pos.y(), true_pos.z());
        ROS_INFO("Delta Velocity:  %.3f  %.3f  %.3f", delta_v_.x(), delta_v_.y(), delta_v_.z());
        ROS_INFO("True Velocity:   %.3f  %.3f  %.3f", true_vel.x(), true_vel.y(), true_vel.z());
    }

private:
    struct ImuData {
        Vector3d acc;
        Vector3d gyro;
    };

    Vector3d delta_p_;
    Vector3d delta_v_;
    Quaterniond delta_q_;
    double sum_dt_;

    Vector3d acc_bias_, gyro_bias_;
    Vector3d gravity_;
    std::deque<ImuData> imu_buffer_;
};

class PreintegrationNode {
public:
    PreintegrationNode() {
        imu_sub_ = nh_.subscribe("/imu", 1000, &PreintegrationNode::imuCallback, this);
        preint_ = new ImuPreintegration();
        last_print_time_ = ros::Time::now().toSec();
        ROS_INFO("IMU preintegration node started.");
    }

    ~PreintegrationNode() {
        delete preint_;
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        preint_->pushImu(msg);

        // 每累计1秒打印一次结果并重置
        double now = msg->header.stamp.toSec();
        if (now - last_print_time_ >= 1.0) {
            preint_->printResult();
            preint_->reset();
            last_print_time_ = now;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ImuPreintegration* preint_;
    double last_print_time_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_preintegration_node");
    PreintegrationNode node;
    ros::spin();
    return 0;
}