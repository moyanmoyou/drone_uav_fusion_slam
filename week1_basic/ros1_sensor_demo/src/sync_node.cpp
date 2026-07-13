#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ros1_sensor_demo/CustomImu.h"
#include "ros1_sensor_demo/CustomPointCloud.h"

using namespace ros1_sensor_demo;
using namespace message_filters;

// 同步回调函数：两个话题时间戳对齐成功后触发
void syncCallback(const CustomImuConstPtr& imu_msg, const CustomPointCloudConstPtr& cloud_msg) {
    // 打印两个消息的时间戳，验证时间差
    double imu_time = imu_msg->header.stamp.toSec();
    double cloud_time = cloud_msg->header.stamp.toSec();
    double time_diff = fabs(imu_time - cloud_time);

    ROS_INFO("Sync success! IMU time: %.6f, Cloud time: %.6f, time diff: %.6f s",
             imu_time, cloud_time, time_diff);
    ROS_INFO("  -> IMU acc z: %.2f m/s^2, Cloud point num: %d",
             imu_msg->linear_acceleration[2], cloud_msg->point_num);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh;

    // 1. 创建两个message_filters订阅者，替代普通ros::Subscriber
    message_filters::Subscriber<CustomImu> imu_sub(nh, "/custom_imu", 100);
    message_filters::Subscriber<CustomPointCloud> cloud_sub(nh, "/custom_pointcloud", 10);

    // 2. 定义同步策略：近似时间同步，输入两个话题
    typedef sync_policies::ApproximateTime<CustomImu, CustomPointCloud> MySyncPolicy;

    // 3. 创建同步器，队列大小设为10
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, cloud_sub);

    // 4. 注册同步回调函数
    sync.registerCallback(boost::bind(&syncCallback, _1, _2));

    ROS_INFO("Multi-sensor sync node started, waiting for data...");

    // 进入循环等待回调
    ros::spin();

    return 0;
}