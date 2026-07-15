#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ctime>
#include <cstdlib>

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "standard_cloud_publisher");
    ros::NodeHandle nh;
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/raw_pointcloud", 10);

    ros::Rate rate(10); // 10Hz发布，对标真实机械激光雷达频率
    srand(time(0));

    ROS_INFO("Standard point cloud publisher started, 10Hz");

    while (ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->header.frame_id = "lidar_link";
        cloud->height = 1;

        // 生成主体平面点 + 高斯噪声
        for (int i = 0; i < 500; ++i) {
            double x = (rand() / double(RAND_MAX) - 0.5) * 10.0;
            double y = (rand() / double(RAND_MAX) - 0.5) * 10.0;
            double z = 0.0 + (rand() / double(RAND_MAX) - 0.5) * 0.1;
            cloud->points.emplace_back(x, y, z);
        }

        // 添加随机离群噪点，模拟真实雷达的干扰点
        for (int i = 0; i < 50; ++i) {
            double x = (rand() / double(RAND_MAX) - 0.5) * 15.0;
            double y = (rand() / double(RAND_MAX) - 0.5) * 15.0;
            double z = (rand() / double(RAND_MAX) - 0.5) * 5.0;
            cloud->points.emplace_back(x, y, z);
        }

        cloud->width = cloud->points.size();

        // PCL点云转ROS标准消息发布
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = ros::Time::now();
        cloud_pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
