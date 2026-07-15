#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ctime>

int main(int argc, char **argv) {
    ros::init(argc, argv, "structured_cloud_publisher");
    ros::NodeHandle nh;
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/structured_cloud", 10);
    ros::Rate rate(10);
    srand(time(0));

    while (ros::ok()) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->header.frame_id = "lidar_link";
        cloud->height = 1;

        // 1. 地面平面点（Z=0）
        for (int i = 0; i < 800; ++i) {
            pcl::PointXYZRGB pt;
            pt.x = (rand()/double(RAND_MAX)-0.5)*12.0;
            pt.y = (rand()/double(RAND_MAX)-0.5)*12.0;
            pt.z = 0.0 + (rand()/double(RAND_MAX)-0.5)*0.08;
            pt.r = 200; pt.g = 200; pt.b = 200;
            cloud->points.push_back(pt);
        }

        // 2. 墙面1：X=5的竖直平面
        for (int i = 0; i < 300; ++i) {
            pcl::PointXYZRGB pt;
            pt.x = 5.0 + (rand()/double(RAND_MAX)-0.5)*0.08;
            pt.y = (rand()/double(RAND_MAX)-0.5)*10.0;
            pt.z = (rand()/double(RAND_MAX))*4.0;
            pt.r = 200; pt.g = 200; pt.b = 200;
            cloud->points.push_back(pt);
        }

        // 3. 墙面2：Y=5的竖直平面
        for (int i = 0; i < 300; ++i) {
            pcl::PointXYZRGB pt;
            pt.x = (rand()/double(RAND_MAX)-0.5)*10.0;
            pt.y = 5.0 + (rand()/double(RAND_MAX)-0.5)*0.08;
            pt.z = (rand()/double(RAND_MAX))*4.0;
            pt.r = 200; pt.g = 200; pt.b = 200;
            cloud->points.push_back(pt);
        }

        // 4. 少量离群噪点
        for (int i = 0; i < 40; ++i) {
            pcl::PointXYZRGB pt;
            pt.x = (rand()/double(RAND_MAX)-0.5)*18.0;
            pt.y = (rand()/double(RAND_MAX)-0.5)*18.0;
            pt.z = (rand()/double(RAND_MAX)-0.5)*6.0;
            pt.r = 255; pt.g = 0; pt.b = 0;
            cloud->points.push_back(pt);
        }

        cloud->width = cloud->points.size();
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);

        for (auto &field : msg.fields) {
            if (field.name == "rgb") {
                field.datatype = sensor_msgs::PointField::UINT32;
                break;
            }
        }

        msg.header.stamp = ros::Time::now();
        cloud_pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}