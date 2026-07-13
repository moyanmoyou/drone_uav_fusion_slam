#include<ros/ros.h>
#include<ros1_sensor_demo/CustomImu.h>
#include<ros1_sensor_demo/CustomPointCloud.h>

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "sensor_publisher");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<ros1_sensor_demo::CustomImu>("/custom_imu", 10);
    ros::Publisher cloud_pub = nh.advertise<ros1_sensor_demo::CustomPointCloud>("/custom_pointcloud", 10);

    ros::Rate rate(100);
    int count = 0;

    ROS_INFO("Sensor publisher node launched, IMU: 100Hz, LiDAR point cloud: 10Hz");

    while (ros::ok())
    {
        ros1_sensor_demo::CustomImu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";

        imu_msg.angular_velocity[0] = 0.01;  // 绕X轴角速度
        imu_msg.angular_velocity[1] = 0.02;  // 绕Y轴角速度
        imu_msg.angular_velocity[2] = 0.5;   // 绕Z轴角速度
        imu_msg.linear_acceleration[0] = 0.0;
        imu_msg.linear_acceleration[1] = 0.0;
        imu_msg.linear_acceleration[2] = 9.81; // 重力加速度
        imu_msg.orientation[0] = 0.0; // 四元数x
        imu_msg.orientation[1] = 0.0; // 四元数y
        imu_msg.orientation[2] = 0.0; // 四元数z
        imu_msg.orientation[3] = 1.0; // 四元数w

        imu_pub.publish(imu_msg);

        if (count % 10 == 0) {
            ros1_sensor_demo::CustomPointCloud cloud_msg;
            cloud_msg.header.stamp = ros::Time::now();
            cloud_msg.header.frame_id = "lidar_link";

            // 模拟5个简单的点
            cloud_msg.point_num = 5;
            cloud_msg.points_x = {0.0, 1.0, 2.0, 3.0, 4.0};
            cloud_msg.points_y = {0.0, 1.0, 0.0, -1.0, 0.0};
            cloud_msg.points_z = {0.0, 0.0, 0.0, 0.0, 0.0};

            cloud_pub.publish(cloud_msg);
            ROS_INFO("Publish one point cloud frame, total points: %d", cloud_msg.point_num);
        }
        count++;
        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}
