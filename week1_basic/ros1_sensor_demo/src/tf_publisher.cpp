#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "tf_publisher");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped imu_tf;
    imu_tf.header.stamp = ros::Time::now();
    imu_tf.header.frame_id = "base_link";    // 父坐标系：机体中心
    imu_tf.child_frame_id = "imu_link";      // 子坐标系：IMU传感器

    // 平移：IMU安装在机体中心正上方0.1米处
    imu_tf.transform.translation.x = 0.0;
    imu_tf.transform.translation.y = 0.0;
    imu_tf.transform.translation.z = 0.1;

    // 旋转：与机体完全同轴，无角度偏差（单位四元数）
    imu_tf.transform.rotation.x = 0.0;
    imu_tf.transform.rotation.y = 0.0;
    imu_tf.transform.rotation.z = 0.0;
    imu_tf.transform.rotation.w = 1.0;

    // ========== 变换2：base_link -> lidar_link ==========
    geometry_msgs::TransformStamped lidar_tf;
    lidar_tf.header.stamp = ros::Time::now();
    lidar_tf.header.frame_id = "base_link";  // 父坐标系：机体中心
    lidar_tf.child_frame_id = "lidar_link";  // 子坐标系：激光雷达

    // 平移：激光雷达安装在机体中心前方0.2米、正上方0.15米处
    lidar_tf.transform.translation.x = 0.2;
    lidar_tf.transform.translation.y = 0.0;
    lidar_tf.transform.translation.z = 0.15;

    // 旋转：激光雷达绕Z轴无偏转，与机体前向一致
    lidar_tf.transform.rotation.x = 0.0;
    lidar_tf.transform.rotation.y = 0.0;
    lidar_tf.transform.rotation.z = 0.0;
    lidar_tf.transform.rotation.w = 1.0;

    // 广播两个静态TF变换
    static_broadcaster.sendTransform(imu_tf);
    static_broadcaster.sendTransform(lidar_tf);

    ROS_INFO("Static TF published: base_link -> imu_link, base_link -> lidar_link");

    ros::spin();

    return 0;
}
