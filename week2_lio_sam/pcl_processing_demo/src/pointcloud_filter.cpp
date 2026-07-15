#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloudFilter {
public:
    PointCloudFilter() {
        // 订阅原始点云话题
        cloud_sub_ = nh_.subscribe("/raw_pointcloud", 10, &PointCloudFilter::cloudCallback, this);
        // 发布滤波后的点云
        filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 10);

        // 从参数服务器读取参数，默认值兜底
        nh_.param<double>("voxel_size", voxel_size_, 0.1);
        nh_.param<double>("z_min", z_min_, -2.0);
        nh_.param<double>("z_max", z_max_, 2.0);
        nh_.param<int>("mean_k", mean_k_, 20);
        nh_.param<double>("std_dev", std_dev_, 1.0);

        ROS_INFO("Point cloud filter node started.");
        ROS_INFO("Voxel size: %.2f m, Z range: [%.1f, %.1f]", voxel_size_, z_min_, z_max_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
        // 1. ROS消息转PCL点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_msg, *input_cloud);

        if (input_cloud->empty()) return;

        // 2. 直通滤波：保留Z轴范围内的点，滤除超出范围的无效点
        pcl::PointCloud<pcl::PointXYZ>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(input_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*pass_cloud);

        // 3. 统计离群点去除：基于邻域统计剔除孤立噪点
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(pass_cloud);
        sor.setMeanK(mean_k_);         // 每个点查询邻域点数
        sor.setStddevMulThresh(std_dev_); // 标准差倍数阈值
        sor.filter(*outlier_cloud);

        // 4. 体素降采样：均匀栅格化压缩点云数量
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(outlier_cloud);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*filtered_cloud);

        // 5. 转回ROS标准消息并发布
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header = input_msg->header;
        filtered_pub_.publish(output_msg);

        ROS_INFO("Raw: %zu pts | Filtered: %zu pts", 
                 input_cloud->size(), filtered_cloud->size());
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_pub_;

    double voxel_size_;
    double z_min_, z_max_;
    int mean_k_;
    double std_dev_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_filter");
    PointCloudFilter filter;
    ros::spin();
    return 0;
}