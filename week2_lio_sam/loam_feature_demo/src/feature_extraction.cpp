#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <cmath>

class FeatureExtraction {
public:
    FeatureExtraction() {
        cloud_sub_ = nh_.subscribe("/structured_cloud", 10, &FeatureExtraction::callback, this);
        planar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/planar_points", 10);
        corner_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corner_points", 10);

        nh_.param<int>("neighbor_k", neighbor_k_, 20);
        nh_.param<double>("corner_thresh", corner_thresh_, 0.15);
        nh_.param<double>("planar_thresh", planar_thresh_, 0.05);

        ROS_INFO("LOAM feature extraction node started.");
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*input_msg, *input_cloud);
        if (input_cloud->empty()) return;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 构建KD树用于邻域查询
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(input_cloud);

        for (size_t i = 0; i < input_cloud->size(); ++i) {
            std::vector<int> indices;
            std::vector<float> dists;
            kdtree.nearestKSearch(input_cloud->points[i], neighbor_k_, indices, dists);

            // 计算邻域Z坐标的标准差（粗糙度）
            double sum_z = 0, sum_z2 = 0;
            for (int idx : indices) {
                double z = input_cloud->points[idx].z;
                sum_z += z;
                sum_z2 += z*z;
            }
            double mean_z = sum_z / indices.size();
            double std_z = sqrt(sum_z2/indices.size() - mean_z*mean_z);

            pcl::PointXYZRGB pt = input_cloud->points[i];
            if (std_z > corner_thresh_) {
                // 角点：标记为红色
                pt.r = 255; pt.g = 0; pt.b = 0;
                corner_cloud->points.push_back(pt);
            } else if (std_z < planar_thresh_) {
                // 平面点：标记为绿色
                pt.r = 0; pt.g = 255; pt.b = 0;
                planar_cloud->points.push_back(pt);
            }
        }

        // 发布两类特征点云
        sensor_msgs::PointCloud2 planar_msg, corner_msg;
        pcl::toROSMsg(*planar_cloud, planar_msg);
        pcl::toROSMsg(*corner_cloud, corner_msg);
        planar_msg.header = input_msg->header;
        corner_msg.header = input_msg->header;

        planar_pub_.publish(planar_msg);
        corner_pub_.publish(corner_msg);

        ROS_INFO("Planar: %zu pts | Corner: %zu pts", 
                 planar_cloud->size(), corner_cloud->size());
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher planar_pub_, corner_pub_;
    int neighbor_k_;
    double corner_thresh_, planar_thresh_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "feature_extraction");
    FeatureExtraction fe;
    ros::spin();
    return 0;
}