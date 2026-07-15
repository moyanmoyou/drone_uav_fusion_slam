# Day6 ROS工程进阶与PCL点云处理基础
## 一、核心知识点
### 1. ROS 工程化工具
- launch 文件：批量启动节点、配置参数、管理命名空间，替代手动逐个开终端
- rosbag：话题录制与离线回放，SLAM 离线调试核心工具
  - 录制：rosbag record -O 包名 话题名
  - 查看信息：rosbag info 包名
  - 回放：rosbag play --clock 包名
- 参数服务器：节点运行时动态读取参数，无需修改源码重新编译

### 2. PCL 点云库基础
PCL（Point Cloud Library）是激光SLAM标准点云处理库，封装了大量滤波、配准、分割算法。
常用点云类型：
- pcl::PointXYZ：仅三维坐标，最基础类型
- pcl::PointXYZRGB：带RGB颜色信息
- pcl::PointXYZI：带强度信息，对应真实激光雷达反射强度

### 3. 激光SLAM标准预处理流水线
1. **直通滤波 PassThrough**：按坐标轴范围裁剪点云，滤除无效距离的点
2. **统计离群点去除 StatisticalOutlierRemoval**：基于邻域统计剔除孤立噪点
3. **体素降采样 VoxelGrid**：均匀栅格压缩点云数量，提升后续配准速度

## 二、实验内容
1. 模拟发布带离群点的标准 PointCloud2 点云
2. 实现三级滤波ROS节点，参数可通过launch动态配置
3. launch一键启动完整系统
4. rosbag录制与离线回放验证
5. RViz可视化滤波前后点云对比

## 三、已知问题
【RViz显示】虚拟机环境下点云颜色渲染异常，始终显示白色，不影响算法功能，实体机可正常显示。