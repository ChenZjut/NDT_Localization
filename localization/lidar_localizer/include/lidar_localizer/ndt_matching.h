/*
 * @Description:
 * @Author: ubuntu
 * @Date: 2021/12/10 下午4:03
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/12/10 下午4:03
 * @Version 1.0
 */
#ifndef SRC_NDT_MATCHING_H
#define SRC_NDT_MATCHING_H

#include <pthread.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif
#ifdef USE_PCL_OPENMP
#include <pcl_omp_registration/ndt.h>
#endif

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <autoware_msgs/NDTStat.h>

using namespace std;


#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

namespace NDTMatching
{
    class NDTMatchingNode
    {
    public:
        struct pose
        {
            double x;
            double y;
            double z;
            double roll;
            double pitch;
            double yaw;
        };

        enum class MethodType
        {
            PCL_GENERIC = 0,
            PCL_ANH = 1,
            PCL_ANH_GPU = 2,
            PCL_OPENMP = 3,
        };

        NDTMatchingNode();
        ~NDTMatchingNode();

        void initForROS();
    private:
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber gnss_sub;
        ros::Subscriber initialpose_sub;
        ros::Subscriber points_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber imu_sub;
        static NDTMatchingNode::pose convertPoseIntoRelativeCoordinate(const pose& target_pose, const pose& reference_pose);

        static void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
        static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input);
        static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& input);
        static void imu_callback(const sensor_msgs::Imu::Ptr& input);
        static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
        static void odom_callback(const nav_msgs::OdometryConstPtr& input);

        static void imu_odom_calc(ros::Time current_time);
        static void odom_calc(ros::Time current_time);
        static void imu_calc(ros::Time current_time);
        static double wrapToPm(double a_num, const double a_max);
        static double wrapToPmPi(const double a_angle_rad);
        static double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
        static void imuUpsideDown(const sensor_msgs::Imu::Ptr input);
        static void* thread_func(void* args);
    };
}
#endif //SRC_NDT_MATCHING_H
