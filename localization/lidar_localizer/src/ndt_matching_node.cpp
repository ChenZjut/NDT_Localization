/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/12/10 下午4:34
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/12/10 下午4:34
 * @Version 1.0
 */
#include <lidar_localizer/ndt_matching.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_matching");
    NDTMatching::NDTMatchingNode app;
    app.initForROS();
    ros::spin();
    return 0;
}