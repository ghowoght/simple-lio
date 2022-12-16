/**
 * @file CustomMsg2SensorMsg.cpp
 * @brief 
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2021-10-03
 * 
 * @copyright Copyright (c) 2021  WHU-EIS
 * 
 */
#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// 先将livox_ros_driver::CustomMs转换为pcl::PCLPointCloud2，再转换为sensor_msgs::PointCloud2

int main(int argc, char** argv){
    ros::init(argc, argv, "CustomMsg2SensorMsg");

    ros::NodeHandle nh;

    ros::Publisher pcl_msgs_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar/sensor_pointcloud2", 10);

    uint8_t TO_MERGE_CNT = 1;
    std::vector<livox_ros_driver::CustomMsgConstPtr> livox_datas; // 消息缓存，到达一定数量时合并成一个PCL消息
    ros::Subscriber custom_msgs_sub = nh.subscribe<livox_ros_driver::CustomMsg>(
                "/livox/lidar", 
                10, 
                [&](const livox_ros_driver::CustomMsgConstPtr& msgs){
        livox_datas.push_back(msgs);

        if(livox_datas.size() < TO_MERGE_CNT) // 取<TO_MERGE_CNT>帧进行处理
            return; 

        PointCloudXYZI points_pcl;
        // 合并点云: 将多帧合并为一帧
        for(size_t j = 0; j < livox_datas.size(); j++){
            auto& livox_msgs = livox_datas[j];
            auto time_end = livox_msgs->points.back().offset_time;
            for(size_t i = 0; i < livox_msgs->point_num; i++){
                PointType pt;
                pt.x = livox_msgs->points[i].x;
                pt.y = livox_msgs->points[i].y;
                pt.z = livox_msgs->points[i].z;
                pt.intensity    = livox_msgs->points[i].line // 整数是线数
                                + livox_msgs->points[i].reflectivity / 10000.0; // ??? 小数是反射率?
                pt.curvature = (livox_msgs->points[i].offset_time / (float)time_end); // 该点时刻在该次扫描时间段的位置
                points_pcl.push_back(pt);
            }
        }        

        uint64_t timebase_ns = livox_datas[0]->timebase;
        ros::Time timestamp;
        timestamp.fromNSec(timebase_ns);

        sensor_msgs::PointCloud2 pcl_msgs;
        pcl::toROSMsg(points_pcl, pcl_msgs);
        pcl_msgs.header.stamp = timestamp;
        pcl_msgs.header.frame_id = "livox";

        pcl_msgs_pub.publish(pcl_msgs);
        livox_datas.clear(); // 清空缓存区
    });

    ros::spin();
}