#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include "imu_process.hpp"

#include <queue>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_process");
    ros::NodeHandle nh;

    std::queue<MeasureData> measure_queue;
    MeasureData meas;

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu0", 100, [&](const sensor_msgs::ImuConstPtr & msg)
    {
        double imu_time = msg->header.stamp.toSec();
        if(meas.cloud == nullptr || imu_time <= meas.pcl_end_time){
            auto&& accel_mpss = msg->linear_acceleration;
            auto&& gyro_rps = msg->angular_velocity;
            ImuData imu;
            imu.time = msg->header.stamp.toSec();
            imu.accel_mpss << accel_mpss.x, accel_mpss.y, accel_mpss.z;
            imu.gyro_rps << gyro_rps.x, gyro_rps.y, gyro_rps.z;
            meas.imu_queue.push(imu);
        }
    });

    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar/sensor_pointcloud2", 100, [&](const sensor_msgs::PointCloud2ConstPtr& msg){
        PointCloud::Ptr cloud = boost::make_shared<PointCloud>();
        pcl::fromROSMsg(*msg, *cloud);
        meas.cloud = cloud;
        meas.pcl_beg_time = msg->header.stamp.toSec();
        meas.pcl_end_time = msg->header.stamp.toSec() + 0.1;
        while(meas.imu_queue.front().time < meas.pcl_beg_time){
            meas.imu_queue.pop();
        }
        measure_queue.push(meas); // 将数据打包到队列中
        meas.cloud = nullptr;
    });


    ROS_INFO("imu_process start");

    ModelParam model_param;
    model_param.ARW                  = 0.1 / 60.0 * SO3Math::D2R;    // deg/sqrt(hr)
    model_param.VRW                  = 0.1 / 60.0;          // m/s/sqrt(hr)
    model_param.gyro_bias_std        = 50 * SO3Math::D2R / 3600.0;   // deg/hr
    model_param.gyro_bias_corr_time  = 1 * 3600.0;
    model_param.accel_bias_std       = 50 * 1e-5;           // mGal 1mGal=1e-5Gal
    model_param.accel_bias_corr_time = 1 * 3600.0;

    IMUProcess imu_process(model_param);

    while(ros::ok()){
        if(!measure_queue.empty()){
            // ROS_INFO("meas size: %d, time1: %f, time2: %f", measure_queue.size(), measure_queue.back().imu_queue.front().time, measure_queue.back().pcl_beg_time);
            // ROS_INFO("meas size: %d, time1: %f, time2: %f", measure_queue.size(), measure_queue.back().imu_queue.back().time, measure_queue.back().pcl_end_time);
            auto&& meas = measure_queue.front();
            imu_process.process(meas);
            measure_queue.pop();
        }
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}