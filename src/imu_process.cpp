#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <queue>
#include <thread>
#include <chrono>

#include "imu_process.hpp"
#include "tictoc.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_process");
    ros::NodeHandle nh;

    std::queue<MeasureData> measure_queue;
    std::queue<sensor_msgs::ImuConstPtr> imu_queue;
    std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue;

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu0", 100, [&](const sensor_msgs::ImuConstPtr & msg)
    {   
        imu_queue.push(msg);

        if(cloud_queue.empty()) return;

        auto pcl_msg = cloud_queue.front();
        auto pcl_beg_time = pcl_msg->header.stamp.toSec();
        auto pcl_end_time = pcl_msg->header.stamp.toSec() + 0.1;
        while(!cloud_queue.empty() 
            && !imu_queue.empty()
            && imu_queue.front()->header.stamp.toSec() > pcl_end_time) // imu时刻比当前帧点云晚，则丢弃该帧点云
        {
            cloud_queue.pop();
            pcl_msg = cloud_queue.front();
            pcl_beg_time = pcl_msg->header.stamp.toSec();
            pcl_end_time = pcl_msg->header.stamp.toSec() + 0.1;
        }

        if(!cloud_queue.empty() 
            && !imu_queue.empty()
            && imu_queue.back()->header.stamp.toSec() > pcl_end_time)
        {
            cloud_queue.pop();            
            auto imu_msg = imu_queue.front();
            auto imu_time = imu_msg->header.stamp.toSec();
            MeasureData meas;
            while(imu_time <= pcl_end_time)
            {   
                imu_queue.pop();

                if(imu_time < pcl_beg_time){
                    imu_msg = imu_queue.front();
                    imu_time = imu_msg->header.stamp.toSec();
                    continue;
                }
                
                auto&& accel_mpss = imu_msg->linear_acceleration;
                auto&& gyro_rps = imu_msg->angular_velocity;
                ImuData imu;
                imu.time = imu_msg->header.stamp.toSec();
                imu.accel_mpss << accel_mpss.x, accel_mpss.y, accel_mpss.z;
                imu.gyro_rps   << gyro_rps.x, gyro_rps.y, gyro_rps.z;
                meas.imu_queue.push_back(imu);

                imu_msg = imu_queue.front();
                imu_time = imu_msg->header.stamp.toSec();
            }
            meas.pcl_beg_time = pcl_beg_time;
            meas.pcl_end_time = pcl_end_time;
            pcl::fromROSMsg(*pcl_msg, *meas.cloud);
            // 将数据打包到队列中
            measure_queue.push(meas); 
            meas.cloud = nullptr;
            meas.imu_queue.clear();
        }
    });

    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar/sensor_pointcloud2", 100, [&](const sensor_msgs::PointCloud2ConstPtr& msg){
        cloud_queue.push(msg);
    });

    ROS_INFO("imu_process start");

    ModelParam model_param;
    // model_param.ARW                  = 0.1 / 60.0 * SO3Math::D2R;    // deg/sqrt(hr)
    // model_param.VRW                  = 0.1 / 60.0;          // m/s/sqrt(hr)
    // model_param.gyro_bias_std        = 50 * SO3Math::D2R / 3600.0;   // deg/hr
    // model_param.gyro_bias_corr_time  = 1 * 3600.0;
    // model_param.accel_bias_std       = 50 * 1e-5;           // mGal 1mGal=1e-5Gal
    // model_param.accel_bias_corr_time = 1 * 3600.0;
 
    model_param.ARW                  = 0.01; 
    model_param.VRW                  = 0.1;  
    model_param.gyro_bias_std        = 0.001; 
    model_param.gyro_bias_corr_time  = 1;
    model_param.accel_bias_std       = 0.01;      
    model_param.accel_bias_corr_time = 1;

    model_param.R = 0.0025;

    model_param.init_t_l_i(-0.09565903, -0.03466711, 0.0407548);
    M3D R_L_I;
    R_L_I << 0.99921203,  0.03962574, -0.00226486,
             0.03961853, -0.99920993, -0.00314573,
            -0.00238773,  0.00305353, -0.99999249;
    model_param.init_r_l_i(R_L_I);
    IMUProcess imu_process(model_param, nh);

    while(ros::ok()){
        if(!measure_queue.empty()){
            auto& meas = measure_queue.front();
            // ROS_INFO("meas size: %d, imu_beg_time: %f, pcl_beg_time: %f", measure_queue.size(), meas.imu_queue.front().time, meas.pcl_beg_time);
            // ROS_INFO("meas size: %d, imu_end_time: %f, pcl_end_time: %f", measure_queue.size(), meas.imu_queue.back().time, meas.pcl_end_time);
            // ROS_INFO("--------------------");
            TicToc t_process;
            imu_process.process(meas);
            ROS_INFO("total process time: %f", t_process.toc());
            ROS_INFO("--------------------");
            measure_queue.pop();
        }
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}