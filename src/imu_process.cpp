#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <queue>
#include <thread>
#include <chrono>
#include <future>
#include <mutex>

#include "imu_process.hpp"
#include "tictoc.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_process");
    ros::NodeHandle nh;

    ros::NodeHandle private_nh("~");
    std::string config_file;
    private_nh.param("config_file", config_file, std::string(""));
    ROS_INFO("config_file: %s", config_file.c_str());
    YAML::Node config = YAML::LoadFile(config_file);

    std::string imu_topic = config["topic"]["imu"].as<std::string>();

    std::queue<sensor_msgs::ImuConstPtr> imu_queue;
    std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue;
    std::queue<MeasureData> measure_queue;
    std::mutex measure_queue_mutex;

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 100, [&](const sensor_msgs::ImuConstPtr & msg)
    {   
        imu_queue.push(msg);

        if(cloud_queue.empty()) return;

        auto pcl_beg_time = cloud_queue.front()->header.stamp.toSec();
        auto pcl_end_time = cloud_queue.front()->header.stamp.toSec() + 0.1;
        while(!cloud_queue.empty() 
            && !imu_queue.empty())
        {
            pcl_beg_time = cloud_queue.front()->header.stamp.toSec();
            pcl_end_time = cloud_queue.front()->header.stamp.toSec() + 0.1;
            if(imu_queue.front()->header.stamp.toSec() < pcl_end_time){
                break;
            }
            cloud_queue.pop();
        }

        if(!cloud_queue.empty() 
            && !imu_queue.empty()
            && imu_queue.back()->header.stamp.toSec() > pcl_end_time)
        {
            auto pcl_msg = cloud_queue.front();
            cloud_queue.pop();     
            MeasureData meas;
            while(true){   
                const auto imu_msg = imu_queue.front();
                auto imu_time = imu_msg->header.stamp.toSec();
                imu_queue.pop();
                
                if(imu_time < pcl_beg_time) continue;
                if(imu_time > pcl_end_time) break;
                
                auto& accel_mpss = imu_msg->linear_acceleration;
                auto& gyro_rps = imu_msg->angular_velocity;
                ImuData imu;
                imu.time = imu_msg->header.stamp.toSec();
                imu.accel_mpss << accel_mpss.x, accel_mpss.y, accel_mpss.z;
                imu.accel_mpss *= 9.7936; // LiLi-OM数据
                imu.gyro_rps   << gyro_rps.x, gyro_rps.y, gyro_rps.z;
                meas.imu_queue.push_back(imu);

            }
            meas.pcl_beg_time = pcl_beg_time;
            meas.pcl_end_time = pcl_end_time;
            pcl::fromROSMsg(*pcl_msg, *meas.cloud);
            // 将数据打包到队列中
            std::unique_lock<std::mutex> lock(measure_queue_mutex);
            measure_queue.push(meas); 
        }
    });

    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar/sensor_pointcloud2", 100, [&](const sensor_msgs::PointCloud2ConstPtr& msg){
        cloud_queue.push(msg);
    });

    ROS_INFO("imu_process start");

    ModelParam model_param;

    // 读取imu & lidar参数
    model_param.ARW                  = config["imu_params"]["ARW"].as<double>() / 60.0 * SO3Math::D2R;              // deg/sqrt(hr)
    model_param.VRW                  = config["imu_params"]["VRW"].as<double>() / 60.0;                             // m/s/sqrt(hr)  
    model_param.gyro_bias_std        = config["imu_params"]["gyro_bias_std"].as<double>() * SO3Math::D2R / 3600.0;  // deg/hr 
    model_param.gyro_bias_corr_time  = config["imu_params"]["gyro_bias_corr_time"].as<double>() * 3600.0;  
    model_param.accel_bias_std       = config["imu_params"]["accel_bias_std"].as<double>() * 1e-5;                  // mGal 1mGal=1e-3Gal=1e-5m/s^2
    model_param.accel_bias_corr_time = config["imu_params"]["accel_bias_corr_time"].as<double>() * 3600.0;

    model_param.R                    = config["lidar_params"]["R_plane"].as<double>();

    // 读取外参
    std::vector<double> r_l_i = config["lidar_params"]["r_l_i"].as<std::vector<double>>();
    if(r_l_i.size() == 4) {
        // 四元数
        QD q_l_i(r_l_i[0], r_l_i[1], r_l_i[2], r_l_i[3]);
        model_param.init_r_l_i(q_l_i);
    } else if(r_l_i.size() == 9) {
        // 旋转矩阵
        M3D R_L_I;
        R_L_I << r_l_i[0], r_l_i[1], r_l_i[2],
                 r_l_i[3], r_l_i[4], r_l_i[5],
                 r_l_i[6], r_l_i[7], r_l_i[8];
        model_param.init_r_l_i(R_L_I);
    } else {
        ROS_ERROR("r_l_i size error");
    }
    std::vector<double> t_l_i = config["lidar_params"]["t_l_i"].as<std::vector<double>>();
    if(t_l_i.size() == 3) {
        model_param.init_t_l_i(t_l_i[0], t_l_i[1], t_l_i[2]);
    } else {
        ROS_ERROR("t_l_i size error");
    }

    IMUProcess imu_process(model_param, private_nh);

    std::future<void> imu_process_future = std::async(std::launch::async, [&](){
        ros::spin();
    });

    while(ros::ok()){
        if(!measure_queue.empty()){
            std::unique_lock<std::mutex> lock(measure_queue_mutex);
            auto meas = measure_queue.front();
            measure_queue.pop();
            lock.unlock();
            // ROS_INFO("meas size: %d, imu_beg_time: %f, pcl_beg_time: %f", measure_queue.size(), meas.imu_queue.front().time, meas.pcl_beg_time);
            // ROS_INFO("meas size: %d, imu_end_time: %f, pcl_end_time: %f", measure_queue.size(), meas.imu_queue.back().time, meas.pcl_end_time);
            // ROS_INFO("--------------------");
            TicToc t_process;
            imu_process.process(meas);
            ROS_INFO("measure_queue size: %d --- total process time: %f", measure_queue.size(), t_process.toc());
            ROS_INFO("--------------------");
        }
        else{
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}