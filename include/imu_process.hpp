/**
 * @file imu_process.hpp
 * @author Linfu Wei (ghowoght@qq.com)
 * @brief IMU状态转移与噪声传播
 * @version 1.0
 * @date 2022-07-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef IMU_PROCESS_HPP
#define IMU_PROCESS_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <queue>

#include "so3_math.hpp"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Quaterniond QD;

struct StateEKF{
    double time;
    QD rot;
    V3D pos;
    V3D vel;
    V3D bg;
    V3D ba;
    V3D grav;
    StateEKF(){
        time = 0;
        rot = QD::Identity();
        pos = V3D::Zero();
        vel = V3D::Zero();
        bg = V3D::Zero();
        ba = V3D::Zero();
        grav = V3D::Zero();
    }
    StateEKF(const StateEKF& s){
        time = s.time;
        rot = s.rot;
        pos = s.pos;
        vel = s.vel;
        bg = s.bg;
        ba = s.ba;
        grav = s.grav;
    }
    void operator=(const StateEKF& other){
        time = other.time;
        rot = other.rot;
        pos = other.pos;
        vel = other.vel;
        bg = other.bg;
        ba = other.ba;
        grav = other.grav;
    }
};

struct ImuData{
    double time;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    ImuData(){
        acc = Eigen::Vector3d::Zero();
        gyro = Eigen::Vector3d::Zero();
        time = 0;
    }
    ImuData(double time_, Eigen::Vector3d acc_, Eigen::Vector3d gyro_){
        time = time_;
        acc = acc_;
        gyro = gyro_;
    }
    ImuData(const ImuData& other){
        acc = other.acc;
        gyro = other.gyro;
        time = other.time;
    }
};

// IMU 和 点云 数据打包
struct MeasureData{
    std::queue<ImuData> imu_queue;
    PointCloud::Ptr cloud;
    double pcl_beg_time;
    double pcl_end_time;
    
};

class IMUProcess{
private:
    Eigen::Matrix<double, 18, 1> delta_x_; // p v phi bg ba g
    Eigen::Matrix<double, 18, 18> P_;
    StateEKF state_last_;
    std::vector<StateEKF> state_queue_;
    bool is_initialized_ = false;

private:
    void forward_propagation(MeasureData& measure){
        if(state_queue_.empty()){
            state_queue_.push_back(state_last_);
        }
        auto size = measure.imu_queue.size();
        for(int i = 0; i < size; i++){
            auto imu = measure.imu_queue.front();
            measure.imu_queue.pop();
            StateEKF state_curr = state_queue_.back();

            double dt = imu.time - state_curr.time;
            state_curr.time = imu.time;
            ROS_INFO("dt = %f", dt);
            // 姿态更新
            M3D R = state_curr.rot.toRotationMatrix() * SO3Math::Exp(imu.gyro * dt);
            state_curr.rot = QD(R);            
            // 速度更新
            state_curr.vel += (state_curr.rot * imu.acc - state_curr.grav) * dt;
            // 位置更新
            state_curr.pos += state_curr.vel * dt;

            state_queue_.push_back(state_curr);

            
        }

        if(1){
            state_last_ = state_queue_.back();
            state_queue_.clear();

            auto euler = SO3Math::quat2euler(state_last_.rot);
            euler = euler * 180 / M_PI;
            std::cout << "time: " << state_last_.time << " euler: " << euler.transpose()<< std::endl;
        }
    }
    void backward_propagation();

public:
    void point_cloud_undistort();
    void process(MeasureData& measure){
        if(!is_initialized_){
            is_initialized_ = true;
            state_last_ = StateEKF();
            state_last_.time = measure.imu_queue.back().time;
            state_queue_.clear();
            return;
        }

        // 前向传播
        forward_propagation(measure);
    }

};

#endif // IMU_PROCESS_HPP