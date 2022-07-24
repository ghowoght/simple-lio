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

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Quaterniond QD;

struct StateEKF{
    QD rot;
    V3D pos;
    V3D vel;
    V3D bg;
    V3D ba;
    V3D grav;
    StateEKF(){
        rot = QD::Identity();
        pos = V3D::Zero();
        vel = V3D::Zero();
        bg = V3D::Zero();
        ba = V3D::Zero();
        grav = V3D::Zero();
    }
    StateEKF(const StateEKF& s){
        rot = s.rot;
        pos = s.pos;
        vel = s.vel;
        bg = s.bg;
        ba = s.ba;
        grav = s.grav;
    }
    void operator=(const StateEKF& other){
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
    

private:
    void forward_propagation();
    void backward_propagation();

public:
    void point_cloud_undistort();
    void process(const MeasureData& measure);

};

#endif // IMU_PROCESS_HPP