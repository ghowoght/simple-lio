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
#include "scan_registration.hpp"

#define N 18

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
    Eigen::Vector3d accel_mpss;
    Eigen::Vector3d gyro_rps;
    ImuData(){
        accel_mpss = Eigen::Vector3d::Zero();
        gyro_rps = Eigen::Vector3d::Zero();
        time = 0;
    }
    ImuData(double time_, Eigen::Vector3d accel_mpss_, Eigen::Vector3d gyro_rps_){
        time = time_;
        accel_mpss = accel_mpss_;
        gyro_rps = gyro_rps_;
    }
    ImuData(const ImuData& other){
        accel_mpss = other.accel_mpss;
        gyro_rps = other.gyro_rps;
        time = other.time;
    }
};

// IMU 和 点云 数据打包
struct MeasureData{
    std::vector<ImuData> imu_queue;
    PointCloud::Ptr cloud;
    double pcl_beg_time;
    double pcl_end_time;
    
};

struct ModelParam{
    double ARW;                             // 角度随机游走
    double VRW;                             // 速度随机游走
    double gyro_bias_std;                   // 陀螺仪零偏标准差
    double gyro_bias_corr_time;             // 陀螺仪零偏相关时间
    double accel_bias_std;                  // 加速度计零偏标准差
    double accel_bias_corr_time;            // 加速度计零偏相关时间
};

class IMUProcess{
private:

    StateEKF state_last_;
    std::vector<StateEKF> state_queue_;
    std::vector<StateEKF> state_bp_queue_;
    bool is_initialized_ = false;
    ModelParam model_param_;

    // kalman相关
    Eigen::Matrix<double, N, 1> delta_x_; // p v phi bg ba g
    Eigen::Matrix<double, N, N> Pmat_;

    Eigen::Matrix<double, N, 12> Gmat_; // 噪声输入映射矩阵 noise-input mapping matrix
    Eigen::Matrix<double, N, 12> Gmat_last_;

public:
    IMUProcess()=default;
    IMUProcess(const ModelParam& model_param) : model_param_(model_param){}

private:
    void forward_propagation(MeasureData& measure){
        if(state_queue_.empty()){
            state_queue_.push_back(state_last_);
        }
        auto size = measure.imu_queue.size();
        for(int i = 0; i < size; i++){
            auto imu = measure.imu_queue[i];
            StateEKF state_curr = state_queue_.back();

            double dt = imu.time - state_curr.time;
            state_curr.time = imu.time;

            //////////////// 机械编排 ////////////////
            // 姿态更新
            M3D R = state_curr.rot.toRotationMatrix() * SO3Math::Exp(imu.gyro_rps * dt);
            state_curr.rot = QD(R);            
            // 速度更新
            state_curr.vel += (state_curr.rot * imu.accel_mpss - state_curr.grav) * dt;
            // 位置更新
            state_curr.pos += state_curr.vel * dt;

            state_queue_.push_back(state_curr);

            //////////////// 噪声传播 ////////////////
            auto I_33 = Eigen::Matrix3d::Identity();
            // 计算状态转移矩阵Φ
            Eigen::Matrix<double, N, N> PHImat = Eigen::Matrix<double, N, N>::Identity();
            // p
            PHImat.block<3, 3>(0,  3) = I_33 * dt;  
            // v
            PHImat.block<3, 3>(3,  6) = SO3Math::get_skew_symmetric(state_curr.rot * imu.accel_mpss) * dt; 
            PHImat.block<3, 3>(3, 12) = state_curr.rot.toRotationMatrix() * dt;
            PHImat.block<3, 3>(3, 15) = I_33 * dt;
            // phi
            PHImat.block<3, 3>(6,  6) = I_33 - SO3Math::get_skew_symmetric(imu.gyro_rps) * dt; // SO3Math::Exp(-imu.gyro_rps * dt)
            PHImat.block<3, 3>(6,  9) = -state_curr.rot.toRotationMatrix() * dt;

            // 计算状态转移噪声协方差矩阵Q
            Eigen::Matrix<double, 12, 12> qmat = Eigen::Matrix<double, 12, 12>::Zero();
            double item[] = {   model_param_.VRW * model_param_.VRW,
                                model_param_.ARW * model_param_.ARW,
                                2 * model_param_.gyro_bias_std * model_param_.gyro_bias_std / model_param_.gyro_bias_corr_time,
                                2 * model_param_.accel_bias_std * model_param_.accel_bias_std / model_param_.accel_bias_corr_time,
                            };
            Eigen::Matrix3d mat[4];
            for(int i = 0; i < 4; i++){
                mat[i] = Eigen::Vector3d(item[i], item[i], item[i]).asDiagonal();
                qmat.block<3, 3>(3 * i,  3 * i) = mat[i];
            }
            Gmat_ = Eigen::Matrix<double, N, 12>::Zero();
            Gmat_.block<3, 3>( 3, 0) = state_curr.rot.toRotationMatrix();
            Gmat_.block<3, 3>( 6, 3) = state_curr.rot.toRotationMatrix();
            Gmat_.block<3, 3>( 9, 6) = I_33;
            Gmat_.block<3, 3>(12, 9) = I_33;
            // 梯形积分
            auto Qmat = 0.5 * (PHImat * Gmat_last_ * qmat * Gmat_last_.transpose() * PHImat.transpose()
                        + Gmat_ * qmat * Gmat_.transpose()) * dt;
            Gmat_last_ = Gmat_;

            // 状态转移
            delta_x_ = PHImat * delta_x_;
            Pmat_ = PHImat * Pmat_ * PHImat.transpose() + Qmat;
        }

        
    }
    void backward_propagation(MeasureData& measure){
        auto size = measure.imu_queue.size();

        StateEKF state_curr;
        state_curr.time = measure.pcl_end_time;
        state_bp_queue_.clear();
        state_bp_queue_.push_back(state_curr);

        
        for(int i = 0; i < size; i++){
            auto imu = measure.imu_queue[size - 1 - i];

            double dt = state_curr.time - imu.time;
            // 姿态更新
            M3D R = state_curr.rot.toRotationMatrix() * SO3Math::Exp(-imu.gyro_rps * dt);
            state_curr.rot = QD(R);            
            // 速度更新
            state_curr.vel -= (state_curr.rot * imu.accel_mpss - state_curr.grav) * dt;
            // 位置更新
            state_curr.pos -= state_curr.vel * dt;

            state_bp_queue_.push_back(state_curr);
        }
        state_bp_queue_.push_back(state_last_);
    }

    void point_cloud_undistort(MeasureData& measure){
        // 根据每一点的
        auto ptr = measure.cloud;
        auto&& cloud = *ptr;

        // 特征提取
        ScanRegistration scan_registration;
        scan_registration.feature_extract(cloud);

        auto& cloud_surf = scan_registration.pointSurf;

        // 将每一点投影到扫描结束时刻的惯导坐标系下    
        for(int i = 0; i < cloud_surf.size(); i++){
            // std::cout << "curvature: " << cloud_surf[i].curvature << std::endl;

        }
    }

public:

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

        // 反向传播
        backward_propagation(measure);

        // 点云运动补偿
        point_cloud_undistort(measure);

        if(1){
            state_last_ = state_queue_.back();
            state_queue_.clear();

            auto euler = SO3Math::quat2euler(state_last_.rot);
            euler = euler * 180 / M_PI;
            // std::cout << "time: " << state_last_.time << " euler: " << euler.transpose()<< std::endl;
        }
    }

};

#endif // IMU_PROCESS_HPP