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

#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include <vector>
#include <queue>

#include "so3_math.hpp"
#include "tictoc.hpp"
#include "file_helper.hpp"
#include "ikd-Tree/ikd_Tree.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#define N 18 // 状态维度

using PointType = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<PointType>;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using QD = Eigen::Quaterniond;

using PointVector = KD_TREE<PointType>::PointVector;
template class KD_TREE<PointType>;

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

    inline std::string to_string() const{
        // 将时间转换为周内秒
        int GPS_LEAP_SECOND = 18; 
        double second_gps = time + GPS_LEAP_SECOND - 315964800;
        int week = floor(second_gps / 604800);
        double sow = second_gps - week * 604800;
        return fmt::format("{} {} {} {} {} {} {} {}\n", 
                        sow,
                        pos.x(), pos.y(), pos.z(),
                        rot.x(), rot.y(), rot.z(), rot.w()
                        );
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
    MeasureData(){
        pcl_beg_time = 0;
        pcl_end_time = 0;
        cloud = PointCloud::Ptr(new PointCloud);
    }
};

struct ModelParam{
    double ARW;                             // 角度随机游走
    double VRW;                             // 速度随机游走
    double gyro_bias_std;                   // 陀螺仪零偏标准差
    double gyro_bias_corr_time;             // 陀螺仪零偏相关时间
    double accel_bias_std;                  // 加速度计零偏标准差
    double accel_bias_corr_time;            // 加速度计零偏相关时间
    double R;                               // 激光雷达协方差
    QD R_L_I = QD::Identity();              // 外参: IMU坐标系到点云坐标系的旋转
    V3D T_L_I;                              // 外参: IMU坐标系到点云坐标系的变换
    void init_r_l_i(double roll, double pitch, double yaw){
        R_L_I.x()=sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2);
        R_L_I.y()=sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2);
        R_L_I.z()=cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2);
        R_L_I.w()=cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2);
    }
    void init_r_l_i(M3D& R_L_I_){
        R_L_I = QD(R_L_I_);
    }
    void init_t_l_i(double x, double y, double z){
        T_L_I(0) = x;
        T_L_I(1) = y;
        T_L_I(2) = z;
    }
};

class IMUProcess{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_surf_map;
    ros::Publisher pub_surf_cloud;
    ros::Publisher pub_traj;
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Path traj_local_msgs;

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

    Eigen::Matrix<double, Eigen::Dynamic, 1> delta_z_;
    Eigen::Matrix<double, Eigen::Dynamic, N> Hmat_;
    Eigen::Matrix<double, N, Eigen::Dynamic> Kmat_;

    // 最大迭代次数
    int max_iter_times_ = 10; 

    // 单帧降采样参数
    pcl::VoxelGrid<PointType> surf_frame_ds_filter_;
    double surf_frame_ds_res_ = 0.5;

    // 地图相关参数
    std::vector<PointCloud::Ptr> surf_cloud_queue_; // 用于初始化
    PointCloud::Ptr map_surf_cloud_;        // 用于初始化
    double map_res_ = 0.5;                  // 地图分辨率
    std::vector<PointVector> nearest_list_; // 当前帧每一个点的最近邻搜索结果
    bool is_surf_list_[100000] = {false};   // 一帧最多10万点
    int KNN_MATCH_NUM = 5;                  // K近邻匹配点数
    KD_TREE<PointType>::Ptr ikd_tree_;

    // 定位结果
    std::shared_ptr<spdlog::logger> logger_;
    FileWriterPtr trajctory_writer_;



public:
    IMUProcess()=default;
    IMUProcess(const ModelParam& model_param, ros::NodeHandle& nh) : model_param_(model_param), nh_(nh){

        pub_surf_map    = nh_.advertise<sensor_msgs::PointCloud2>("/surf_map", 1);
        pub_surf_cloud  = nh_.advertise<sensor_msgs::PointCloud2>("/surf_cloud", 1);
        pub_traj        = nh_.advertise<nav_msgs::Path>("/traj/local", 10);

        // logger_ = spdlog::basic_logger_st("trajctory", "/home/ghowoght/workspace/lidar_ws/src/simple_lio/logger.txt");

        // trajctory_writer_ = FileWriter::create("/home/ghowoght/workspace/lidar_ws2/src/simple_lio/result/trajctory.txt");

        surf_frame_ds_filter_.setLeafSize(surf_frame_ds_res_, surf_frame_ds_res_, surf_frame_ds_res_);

        map_surf_cloud_ = boost::make_shared<PointCloud>();
        ikd_tree_ = std::make_shared<KD_TREE<PointType>>(); // delete_param, balance_param, box_length
        ikd_tree_->set_downsample_param(map_res_);
        
        state_last_ = StateEKF();
        Pmat_ = Eigen::Matrix<double, N, N>::Identity();
        Pmat_(6,6) = Pmat_(7,7) = Pmat_(8,8) = 0.00001;
        Pmat_(9,9) = Pmat_(10,10) = Pmat_(11,11) = 0.00001;
        Pmat_(15,15) = Pmat_(16,16) = Pmat_(17,17) = 0.00001;
    }
    ~IMUProcess(){
        spdlog::drop_all();
    }

private:
    void forward_propagation(MeasureData& measure){
        state_queue_.clear();
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
            state_curr.rot = state_curr.rot * SO3Math::Exp(imu.gyro_rps * dt);
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
            PHImat.block<3, 3>(3,  6) = -SO3Math::get_skew_symmetric(state_curr.rot * imu.accel_mpss) * dt; 
            PHImat.block<3, 3>(3, 12) = -state_curr.rot.toRotationMatrix() * dt;
            PHImat.block<3, 3>(3, 15) = I_33 * dt;
            // phi
            // PHImat.block<3, 3>(6,  6) = SO3Math::Exp(-imu.gyro_rps * dt);
            // PHImat.block<3, 3>(6,  9) = -SO3Math::J_l(imu.gyro_rps * dt).transpose() * dt;
            PHImat.block<3, 3>(6,  6) = I_33 + SO3Math::get_skew_symmetric(-imu.gyro_rps * dt);  // 近似
            PHImat.block<3, 3>(6,  9) = -I_33 * dt;                                              // 近似

            // 计算状态转移噪声协方差矩阵Q
            Eigen::Matrix<double, 12, 12> qmat = Eigen::Matrix<double, 12, 12>::Zero();
            double item[] = {   model_param_.VRW * model_param_.VRW,
                                model_param_.ARW * model_param_.ARW,
                                2 * model_param_.gyro_bias_std * model_param_.gyro_bias_std / model_param_.gyro_bias_corr_time,
                                2 * model_param_.accel_bias_std * model_param_.accel_bias_std / model_param_.accel_bias_corr_time,
                            };
            for(int i = 0; i < 4; i++){
                qmat.block<3, 3>(3 * i,  3 * i) = item[i] * M3D::Identity();
            }
            Gmat_ = Eigen::Matrix<double, N, 12>::Zero();
            // Gmat_.block<3, 3>( 3, 0) = -state_curr.rot.toRotationMatrix() * dt;
            // Gmat_.block<3, 3>( 6, 3) = -SO3Math::J_l(imu.gyro_rps * dt).transpose() * dt;
            Gmat_.block<3, 3>( 3, 0) = -I_33 * dt; // 近似
            Gmat_.block<3, 3>( 6, 3) = -I_33 * dt; // 近似
            Gmat_.block<3, 3>( 9, 6) = I_33 * dt;
            Gmat_.block<3, 3>(12, 9) = I_33 * dt;
            auto Qmat = Gmat_ * qmat * Gmat_.transpose();

            // 状态转移
            Pmat_ = PHImat * Pmat_ * PHImat.transpose() + Qmat;
        }

        
    }
    void backward_propagation(MeasureData& measure){
        auto size = measure.imu_queue.size();

        StateEKF state_curr = state_queue_.back();
        state_curr.pos = V3D(0, 0, 0);
        state_curr.vel = state_curr.rot.conjugate() * state_curr.vel;   // 变换到I系
        state_curr.grav = state_curr.rot.conjugate() * state_curr.grav; // 变换到I系
        state_curr.rot = QD::Identity();
        state_curr.time = measure.imu_queue.back().time; // measure.pcl_end_time;
        state_bp_queue_.clear();
        state_bp_queue_.push_back(state_curr);
        
        for(int i = size - 1; i > 0; i--){
            double dt = state_curr.time - measure.imu_queue[i - 1].time;
            state_curr.time = measure.imu_queue[i - 1].time;

            auto imu = measure.imu_queue[i];
            M3D R = state_curr.rot.toRotationMatrix() * SO3Math::Exp(-imu.gyro_rps * dt);
            state_curr.rot = QD(R);            
            state_curr.vel -= (state_curr.rot * imu.accel_mpss - state_curr.grav) * dt;
            state_curr.pos -= state_curr.vel * dt;

            state_bp_queue_.push_back(state_curr);
        }
        state_bp_queue_.push_back(state_last_);
    }

    void point_cloud_undistort(MeasureData& measure, PointCloud::Ptr& pcl_in, PointCloud::Ptr& pcl_out){
        // 将雷达坐标系下的点投影到扫描结束时刻的惯导坐标系下    
        int idx = 0;
        for(int i = pcl_in->size() - 1; i >= 0; i--){
            auto& point_curr = (*pcl_in)[i];
            if(!std::isfinite(point_curr.x)
                || !std::isfinite(point_curr.y)
                || !std::isfinite(point_curr.z)){
                continue;
            }
            double time_interval = 0.1;
            double time_curr_point = measure.pcl_beg_time + point_curr.curvature * time_interval;

            auto p_to_add = (*pcl_in)[i];
            if(time_curr_point > state_bp_queue_[idx].time){
                V3D p_l_i((*pcl_in)[i].x, (*pcl_in)[i].y, (*pcl_in)[i].z);
                // 从雷达坐标系转换到惯导坐标系下
                V3D p_b_i = model_param_.R_L_I * p_l_i + model_param_.T_L_I; 
                p_to_add.x = p_b_i.x();
                p_to_add.y = p_b_i.y();
                p_to_add.z = p_b_i.z();
                continue;
            }
            while(time_curr_point < state_bp_queue_[idx + 1].time){
                idx++;
            }
            // 当前点的时间在idx和idx+1之间
            if(idx + 1 < state_bp_queue_.size() 
                && time_curr_point > state_bp_queue_[idx + 1].time){
                
                auto imu = measure.imu_queue[(int)measure.imu_queue.size() - 1 - idx];
                double dt = state_bp_queue_[idx].time - time_curr_point;
                auto state_curr = state_bp_queue_[idx];
                M3D R = state_curr.rot.toRotationMatrix() * SO3Math::Exp(-imu.gyro_rps * dt);
                state_curr.rot = QD(R);            
                state_curr.vel -= (state_curr.rot * imu.accel_mpss - state_curr.grav) * dt;
                state_curr.pos -= state_curr.vel * dt;

                V3D p_l_i((*pcl_in)[i].x, (*pcl_in)[i].y, (*pcl_in)[i].z);
                V3D p_b_i = model_param_.R_L_I * p_l_i + model_param_.T_L_I; // 转换到惯导坐标系下
                V3D p_b_end = state_curr.rot * p_b_i + state_curr.pos;      // 转换到扫描结束时刻的惯导坐标系下
                
                p_to_add.x = p_b_end.x();
                p_to_add.y = p_b_end.y();
                p_to_add.z = p_b_end.z();

            }
            else if(idx + 1 >= state_bp_queue_.size()){
                V3D p_l_i((*pcl_in)[i].x, (*pcl_in)[i].y, (*pcl_in)[i].z);
                V3D p_b_i = model_param_.R_L_I * p_l_i + model_param_.T_L_I; // 转换到惯导坐标系下
                V3D p_b_end = state_last_.rot * p_b_i + state_last_.pos;      // 转换到扫描结束时刻的惯导坐标系下
                
                p_to_add.x = p_b_end.x();
                p_to_add.y = p_b_end.y();
                p_to_add.z = p_b_end.z();
            }
            pcl_out->push_back(p_to_add);
        }
    }

    struct EffectFeature{
        V3D pb;         // 点的I系坐标
        V3D norm_vec;   // 法向量
        double res;     // 残差
    };
    void iterate_update(MeasureData& measure, PointCloud::Ptr& pcl_features){
        // ROS_INFO("map_surf_cloud: %d", map_surf_cloud_->size());

        auto state_curr_iter = state_queue_.back(); // 当前迭代时刻的状态
        auto state_last_iter = state_queue_.back(); // 上一迭代时刻的状态
        auto state_iter_0    = state_queue_.back(); // 迭代更新前的状态
        Eigen::Matrix<double, N, N> Jmat     = Eigen::Matrix<double, N, N>::Identity();
        Eigen::Matrix<double, N, N> Jmat_inv = Eigen::Matrix<double, N, N>::Identity();
        auto Pmat = Pmat_;
        bool is_converge = true;
        int converge_cnt = 0;
        nearest_list_.resize(pcl_features->size());
        for(int iter = 0; iter < max_iter_times_; iter++){
            std::vector<EffectFeature> effect_feature_queue; // 有效特征点队列
            // ROS_INFO("pcl_features: %d", pcl_features->size());

            // omp_set_num_threads(MP_PROC_NUM);
            // #pragma omp parallel for
            TicToc t_feat;
            double time_k_search = 0;
            for(int i = 0; i < pcl_features->size(); i++){
                auto p = (*pcl_features)[i];
                // 将特征点投影到世界坐标系
                V3D cp(p.x, p.y, p.z);
                V3D wp = state_curr_iter.rot * cp + state_curr_iter.pos;
                p.x = wp.x();
                p.y = wp.y();
                p.z = wp.z(); 
                if(!std::isfinite(wp.x()) || std::isnan(wp.x())){
                    std::cout << "iter: " << iter << " i: " << i << "/" << pcl_features->size() << " cp: " << cp.transpose() << std::endl;
                }
                auto& nearest = nearest_list_[i];
                if(is_converge){
                    // 最近邻搜索
                    TicToc t_kdtree;
                    std::vector<float> pointSearchSqDis; // 最近邻搜索结果的距离，从小到大排列
                    ikd_tree_->Nearest_Search(p, KNN_MATCH_NUM, nearest, pointSearchSqDis);
                    time_k_search += t_kdtree.toc();

                    // 最近邻都在指定范围内
                    if(pointSearchSqDis[KNN_MATCH_NUM - 1] < 5.0 
                        && nearest.size() >= KNN_MATCH_NUM){
                            is_surf_list_[i] = true;
                    }
                }
                if(!is_surf_list_[i]){
                    continue;
                }

                // 求解方程Ax=b
                Eigen::Matrix<double, Eigen::Dynamic, 3> A = Eigen::MatrixXd::Zero(KNN_MATCH_NUM, 3);
                Eigen::Matrix<double, Eigen::Dynamic, 1> b = Eigen::VectorXd::Ones(KNN_MATCH_NUM) * (-1.0);
                for(int k = 0; k < KNN_MATCH_NUM; k++){
                    A(k, 0) = nearest[k].x;
                    A(k, 1) = nearest[k].y;
                    A(k, 2) = nearest[k].z;
                }
                V3D normvec = A.colPivHouseholderQr().solve(b); // 求解法向量
                double norm = normvec.norm(); // 法向量模长
                normvec.normalize(); // 法向量归一化

                Eigen::Vector4d abcd;
                abcd(0) = normvec(0);
                abcd(1) = normvec(1);
                abcd(2) = normvec(2);
                abcd(3) = 1 / norm;
                // 判断最近邻是否在同一平面
                bool is_plane = true;
                for(int k = 0; k < KNN_MATCH_NUM; k++){
                    if(abcd(0) * nearest[k].x + abcd(1) * nearest[k].y + abcd(2) * nearest[k].z + abcd(3) > 0.1){
                        is_plane = false;
                        break;
                    }
                }
                if(is_plane){
                    // 计算点到平面的距离
                    double res = abcd(0) * wp.x() + abcd(1) * wp.y() + abcd(2) * wp.z() + abcd(3);
                    // 判断是否为有效特征点
                    double s = 1 - 0.9 * fabs(res) / sqrt(cp.norm());
                    if(s > 0.9)
                        effect_feature_queue.push_back(EffectFeature{cp, normvec, res});
                    else
                        is_surf_list_[i] = false;
                }
            }
            // spdlog::info("iter: {} points: {} feat: {} time_k_search: {} total_time: {}", iter, pcl_features->size(), effect_feature_queue.size(), time_k_search, t_feat.toc());

            Hmat_ = Eigen::MatrixXd::Zero(effect_feature_queue.size(), N);
            delta_z_ = Eigen::VectorXd::Zero(effect_feature_queue.size());
            for(int i = 0; i < effect_feature_queue.size(); i++){
                auto& ef = effect_feature_queue[i];
                Hmat_.block<1, 3>(i, 0) =  ef.norm_vec.transpose();
                Hmat_.block<1, 3>(i, 6) = -ef.norm_vec.transpose() * state_curr_iter.rot.toRotationMatrix() * SO3Math::get_skew_symmetric(ef.pb);
                delta_z_(i) = ef.res;
            }

            double R = model_param_.R; // 观测噪声协方差

            delta_x_.block<3, 1>( 0, 0) = state_curr_iter.pos - state_iter_0.pos;
            delta_x_.block<3, 1>( 3, 0) = state_curr_iter.vel - state_iter_0.vel;
            delta_x_.block<3, 1>( 6, 0) = SO3Math::Log(state_iter_0.rot.toRotationMatrix().transpose() 
                                        * state_curr_iter.rot.toRotationMatrix());
            delta_x_.block<3, 1>( 9, 0) = state_curr_iter.bg - state_iter_0.bg;
            delta_x_.block<3, 1>(12, 0) = state_curr_iter.ba - state_iter_0.ba;
            delta_x_.block<3, 1>(15, 0) = state_curr_iter.grav - state_iter_0.grav;

            auto dx_new = delta_x_;
             
            auto Amat = SO3Math::J_l(-delta_x_.block<3, 1>( 6, 0));
            Jmat_inv.block<3, 3>(6, 6) = Amat;//.transpose();
            Pmat = Jmat_inv * Pmat_ * Jmat_inv.transpose();

            // 卡尔曼增益
            Kmat_ = (Hmat_.transpose() * Hmat_ + (Pmat / R).inverse()).inverse() * Hmat_.transpose();
            // Kmat_ = Pmat_ * Hmat_.transpose() * (Hmat_ * Pmat_ * Hmat_.transpose() + Rmat).inverse();
            // std::cout << "Kmat_: " << Kmat_ << std::endl;

            // 状态迭代
            dx_new.block<3, 1>(6, 0) = Amat.transpose() * dx_new.block<3, 1>(6, 0);
            auto delta = -Kmat_ * delta_z_ - (Eigen::Matrix<double, N, N>::Identity() - Kmat_ * Hmat_) * dx_new;
            // std::cout << delta.transpose() << std::endl;
            state_curr_iter.pos = state_last_iter.pos + delta.block<3, 1>( 0, 0);
            state_curr_iter.vel = state_last_iter.vel + delta.block<3, 1>( 3, 0);
            state_curr_iter.rot = state_last_iter.rot.toRotationMatrix() * SO3Math::Exp(delta.block<3, 1>( 6, 0));
            state_curr_iter.rot.normalize(); // ?? 理论上不需要再进行归一化
            state_curr_iter.bg = state_last_iter.bg + delta.block<3, 1>( 9, 0);
            state_curr_iter.ba = state_last_iter.ba + delta.block<3, 1>(12, 0);
            // state_curr_iter.grav = state_last_iter.grav + delta.block<3, 1>(15, 0);
            state_last_iter = state_curr_iter;

            // 收敛状态判断
            is_converge = true;
            for(int i = 0; i < N - 3; i++){
                if(delta(i) > 0.001){
                    is_converge = false;
                    break;
                }
            }
            if(is_converge)
                converge_cnt++;
            if(!converge_cnt && iter == max_iter_times_ - 2)
                is_converge = true;
            if(converge_cnt > 1)
                break;
        }
        state_last_ = state_curr_iter;
        Pmat_ = (Eigen::Matrix<double, N, N>::Identity() - Kmat_ * Hmat_) * Pmat;        
    }

    void map_update(PointCloud::Ptr& features){
        TicToc t_build_map;
        PointCloud::Ptr points_to_add = boost::make_shared<PointCloud>();
        PointCloud::Ptr points_not_to_downsample = boost::make_shared<PointCloud>();
        for(int i = 0; i < features->size(); i++){
            auto& nearest = nearest_list_[i];
            V3D pb((*features)[i].x, (*features)[i].y, (*features)[i].z);
            V3D pw = state_last_.rot * pb + state_last_.pos; // 转换到世界坐标系下

            PointType p;
            p.x = pw.x();
            p.y = pw.y();
            p.z = pw.z();

            PointType mid_point; // 当前点所在体素的中心点
            mid_point.x = floor(pw.x() / map_res_) * map_res_ + map_res_ / 2;
            mid_point.y = floor(pw.y() / map_res_) * map_res_ + map_res_ / 2;
            mid_point.z = floor(pw.z() / map_res_) * map_res_ + map_res_ / 2;
            V3D p_mid(mid_point.x, mid_point.y, mid_point.z);
            double dist = (pw - p_mid).norm(); // 到体素中心的距离
            // 如果最近邻到该体素中心的距离大于阈值，则认为该点是新的特征点，不需要降采样
            if(fabs(nearest[0].x - mid_point.x) > map_res_ / 2
                && fabs(nearest[0].y - mid_point.y) > map_res_ / 2
                && fabs(nearest[0].z - mid_point.z) > map_res_ / 2){
                points_not_to_downsample->push_back(p);
                continue;
            }
            bool need_add = true;
            // 如果有最近邻在体素中，而且距离中心更近，则不添加该点
            for(int k = 0; k < KNN_MATCH_NUM; k++){
                if(nearest.size() < KNN_MATCH_NUM)
                    break;
                V3D pn(nearest[k].x, nearest[k].y, nearest[k].z);
                if((pn - p_mid).norm() < dist){
                    need_add = false;
                    break;
                }
            }
            if(need_add){
                points_to_add->push_back(p);
            }
        }
        ikd_tree_->Add_Points(points_to_add->points, true);
        ikd_tree_->Add_Points(points_not_to_downsample->points, false);

        spdlog::info("map size: {}", ikd_tree_->validnum());
        ROS_INFO("build map time: %f", t_build_map.toc());
    }

public:
    bool init_imu(MeasureData& measure){
        static int cnt = 0;
        for(auto& imu : measure.imu_queue){
            state_last_.ba += imu.accel_mpss;
            state_last_.bg += imu.gyro_rps;
            cnt++;
        }
        if(cnt > 1000){
            state_last_.grav << 0, 0, -9.7936;
            state_last_.bg /= (double)cnt;
            state_last_.ba /= (double)cnt;
            state_last_.ba -= state_last_.grav;
            cnt = 0;
            return true;
        }
        return false;
    }
    void process(MeasureData& measure){
        
        auto& cloud_surf = measure.cloud;

        if(!is_initialized_){
            // 转换到惯性系
            for(int i = 0; i < cloud_surf->points.size(); i++){
                auto& p = cloud_surf->points[i];
                V3D pl = V3D(p.x, p.y, p.z);
                V3D pb = model_param_.R_L_I * pl + model_param_.T_L_I;
                p.x = pb.x();
                p.y = pb.y();
                p.z = pb.z();
            }
            surf_cloud_queue_.push_back(cloud_surf);
            *map_surf_cloud_ += *cloud_surf;

            PointCloud::Ptr surf_filtered(new PointCloud);
            pcl::VoxelGrid<PointType> sor;
            sor.setInputCloud(map_surf_cloud_);
            sor.setLeafSize(map_res_, map_res_, map_res_);
            sor.filter(*surf_filtered);
            map_surf_cloud_ = surf_filtered;

            if(map_surf_cloud_->size() > 1000 && init_imu(measure)){
                is_initialized_ = true;
                state_last_.time = measure.imu_queue.back().time;
                ikd_tree_->Build(map_surf_cloud_->points);

                state_queue_.clear();
                map_surf_cloud_->clear();
                surf_cloud_queue_.clear();
            }
            return;
        }
        // 原始数据补偿
        for(auto& imu : measure.imu_queue){
            imu.accel_mpss  -= state_last_.ba;
            imu.gyro_rps    -= state_last_.bg;
        }
        // 前向传播
        forward_propagation(measure);
        // 反向传播
        backward_propagation(measure);
        // 点云运动补偿，转换到该帧结束时的惯导坐标系下
        PointCloud::Ptr pcl_out = boost::make_shared<PointCloud>();
        point_cloud_undistort(measure, cloud_surf, pcl_out);
        // 降采样
        surf_frame_ds_filter_.setInputCloud(pcl_out);
        PointCloud::Ptr pcl_out_filtered(new PointCloud);
        surf_frame_ds_filter_.filter(*pcl_out_filtered);
        // ROS_INFO("pcl out size: %d", pcl_out_filtered->size());
        // 点云迭代更新
        TicToc t_update;
        iterate_update(measure, pcl_out_filtered);
        ROS_INFO("update time: %f", t_update.toc());
        // 地图增量更新
        map_update(pcl_out_filtered);

        // 发布坐标转换
        odom_trans.header.stamp = ros::Time(measure.pcl_end_time);
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "livox";
        odom_trans.transform.translation.x = state_last_.pos.x();
        odom_trans.transform.translation.y = state_last_.pos.y();
        odom_trans.transform.translation.z = state_last_.pos.z();
        odom_trans.transform.rotation.x = state_last_.rot.x();
        odom_trans.transform.rotation.y = state_last_.rot.y();
        odom_trans.transform.rotation.z = state_last_.rot.z();
        odom_trans.transform.rotation.w = state_last_.rot.w();
        odom_broadcaster.sendTransform(odom_trans);

        // 发布轨迹
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time(measure.pcl_end_time);
        pose.header.frame_id = "map";
        pose.pose.position.x = state_last_.pos.x();
        pose.pose.position.y = state_last_.pos.y();
        pose.pose.position.z = state_last_.pos.z();
        pose.pose.orientation.x = state_last_.rot.x();
        pose.pose.orientation.y = state_last_.rot.y();
        pose.pose.orientation.z = state_last_.rot.z();
        pose.pose.orientation.w = state_last_.rot.w();
        traj_local_msgs.poses.push_back(pose);
        traj_local_msgs.header = pose.header;
        pub_traj.publish(traj_local_msgs);

        // 发布点云
        sensor_msgs::PointCloud2 cloud_msg;
        PointCloud::Ptr points_world = boost::make_shared<PointCloud>();
        for(int i = 0; i < pcl_out->size(); i++){
            V3D pb((*pcl_out)[i].x, (*pcl_out)[i].y, (*pcl_out)[i].z);
            V3D pw = state_last_.rot * pb + state_last_.pos; // 转换到世界坐标系下
            PointType p = (*pcl_out)[i];
            p.x = pw.x();
            p.y = pw.y();
            p.z = pw.z();
            points_world->push_back(p);
        }
        pcl::toROSMsg(*points_world, cloud_msg);
        cloud_msg.header.stamp = ros::Time(measure.pcl_end_time);
        cloud_msg.header.frame_id = "map";
        pub_surf_cloud.publish(cloud_msg);

        // 将结果保存为文本文件
        if(0){
            trajctory_writer_->write_txt(state_last_.to_string());
        }
    }

};

#endif // IMU_PROCESS_HPP