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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

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
    QD R_L_I = QD::Identity();              // IMU坐标系到点云坐标系的旋转
    V3D T_L_I;                              // IMU坐标系到点云坐标系的变换
    void init_r_l_i(double roll, double pitch, double yaw){
        R_L_I.x()=sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2);
        R_L_I.y()=sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2);
        R_L_I.z()=cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2);
        R_L_I.w()=cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2);
    }
    void init_t_l_i(double x, double y, double z){
        T_L_I(0) = x;
        T_L_I(1) = y;
        T_L_I(2) = z;
    }
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

    Eigen::Matrix<double, Eigen::Dynamic, 1> delta_z_;
    Eigen::Matrix<double, Eigen::Dynamic, N> Hmat_;

    // 地图相关
    std::vector<PointCloud::Ptr> surf_cloud_queue_;
    PointCloud::Ptr map_surf_cloud_;
    double map_res_ = 0.2;
    pcl::KdTreeFLANN<PointType> kdtree_;


public:
    IMUProcess()=default;
    IMUProcess(const ModelParam& model_param) : model_param_(model_param){
        map_surf_cloud_ = boost::make_shared<PointCloud>();
        state_last_ = StateEKF();
    }

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

        StateEKF state_curr = state_queue_.back();
        state_curr.pos = V3D(0, 0, 0);
        state_curr.vel = state_curr.rot.conjugate() * state_curr.vel; // 变换到I系
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

        // 将每一点投影到扫描结束时刻的惯导坐标系下    
        int idx = 0;
        for(int i = pcl_in->size() - 1; i >= 0; i--){
            auto& point_curr = (*pcl_in)[i];
            double time_interval = map_res_;
            double time_curr_point = measure.pcl_beg_time + point_curr.curvature * time_interval;

            if(time_curr_point > state_bp_queue_[idx].time){
                V3D p_l_i((*pcl_in)[i].x, (*pcl_in)[i].y, (*pcl_in)[i].z);
                V3D p_b_i = model_param_.R_L_I * p_l_i + model_param_.T_L_I; // 转换到惯导坐标系下
                auto p = (*pcl_in)[i];
                p.x = p_b_i.x();
                p.y = p_b_i.y();
                p.z = p_b_i.z();
                pcl_out->push_back(p);
                continue;
            }
            while(time_curr_point < state_bp_queue_[idx + 1].time){
                idx++;
            }
            if(time_curr_point > state_bp_queue_[idx + 1].time){
                auto imu = measure.imu_queue[measure.imu_queue.size() - 1 - idx];
                double dt = state_bp_queue_[idx].time - time_curr_point;
                // std::cout << "idx: " << idx <<" dt: " << dt << std::endl;
                auto state_curr = state_bp_queue_[idx];
                M3D R = state_curr.rot.toRotationMatrix() * SO3Math::Exp(-imu.gyro_rps * dt);
                state_curr.rot = QD(R);            
                state_curr.vel -= (state_curr.rot * imu.accel_mpss - state_curr.grav) * dt;
                state_curr.pos -= state_curr.vel * dt;

                V3D p_l_i((*pcl_in)[i].x, (*pcl_in)[i].y, (*pcl_in)[i].z);
                V3D p_b_i = model_param_.R_L_I * p_l_i + model_param_.T_L_I; // 转换到惯导坐标系下
                V3D p_b_end = state_curr.rot * p_l_i + state_curr.pos;      // 转换到扫描结束时刻的惯导坐标系下
                auto p = (*pcl_in)[i];
                p.x = p_b_end.x();
                p.y = p_b_end.y();
                p.z = p_b_end.z();
                pcl_out->push_back(p);

                // std::cout << "p_b_end: " << p_b_end.transpose() << std::endl;
            }
            
        }
        // std::cout << "----------------------------------------------------" << std::endl;
    }

    struct EffectFeature{
        V3D pb;          // 点的I系坐标
        V3D norm_vec; // 法向量
        double res;      // 残差
    };

    void update(MeasureData& measure, PointCloud::Ptr& pcl_features){
        kdtree_.setInputCloud(map_surf_cloud_);
        // KD-Tree最近邻搜索结果的索引和距离
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        auto state_curr = state_queue_.back();
        for(int iter = 0; iter < 10; iter++){

            std::vector<EffectFeature> effect_feature_queue;

            for(int i = 0; i < pcl_features->size(); i++){
                auto p = (*pcl_features)[i];
                // 投影到世界坐标系
                V3D cp(p.x, p.y, p.z);
                V3D wp = state_curr.rot * cp + state_curr.pos;
                p.x = wp.x();
                p.y = wp.y();
                p.z = wp.z();
                // 最近邻搜索
                kdtree_.nearestKSearch(p, 8, pointSearchInd, pointSearchSqDis);
                // 最近邻都在指定范围内
                if (pointSearchSqDis[7] < 5){
                    // 计算质心和协方差
                    PointCloudXYZI nearest;
                    for (int k = 0; k < pointSearchInd.size(); k++)
                    {
                        nearest.push_back((*map_surf_cloud_)[pointSearchInd[k]]);
                    }
                    Eigen::Vector4f centroid;   // 质心
                    Eigen::Matrix3f covariance; // 协方差
                    pcl::compute3DCentroid(nearest, centroid);
                    pcl::computeCovarianceMatrix(nearest, centroid, covariance);

                    // 计算协方差矩阵的特征值
                    Eigen::Vector3f eigenValues;
                    pcl::eigen33(covariance, eigenValues);

                    std::vector<float> eigenValues_sort{eigenValues[0], eigenValues[1], eigenValues[2]};
                    std::sort(eigenValues_sort.begin(), eigenValues_sort.end()); // 升序排列
                    // 最近邻在一个平面上
                    if (eigenValues_sort[1] > 3 * eigenValues_sort[0])
                    {
                        Eigen::Vector3d lpj{nearest[0].x, nearest[0].y, nearest[0].z};
                        Eigen::Vector3d lpl{nearest[4].x, nearest[4].y, nearest[4].z};
                        Eigen::Vector3d lpm{nearest[7].x, nearest[7].y, nearest[7].z};
                        // 计算jlm的归一化法向量
                        V3D ljm = (lpj - lpl).cross(lpj - lpm);
                        ljm.normalize();
                        double res = (wp - lpj).dot(ljm);
                        double s = 1 - 0.9 * fabs(res) / sqrt(cp.norm());
                        if(s > 0.9)
                            effect_feature_queue.push_back(EffectFeature{cp, ljm, res});
                    }
                }
            }
            Hmat_ = Eigen::MatrixXd::Zero(effect_feature_queue.size(), N);
            delta_z_ = Eigen::VectorXd::Zero(effect_feature_queue.size());
            for(int i = 0; i < effect_feature_queue.size(); i++){
                auto& ef = effect_feature_queue[i];
                Hmat_.block<1, 3>(i, 0) =  ef.norm_vec.transpose();
                Hmat_.block<1, 3>(i, 6) = -ef.norm_vec.transpose() * state_curr.rot.toRotationMatrix() * SO3Math::get_skew_symmetric(ef.pb);
                delta_z_(i) = ef.res;
            }
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Rmat 
                = Eigen::MatrixXd::Identity(effect_feature_queue.size(), effect_feature_queue.size()) * 0.001;
            Eigen::Matrix<double, N, Eigen::Dynamic> Kmat 
                = (Hmat_.transpose() * Rmat.inverse() * Hmat_ + Pmat_.inverse()).inverse() * Hmat_.transpose() * Rmat.inverse();
            


        }
    }

public:
    bool init_imu(MeasureData& measure){
        static int cnt = 0;
        for(auto& imu : measure.imu_queue){
            state_last_.ba += imu.accel_mpss;
            state_last_.bg += imu.gyro_rps;
            cnt++;
        }
        if(cnt > 2000){
            state_last_.grav << 0, 0, -9.7936;
            state_last_.bg /= cnt;
            state_last_.ba /= cnt;
            state_last_.ba -= state_last_.grav;
            cnt = 0;
            return true;
        }
        return false;
    }
 
    void process(MeasureData& measure){

        // 特征提取
        ScanRegistration scan_registration;
        scan_registration.feature_extract(measure.cloud);
        auto& cloud_surf = scan_registration.pointSurf;


        if(!is_initialized_){
            surf_cloud_queue_.push_back(cloud_surf);
            *map_surf_cloud_ += *cloud_surf;

            PointCloudXYZI::Ptr surf_filtered(new PointCloudXYZI);
            pcl::VoxelGrid<PointType> sor;
            sor.setInputCloud(map_surf_cloud_);
            sor.setLeafSize(map_res_, map_res_, map_res_);
            sor.filter(*surf_filtered);
            map_surf_cloud_ = surf_filtered;

            if(map_surf_cloud_->size() > 1000 && init_imu(measure)){
                is_initialized_ = true;
                state_last_.time = measure.imu_queue.back().time;
                state_queue_.clear();
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

        // 点云运动补偿
        PointCloud::Ptr pcl_out = boost::make_shared<PointCloud>();
        point_cloud_undistort(measure, cloud_surf, pcl_out);

        // 点云迭代更新
        update(measure, pcl_out);

        // 误差状态修正
        delta_x_ = Eigen::Matrix<double, N, 1>::Zero();

        // 地图增量更新
        PointCloud::Ptr cloud_surf_w = boost::make_shared<PointCloud>();
        cloud_surf_w->resize(pcl_out->size());
        for(int i = 0; i < pcl_out->size(); i++){
            V3D pb((*pcl_out)[i].x, (*pcl_out)[i].y, (*pcl_out)[i].z);
            V3D pl = state_last_.rot * pb + state_last_.pos; // 转换到世界坐标系下
            (*cloud_surf_w)[i].x = pl.x();
            (*cloud_surf_w)[i].y = pl.y();
            (*cloud_surf_w)[i].z = pl.z();
        }
        surf_cloud_queue_.push_back(cloud_surf_w);
        map_surf_cloud_->clear();
        for(auto& cloud : surf_cloud_queue_){
            *map_surf_cloud_ += *cloud;
        }
        PointCloudXYZI::Ptr surf_filtered(new PointCloudXYZI);
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(map_surf_cloud_);
        sor.setLeafSize(map_res_, map_res_, map_res_);
        sor.filter(*surf_filtered);
        map_surf_cloud_ = surf_filtered;
        if (map_surf_cloud_->size() > 16000)
            surf_cloud_queue_.erase(surf_cloud_queue_.begin());

        if(1){
            state_last_ = state_queue_.back();
            state_queue_.clear();
            auto euler = SO3Math::quat2euler(state_last_.rot);
            euler = euler * 180 / M_PI;
            std::cout << "time: " << state_last_.time << " euler: " << euler.transpose()<< std::endl;
        }
    }

};

#endif // IMU_PROCESS_HPP