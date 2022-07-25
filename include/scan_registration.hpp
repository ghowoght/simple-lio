/**
 * @file scanRegistration.hpp
 * @brief 点云配准
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2021-10-15
 * 
 * @copyright Copyright (c) 2021  WHU-I2NAV
 * 
 */
#ifndef SCAN_REGISTRATION_HPP
#define SCAN_REGISTRATION_HPP

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include <eigen3/Eigen/Core>
#include <Eigen/Eigenvalues>

enum POINTTYPE{
    TYPE_SURF           = 1,   // 平面点
    TYPE_BREAK_POINT    = 100, // 
    TYPE_CORNER         = 150, // 角点
    TYPE_OUTLIER        = 250, // 外点/异常点
};

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class ScanRegistration{

public:
    PointCloudXYZI pointEdge;      // 边缘点
    PointCloudXYZI pointSurf;      // 平面点

private:
    // ros::NodeHandle nh_;
    // ros::Subscriber sub_cloud;
    // ros::Publisher pub_laser_cloud;
    // ros::Publisher pub_edge_cloud;
    // ros::Publisher pub_surf_cloud;
public:
    void feature_extract(pcl::PointCloud<PointType>& laserCloudIn){

        // 预处理
        int cloudSize = laserCloudIn.points.size();
        PointCloudXYZI allPoints;
        for(size_t i = 1; i < cloudSize - 1; i++){
            auto point = laserCloudIn.points[i];
            auto dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            auto phi = atan(sqrt((point.y * point.y + point.z * point.z) / (point.x * point.x))); // 光束与激光雷达x轴的夹角

            auto reflectivity = (laserCloudIn.points[i].intensity - (int)laserCloudIn.points[i].intensity) * 10000;
            auto intensity = reflectivity / (dist * dist);

            auto a = laserCloudIn.points[i - 1];
            auto b = laserCloudIn.points[i];
            auto c = laserCloudIn.points[i + 1];
            Eigen::Vector3d pa{a.x, a.y, a.z};
            Eigen::Vector3d pb{b.x, b.y, b.z};
            Eigen::Vector3d pc{c.x, c.y, c.z};
            auto incident_angle = acos(abs((pa - pc).dot(pb)) / ((pa - pc).norm() * pb.norm())); // incident angle 与光束的夹角
            
            if( std::isfinite(point.x)
                && std::isfinite(point.y)
                && std::isfinite(point.z)
                && phi < 35 * 3.1415926535 / 180.0
                // && intensity > 7e-3 && intensity < 1e-1
                && incident_angle > 5 * 3.1415926535 / 180.0
                && incident_angle < 175 * 3.1415926535 / 180.0
            ){
                allPoints.push_back(point);
            }
             
            
        }

        PointCloudXYZI::Ptr laserCloud(new PointCloudXYZI());
        *laserCloud += allPoints;
        cloudSize = laserCloud->size();

        int cloudFeatureFlag[32000] = {0};
       
        pointEdge.clear();      // 边缘点
        pointSurf.clear();      // 平面点
        bool left_surf_flag = false;
        bool right_surf_flag = false;
        int step = 1;

        /***** 提取平面点和角点 *****/
        for(size_t i = 5; i < cloudSize - 5; i += step){
            // 计算左侧点points[i-2]的平滑度
            double diffXl = laserCloud->points[i - 4].x
                          + laserCloud->points[i - 3].x
                          + laserCloud->points[i - 1].x
                          + laserCloud->points[i - 0].x
                          - laserCloud->points[i - 2].x * 4;
            double diffYl = laserCloud->points[i - 4].y
                          + laserCloud->points[i - 3].y
                          + laserCloud->points[i - 1].y
                          + laserCloud->points[i - 0].y
                          - laserCloud->points[i - 2].y * 4;
            double diffZl = laserCloud->points[i - 4].z
                          + laserCloud->points[i - 3].z
                          + laserCloud->points[i - 1].z
                          + laserCloud->points[i - 0].z
                          - laserCloud->points[i - 2].z * 4;
            double curvature = diffXl * diffXl + diffYl * diffYl + diffZl * diffZl;
            if(curvature < 0.01){
                if(curvature < 0.001) // 平滑度小于x，则认为该点为平面点
                    cloudFeatureFlag[i - 2] = TYPE_SURF;
                left_surf_flag = true;
            }
            else left_surf_flag = false;
            

            // 计算右侧点points[i+2]的平滑度
            double diffXr = laserCloud->points[i + 4].x
                          + laserCloud->points[i + 3].x
                          + laserCloud->points[i + 1].x
                          + laserCloud->points[i + 0].x
                          - laserCloud->points[i + 2].x * 4;
            double diffYr = laserCloud->points[i + 4].y
                          + laserCloud->points[i + 3].y
                          + laserCloud->points[i + 1].y
                          + laserCloud->points[i + 0].y
                          - laserCloud->points[i + 2].y * 4;
            double diffZr = laserCloud->points[i + 4].z
                          + laserCloud->points[i + 3].z
                          + laserCloud->points[i + 1].z
                          + laserCloud->points[i + 0].z
                          - laserCloud->points[i + 2].z * 4;
            curvature = diffXr * diffXr + diffYr * diffYr + diffZr * diffZr;
            if(curvature < 0.01){
                if(curvature < 0.001) // 平滑度小于x，则认为该点为平面点
                    cloudFeatureFlag[i + 2] = TYPE_SURF;
                right_surf_flag = true;
                step = 4;
            }
            else {
                right_surf_flag = false;
                step = 1;
            }
            // 如果左右两侧都是平面，则判断该点是否为角点
            if(left_surf_flag && right_surf_flag){
                Eigen::Vector3d norm_left(0, 0, 0);
                Eigen::Vector3d norm_right(0, 0, 0);
                for(size_t k = 1; k < 5; k++){
                    Eigen::Vector3d tmp_l, tmp_r;
                    tmp_l << laserCloud->points[i - k].x - laserCloud->points[i].x,
                             laserCloud->points[i - k].y - laserCloud->points[i].y,
                             laserCloud->points[i - k].z - laserCloud->points[i].z;
                    tmp_l.normalize();
                    norm_left += (k / 10.0) * tmp_l;
                    tmp_r << laserCloud->points[i + k].x - laserCloud->points[i].x,
                             laserCloud->points[i + k].y - laserCloud->points[i].y,
                             laserCloud->points[i + k].z - laserCloud->points[i].z;
                    tmp_r.normalize();
                    norm_right += (k / 10.0) * tmp_r;
                }
                // 计算角度
                double theta = fabs(norm_left.dot(norm_right) / (norm_left.norm() * norm_right.norm()));
                Eigen::Vector3d left_tmp, right_tmp;
                left_tmp << laserCloud->points[i - 4].x - laserCloud->points[i].x,
                            laserCloud->points[i - 4].y - laserCloud->points[i].y,
                            laserCloud->points[i - 4].z - laserCloud->points[i].z;
                right_tmp << laserCloud->points[i + 4].x - laserCloud->points[i].x,
                             laserCloud->points[i + 4].y - laserCloud->points[i].y,
                             laserCloud->points[i + 4].z - laserCloud->points[i].z;
                /**
                 *  p1--------p0(角点) / surf-surf角点
                 *          /
                 *        /
                 *    p2/ 
                 */
                if(theta < 0.5 && left_tmp.norm() > 0.05 && right_tmp.norm() > 0.05){
                    // 判断为角点
                    cloudFeatureFlag[i] = TYPE_CORNER;
                }
                
            }
        }
        
        ///// TODO 进一步处理 /////
        /***** 剔除异常点和break point *****/
        for(size_t i = 5; i < cloudSize - 5; i++){
            float depth = sqrt(   laserCloud->points[i].x * laserCloud->points[i].x
                                + laserCloud->points[i].y * laserCloud->points[i].y
                                + laserCloud->points[i].z * laserCloud->points[i].z);
            // 计算该点附近的点与该点的距离
            float depth_diff_r[2], depth_diff_l[2]; // 存储左右两点与当前点的距离
            for(size_t k = 1; k < 3; k++){
                float diffX = laserCloud->points[i + k].x - laserCloud->points[i].x;
                float diffY = laserCloud->points[i + k].y - laserCloud->points[i].y;
                float diffZ = laserCloud->points[i + k].z - laserCloud->points[i].z;
                depth_diff_r[k - 1] = sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ); // 第i点到第i+k点的距离
                diffX = laserCloud->points[i - k].x - laserCloud->points[i].x;
                diffY = laserCloud->points[i - k].y - laserCloud->points[i].y;
                diffZ = laserCloud->points[i - k].z - laserCloud->points[i].z;
                depth_diff_l[k - 1] = sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
            }
            // 右侧激光点到LiDAR的距离
            float depth_right = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x
                                + laserCloud->points[i + 1].y * laserCloud->points[i + 1].y
                                + laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
            // 左侧激光点到LiDAR的距离
            float depth_left  = sqrt(laserCloud->points[i - 1].x * laserCloud->points[i - 1].x
                                + laserCloud->points[i - 1].y * laserCloud->points[i - 1].y
                                + laserCloud->points[i - 1].z * laserCloud->points[i - 1].z);
            /* 
             * 强凹凸点 / 与激光束平行 判定为异常点
             * ....   ...      //  .....
             *     . .         //       . @outlier 
             *      . @outlier //        ......
             */
            if(depth_diff_r[0] > 0.1 * depth && depth_diff_l[0] > 0.1 * depth){
                cloudFeatureFlag[i] = TYPE_OUTLIER; // 判断为异常点
                continue;
            }
            // 不是异常点，继续处理，判断是不是break point
            if(fabs(depth_diff_r[0] - depth_diff_l[0]) > 0.1){

                if(depth_diff_r[0] > depth_diff_l[0]){

                    // 第i点到第(i-4)点的平面向量
                    Eigen::Vector3d a = Eigen::Vector3d(laserCloud->points[i - 4].x - laserCloud->points[i].x,
                                                        laserCloud->points[i - 4].y - laserCloud->points[i].y,
                                                        laserCloud->points[i - 4].z - laserCloud->points[i].z);
                    // 第i点的激光束向量
                    Eigen::Vector3d b = Eigen::Vector3d(laserCloud->points[i].x,
                                                        laserCloud->points[i].y,
                                                        laserCloud->points[i].z);
                    double surf_dist = a.norm();
                    // 计算a、b向量夹角β的余弦 a·b=|a||b|cosβ
                    // 接近1则表示两个向量平行，接近0则垂直
                    double cos_beta = fabs(a.dot(b) / a.norm() / b.norm());

                    // 计算左侧相邻点p(i-k),p(i-k-1),k=0,1,...3之间的距离
                    PointCloudXYZI points;
                    double min_dist = 10000;
                    double max_dist = 0;
                    for(int j = 0; j < 4; j++){
                        points.push_back(laserCloud->points[i - j]);
                        Eigen::Vector3f temp{laserCloud->points[i - j].x - laserCloud->points[i - j - 1].x,
                                             laserCloud->points[i - j].y - laserCloud->points[i - j - 1].y,
                                             laserCloud->points[i - j].z - laserCloud->points[i - j - 1].z};
                        double temp_dist = temp.norm();
                        if(temp_dist > max_dist) max_dist = temp_dist;
                        if(temp_dist < min_dist) min_dist = temp_dist;                      
                    }

                    Eigen::Vector4f centroid; // 质心
                    Eigen::Matrix3f covariance; // 协方差
                    pcl::compute3DCentroid(points, centroid);
                    pcl::computeCovarianceMatrix(points, centroid, covariance);
                    // 计算协方差矩阵的特征值
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(covariance);
                    Eigen::Vector3f eigenValues = saes.eigenvalues(); // 默认升序
                    bool is_plane = (eigenValues[1] > 100 * eigenValues[0]);
                    // ROS_INFO("is_plane: %d, max_dist: %f, min_dist: %f, surf_dist: %f, cos_beta: %f"
                    //         , is_plane, max_dist, min_dist, surf_dist, cos_beta);

                    // if(surf_dist < 0.05){
                    //     ROS_INFO("surf_dist: %f"
                    //             , surf_dist);
                    //     }

                    if(is_plane
                        && (max_dist < 2 * min_dist) 
                        && surf_dist < 0.5 // 第4个点到当前点的距离
                        && cos_beta < 0.8
                    ){
                        if(depth_right < depth_left){
                            cloudFeatureFlag[i] = TYPE_BREAK_POINT;
                        }
                        else{
                            if(depth_left == 0) cloudFeatureFlag[i] = TYPE_BREAK_POINT;
                        }
                    }
                    
                }
                else{
                    // 第i点到第(i+4)点的平面向量
                    Eigen::Vector3d a = Eigen::Vector3d(laserCloud->points[i + 4].x - laserCloud->points[i].x,
                                                        laserCloud->points[i + 4].y - laserCloud->points[i].y,
                                                        laserCloud->points[i + 4].z - laserCloud->points[i].z);
                    // 第i点的激光束向量
                    Eigen::Vector3d b = Eigen::Vector3d(laserCloud->points[i].x,
                                                        laserCloud->points[i].y,
                                                        laserCloud->points[i].z);
                    double surf_dist = a.norm();
                    // 计算a、b向量夹角β的余弦 a·b=|a||b|cosβ
                    // 接近1则表示两个向量平行，接近0则垂直
                    double cos_beta = fabs(a.dot(b) / a.norm() / b.norm());

                    // 计算左侧相邻点p(i-k),p(i-k-1),k=0,1,...3之间的距离
                    PointCloudXYZI points;
                    double min_dist = 10000;
                    double max_dist = 0;
                    for(int j = 0; j < 4; j++){
                        points.push_back(laserCloud->points[i + j]);
                        Eigen::Vector3f temp{laserCloud->points[i + j].x - laserCloud->points[i + j + 1].x,
                                             laserCloud->points[i + j].y - laserCloud->points[i + j + 1].y,
                                             laserCloud->points[i + j].z - laserCloud->points[i + j + 1].z};
                        double temp_dist = temp.norm();
                        if(temp_dist > max_dist) max_dist = temp_dist;
                        if(temp_dist < min_dist) min_dist = temp_dist;                      
                    }

                    Eigen::Vector4f centroid; // 质心
                    Eigen::Matrix3f covariance; // 协方差
                    pcl::compute3DCentroid(points, centroid);
                    pcl::computeCovarianceMatrix(points, centroid, covariance);
                    // 计算协方差矩阵的特征值
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(covariance);
                    Eigen::Vector3f eigenValues = saes.eigenvalues(); // 默认升序
                    bool is_plane = (eigenValues[1] > 100 * eigenValues[0]);

                    if(is_plane
                        && (max_dist < 2 * min_dist) 
                        && surf_dist < 0.5 // 第4个点到当前点的距离
                        && cos_beta < 0.8
                    ){
                        // ROS_INFO("%d", i);
                        if(depth_right > depth_left){
                            cloudFeatureFlag[i] = TYPE_BREAK_POINT;
                        }
                        else{
                            if(depth_right == 0) cloudFeatureFlag[i] = TYPE_BREAK_POINT;
                        }
                    }
                    
                }
            }
        }

        // 对特征进行汇总
        for(size_t i = 0; i < cloudSize; i++){
            if(cloudFeatureFlag[i] == TYPE_SURF)
                pointSurf.push_back(laserCloud->points[i]);
            else if(cloudFeatureFlag[i] == TYPE_CORNER || cloudFeatureFlag[i] == TYPE_BREAK_POINT)
                pointEdge.push_back(laserCloud->points[i]);
        }
        
        // std::cout << "--------------------------\n"
        //             << "All points : " << cloudSize << std::endl
        //             << "Edge points: " << pointEdge.size() << std::endl
        //             << "Surf points: " << pointSurf.size() << std::endl;

        // 发布点云消息
        // sensor_msgs::PointCloud2 laserCloudMsgOut;
        // pcl::toROSMsg(*laserCloud, laserCloudMsgOut);
        // laserCloudMsgOut.header.stamp = laserCloudMsg->header.stamp;
        // laserCloudMsgOut.header.frame_id = laserCloudMsg->header.frame_id;
        // pub_laser_cloud.publish(laserCloudMsgOut);

        // sensor_msgs::PointCloud2 laserSurfCloudMsgOut;
        // pcl::toROSMsg(pointSurf, laserSurfCloudMsgOut);
        // laserSurfCloudMsgOut.header.stamp = laserCloudMsg->header.stamp;
        // laserSurfCloudMsgOut.header.frame_id = laserCloudMsg->header.frame_id;
        // pub_surf_cloud.publish(laserSurfCloudMsgOut);

        // sensor_msgs::PointCloud2 laserEdgeCloudMsgOut;
        // pcl::toROSMsg(pointEdge, laserEdgeCloudMsgOut);
        // laserEdgeCloudMsgOut.header.stamp = laserCloudMsg->header.stamp;
        // laserEdgeCloudMsgOut.header.frame_id = laserCloudMsg->header.frame_id;
        // pub_edge_cloud.publish(laserEdgeCloudMsgOut);

    }

public:
    ScanRegistration()=default;
};

#endif // SCAN_REGISTRATION_HPP_