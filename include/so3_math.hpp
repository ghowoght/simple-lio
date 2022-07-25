/**
 * @file so3_math.hpp
 * @author Linfu Wei (ghowoght@qq.com)
 * @brief Exp/Log of SO(3) group.
 * @version 1.0
 * @date 2022-07-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef SO3_MATH_HPP
#define SO3_MATH_HPP

#include <Eigen/Core>
#include <math.h>

class SO3Math{
public:
    // 反对称矩阵
    static Eigen::Matrix3d get_skew_symmetric(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d m;
        m <<   0 , -v(2),  v(1),
             v(2),    0 , -v(0),
            -v(1),  v(0),    0 ;
        return m;
    }

    static Eigen::Vector3d quat2euler(const Eigen::Quaterniond& q)
    {
        Eigen::Vector3d euler;
        euler(0) = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(q.x()*q.x()+q.y()*q.y()));
        euler(1) = asin(2*(q.w()*q.y()-q.z()*q.x()));
        euler(2) = atan2(2*(q.w()*q.z()+q.x()*q.y()), 1-2*(q.y()*q.y()+q.z()*q.z()));
        return euler;
    }

    static Eigen::Matrix3d Exp(const Eigen::Vector3d& r){
        Eigen::Matrix3d expr;
        double theta = r.norm();
        if(theta < 1e-8){
            expr = Eigen::Matrix3d::Identity();
        }else{
            Eigen::Matrix3d skew = get_skew_symmetric(r / theta);
            expr = Eigen::Matrix3d::Identity() + sin(theta) * skew + (1 - cos(theta)) * skew * skew;
        }
        return expr;
    }

    static Eigen::Vector3d Log(const Eigen::Matrix3d& R){
        double theta = acos((R.trace() - 1) / 2);
        Eigen::Vector3d r(R(1, 0) - R(0, 1), R(0, 2) - R(2, 0), R(1, 2) - R(2, 1));
        r /= (2 * sin(theta));
        return r;
    }   

};

#endif // SO3_MATH_HPP