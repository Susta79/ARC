#ifndef GLOBAL_H
#define GLOBAL_H

#include <include/ARC/error_def.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>

#define RAD_TO_DEG 180.0 / M_PI
#define DEG_TO_RAD M_PI / 180.0

inline Eigen::Matrix3d rotx(double angle, bool in_rads){
    Eigen::Matrix3d R;
    //R << 1, 0, 0,
    //    0, cos(angle), -sin(angle),
    //    0, sin(angle), cos(angle);
    if (in_rads == false)
        angle *= DEG_TO_RAD;
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
    return R;
}

inline Eigen::Matrix3d roty(double angle, bool in_rads){
    Eigen::Matrix3d R;
    //R << cos(angle), 0, sin(angle),
    //    0, 1, 0,
    //    -sin(angle), 0, cos(angle);
    if (in_rads == false)
        angle *= DEG_TO_RAD;
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return R;
}

inline Eigen::Matrix3d rotz(double angle, bool in_rads){
    Eigen::Matrix3d R;
    //R << cos(angle), -sin(angle), 0,
    //    sin(angle), cos(angle), 0,
    //    0, 0, 1;
    if (in_rads == false)
        angle *= DEG_TO_RAD;
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return R;
}

inline Eigen::Affine3d trans_mat(Eigen::Matrix3d rot, Eigen::Vector3d trans, bool in_mm){
    Eigen::Affine3d T;
    T.linear() = rot;
    T.translation() = trans;
    return T;
}

inline Eigen::Matrix3d rot_mat_from_euler(Eigen::Vector3d euler, bool in_rads){
    double a = euler(0);
    double b = euler(1);
    double c = euler(2);
    Eigen::Matrix3d R = rotz(c, in_rads) * roty(b, in_rads) * rotx(a, in_rads);
    return R;
}

inline Eigen::Quaterniond q_from_rot_mat(Eigen::Matrix3d R){
    Eigen::Quaterniond q(R);
    return q;
}

inline Eigen::Matrix3d rot_mat_from_q(Eigen::Quaterniond q){
    Eigen::Matrix3d R;
    R = q.normalized().toRotationMatrix();
    return R;
}

inline Eigen::Vector3d euler_from_rot_mat(Eigen::Matrix3d R, bool out_rads){
    double a = atan2(R(2,1),R(2,2));
    double b = atan2(-R(2,0), sqrt(1-pow(R(2,0),2)));
    double c = atan2( R(1,0),R(0,0));
    Eigen::Vector3d euler;
    euler << a, b, c;
    if (out_rads == false)
        euler *= RAD_TO_DEG;
    return euler;
}

#endif // GLOBAL_H