#include "path_linear.h"

ARCCode_t Path_linear::path_lenght(double *lenght, bool out_mm)
{
    if (out_mm == false)
        *lenght = (this->P1 - this->P0).norm()/1000.0;
    else
        *lenght = (this->P1 - this->P0).norm();
    return ARC_CODE_OK;
}

Eigen::MatrixXd Path_linear::xyz_array(){
    size_t len_s = this->s.size();
    Eigen::Vector3d xyz;
    Eigen::MatrixXd xyz_array(len_s,3);
    for (size_t i = 0; i < len_s; i++){
        xyz = this->P0*(1 - this->s(i)) + this->P1 * this->s(i);
        xyz_array.row(i) << xyz(0), xyz(1), xyz(2);
    }
    return xyz_array;
}

Eigen::MatrixXd Path_linear::slerp(bool in_rads, bool out_rads){
    Eigen::Quaterniond q0, q1, qSlerp;
    Eigen::Vector3d euler;
    Eigen::Matrix3d R, R0, R1;
    size_t len_s;

    R0 = rot_mat_from_euler(this->euler0, in_rads = in_rads);
    R1 = rot_mat_from_euler(this->euler1, in_rads = in_rads);

    q0 = q_from_rot_mat(R0);
    q1 = q_from_rot_mat(R1);
    
    len_s = s.size();
    Eigen::MatrixXd euler_array(len_s,3);
    for (size_t i = 0; i < len_s; i++){
        qSlerp = q0.slerp(s(i), q1);
        R = rot_mat_from_q(qSlerp);
        euler = euler_from_rot_mat(R, out_rads);
        euler_array.row(i) << euler(0), euler(1), euler(2);
    }

    if (out_rads == false)
        euler_array *= RAD_TO_DEG;
    
    return euler_array;
}

Eigen::Vector3d Path_linear::path_lambda(Eigen::Vector3d P0, Eigen::Vector3d P1, double s){
    return P0*(1-s)+ P1*s;
}

ARCCode_t Path_linear::get_pose_at_s(double s, bool out_mm, bool out_rads, Eigen::Vector<double, 6> *pose){
    Eigen::Quaterniond q0, q1, qSlerp;
    Eigen::Vector3d P, euler;
    Eigen::Matrix3d R, R0, R1;

    P = this->P0*(1-s)+ this->P1*s;

    R0 = rot_mat_from_euler(this->euler0, false);
    R1 = rot_mat_from_euler(this->euler1, false);

    q0 = q_from_rot_mat(R0);
    q1 = q_from_rot_mat(R1);
    
    qSlerp = q0.slerp(s, q1);
    R = rot_mat_from_q(qSlerp);
    euler = euler_from_rot_mat(R, out_rads);

    if (out_mm == false)
        P /= 1000.0;

    //if (out_rads == false)
    //    euler *= RAD_TO_DEG;
    
    *pose << P, euler;

    return ARC_CODE_OK;
}
