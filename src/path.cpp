#include "path.h"

Eigen::MatrixXd Path::slerp(bool in_rads, bool out_rads){
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