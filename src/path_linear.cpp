#include "path_linear.h"

ARCCode_t Path_linear::path_lenght(double *lenght)
{
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

Eigen::Vector3d Path_linear::path_lambda(Eigen::Vector3d P0, Eigen::Vector3d P1, double s){
    return P0*(1-s)+ P1*s;
}