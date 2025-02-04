#ifndef ROBI_H
#define ROBI_H

#include <include/ARC/error_def.h>

#include <Eigen/Dense>

class Robi
{
private:
    double a1z;
    double a2x;
    double a2z;
    double a3z;
    double a4z;
    double a4x;
    double a5x;
    double a6x;

    Eigen::Matrix3d rotx(double angle){
        Eigen::Matrix3d R;
        R << 1, 0, 0,
            0, cos(angle), -sin(angle),
            0, sin(angle), cos(angle);
        return R;
    }

    Eigen::Matrix3d roty(double angle){
        Eigen::Matrix3d R;
        R << cos(angle), 0, sin(angle),
            0, 1, 0,
            -sin(angle), 0, cos(angle);
        return R;
    }

    Eigen::Matrix3d rotz(double angle){
        Eigen::Matrix3d R;
        R << cos(angle), -sin(angle), 0,
            sin(angle), cos(angle), 0,
            0, 0, 1;
        return R;
    }

    double rads(double angle){ return angle * M_PI / 180; }
    double degs(double angle){ return angle * 180 / M_PI; }
    Eigen::Vector3d vec_norm(Eigen::Vector3d vec){ return vec.normalized(); }
    double vec_len(Eigen::Vector3d vec){ return vec.norm(); }

    Eigen::Affine3d trans_mat(Eigen::Matrix3d rot, Eigen::Vector3d trans){
        Eigen::Affine3d T;
        T.linear() = rot;
        T.translation() = trans;
        return T;
    }
    
public:
    Robi(double a1z, double a2x, double a2z, double a3z, double a4z, double a4x, double a5x, double a6x){
        this->a1z = a1z;
        this->a2x = a2x;
        this->a2z = a2z;
        this->a3z = a3z;
        this->a4z = a4z;
        this->a4x = a4x;
        this->a5x = a5x;
        this->a6x = a6x;
    }
    //~Robi();

    ARCCode_t for_kin(Eigen::Array<double, 6, 1> joint, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> &xyzabc, Eigen::Affine3d &t16);
    ARCCode_t inv_kin(Eigen::Array<double, 6, 1> xyzabc, bool front_pose, bool up_pose, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> &joint);
    ARCCode_t Jacobian(Eigen::Array<double, 6, 1> joint, bool in_rads, Eigen::Matrix<double, 6, 6> &J);
};

#endif // ROBI_H