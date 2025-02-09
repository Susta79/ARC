#ifndef ROBI_H
#define ROBI_H

#include "global.h"

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

    // code for optimum trajectory
    ARCCode_t interp_value(double si, Eigen::ArrayXd s_array, Eigen::ArrayXd val_array, double *val, int *idx);
    ARCCode_t sd_limits_J(Eigen::ArrayXd s_array, Eigen::ArrayXd speed_limits, Eigen::MatrixXd xyzabcs0, std::function<Eigen::Vector3d(double)> path_func, Eigen::Vector3d euler1, Eigen::Vector3d euler2, Robi *robot_obj, double lr, int k, double alpha, bool front_pose, bool up_pose);

    // Robot object
    ARCCode_t for_kin(Eigen::Array<double, 6, 1> joint, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> *xyzabc, Eigen::Affine3d *t16);
    ARCCode_t inv_kin(Eigen::Array<double, 6, 1> xyzabc, bool front_pose, bool up_pose, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> *joint);
    ARCCode_t Jacobian(Eigen::Array<double, 6, 1> joint, bool in_rads, Eigen::Matrix<double, 6, 6> *J);
    void init(double a1z, double a2x, double a2z, double a3z, double a4z, double a4x, double a5x, double a6x);

public:
    Robi();
    Robi(double a1z, double a2x, double a2z, double a3z, double a4z, double a4x, double a5x, double a6x);
    ~Robi();
};

#endif // ROBI_H