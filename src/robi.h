#ifndef ROBI_H
#define ROBI_H

#include <include/ARC/error_def.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>

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


    // Basic functions
    Eigen::Matrix3d rotx(double angle);
    Eigen::Matrix3d roty(double angle);
    Eigen::Matrix3d rotz(double angle);
    double rads(double angle);
    double degs(double angle);
    Eigen::Affine3d trans_mat(Eigen::Matrix3d rot, Eigen::Vector3d trans);
    // Orientation functions
    Eigen::Matrix3d rot_mat_from_euler(Eigen::Vector3d euler, bool in_rads);
    Eigen::Quaterniond q_from_rot_mat(Eigen::Matrix3d R);
    Eigen::Matrix3d rot_mat_from_q(Eigen::Quaterniond q);
    Eigen::Vector3d euler_from_rot_mat(Eigen::Matrix3d R, bool out_rads);
    double alpha_slerp(Eigen::Quaterniond q1, Eigen::Quaterniond q2, bool out_rads);
    Eigen::MatrixXd slerp(Eigen::Vector3d euler_array1, Eigen::Vector3d euler_array2, Eigen::VectorXd s, bool in_rads, bool out_rads);
    //path functions
    Eigen::Vector3d circumcenter(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    Eigen::Vector3d circumcenter2(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    Eigen::Vector3d circumcenter3(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    double circumradius(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    ARCCode_t circum_alpha(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, double *alpha);
    //def circular_path_lambda(CC, CR,U,V):
    //    return lambda theta: np.array([CC + CR*np.cos(theta)*U + CR*np.sin(theta)*V])
    Eigen::VectorXd linspace(double start, double end, int num);
    ARCCode_t circle_params(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector3d *CC, double *CR, Eigen::Vector3d *U, Eigen::Vector3d *V, double *alpha);
    double arc_length(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    Eigen::MatrixXd circular_path(Eigen::Vector3d CC, double CR, Eigen::Vector3d U, Eigen::Vector3d V, Eigen::VectorXd s);
    // Robot object
    ARCCode_t for_kin(Eigen::Array<double, 6, 1> joint, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> *xyzabc, Eigen::Affine3d *t16);
    // inv_kin
    ARCCode_t inv_kin(Eigen::Array<double, 6, 1> xyzabc, bool front_pose, bool up_pose, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> *joint);
    ARCCode_t Jacobian(Eigen::Array<double, 6, 1> joint, bool in_rads, Eigen::Matrix<double, 6, 6> *J);
    void init(double a1z, double a2x, double a2z, double a3z, double a4z, double a4x, double a5x, double a6x);

public:
    Robi();
    Robi(double a1z, double a2x, double a2z, double a3z, double a4z, double a4x, double a5x, double a6x);
    ~Robi();

    void test();
};

#endif // ROBI_H