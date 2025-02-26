#ifndef PATH_LINEAR_H
#define PATH_LINEAR_H

#include "global.h"
#include "path.h"

class Path_linear : public Path
{
public:
    Path_linear(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d euler0, Eigen::Vector3d euler1, double N, double dt, bool in_mm, bool in_rads) : Path(P0, P1, euler0, euler1, N, dt, in_mm, in_rads){};
    ARCCode_t path_lenght(double *lenght, bool out_mm);
    Eigen::MatrixXd xyz_array();
    Eigen::MatrixXd slerp(bool in_rads, bool out_rads);
    Eigen::Vector3d path_lambda(Eigen::Vector3d P0, Eigen::Vector3d P1, double s);
    ARCCode_t get_pose_at_s(double s, bool out_mm, bool out_rads, Eigen::Vector<double, 6> *pose);
};

#endif // PATH_LINEAR_H
