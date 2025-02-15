#ifndef PATH_LINEAR_H
#define PATH_LINEAR_H

#include "global.h"
#include "path.h"

class Path_linear : public Path
{
public:
    Path_linear(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d euler0, Eigen::Vector3d euler1, double N, double dt) : Path(P0, P1, euler0, euler1, N, dt){};
    ARCCode_t path_lenght(double *lenght);
    Eigen::MatrixXd xyz_array();
    Eigen::MatrixXd slerp(bool in_rads, bool out_rads);
    Eigen::Vector3d path_lambda(Eigen::Vector3d P0, Eigen::Vector3d P1, double s);
};

#endif // PATH_LINEAR_H
