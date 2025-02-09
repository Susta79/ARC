#ifndef PATH_LINEAR_H
#define PATH_LINEAR_H

#include "global.h"
#include "path.h"

class Path_linear : public Path
{
public:
    ARCCode_t path_lenght(double *lenght);
    Eigen::MatrixXd xyz_array();
    Eigen::Vector3d path_lambda(Eigen::Vector3d P0, Eigen::Vector3d P1, double s);
};

#endif // PATH_LINEAR_H
