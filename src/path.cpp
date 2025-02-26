#include "path.h"

Path::Path(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d euler0, Eigen::Vector3d euler1, double N, double dt){
    this->P0 = P0;
    this->P1 = P1;
    this->euler0 = euler0;
    this->euler1 = euler1;
    this->N = N;
    this->dt = dt;
    s.setLinSpaced(N,0,1);
}
