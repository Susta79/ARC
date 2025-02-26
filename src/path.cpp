#include "path.h"

Path::Path(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d euler0, Eigen::Vector3d euler1, double N, double dt, bool in_mm, bool in_rads){
    this->set_P0(P0, in_mm);
    this->set_P1(P1, in_mm);
    this->set_euler0(euler0, in_rads);
    this->set_euler1(euler1, in_rads);
    this->set_N(N);
    this->set_dt(dt);
    s.setLinSpaced(N,0,1);
}

//Path::Path(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d euler0, Eigen::Vector3d euler1, bool in_mm, bool in_rads){
//    this->set_P0(P0, in_mm);
//    this->set_P1(P1, in_mm);
//    this->set_euler0(euler0, in_rads);
//    this->set_euler1(euler1, in_rads);
//}
