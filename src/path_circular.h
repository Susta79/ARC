#ifndef PATH_CIRCULAR_H
#define PATH_CIRCULAR_H

#include "path.h"

class Path_circular : public Path
{
protected:
    Eigen::Vector3d P2;
    Eigen::Vector3d CC;
    double CR;
    Eigen::Vector3d U;
    Eigen::Vector3d V;
    double alpha;

    Eigen::Vector3d circumcenter();
    double circumradius();
    double alpha_slerp(Eigen::Quaterniond q1, Eigen::Quaterniond q2, bool out_rads);
    //path functions
    //Eigen::Vector3d circumcenter2(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    //Eigen::Vector3d circumcenter3(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    ARCCode_t circum_alpha(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, double *alpha);    
    Eigen::Vector3d circular_path_lambda(Eigen::Vector3d CC, double CR, Eigen::Vector3d U, Eigen::Vector3d V, double theta);

public:
    // P2
    Eigen::Vector3d get_P2(){ return this->P2; }
    void set_P2(Eigen::Vector3d P2){ this->P2 = P2; }

    // CC
    Eigen::Vector3d get_CC(){ return this->CC; }
    void set_CC(Eigen::Vector3d CC){ this->CC = CC; }

    // CR
    double get_CR(){ return this->CR; }
    void set_CR(double CR){ this->CR = CR; }

    // U
    Eigen::Vector3d get_U(){ return this->U; }
    void set_U(Eigen::Vector3d U){ this->U = U; }

    // V
    Eigen::Vector3d get_V(){ return this->V; }
    void set_V(Eigen::Vector3d V){ this->V = V; }

    // alpha
    double get_alpha(){ return this->alpha; }
    void set_alpha(double alpha){ this->alpha = alpha; }

    ARCCode_t circle_params(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    Path_circular(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector3d euler0, Eigen::Vector3d euler1, double N, double dt);
    ARCCode_t path_lenght(double *arc_length);
    Eigen::MatrixXd xyz_array();
};

#endif // PATH_CIRCULAR_H
