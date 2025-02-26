#ifndef PATH_CIRCULAR_H
#define PATH_CIRCULAR_H

#include "path.h"

class Path_circular : public Path
{
protected:
    // P2 in mm
    Eigen::Vector3d P2;
    // euler2 in degrees
    Eigen::Vector3d euler2;
    Eigen::Vector3d CC;
    double CR;
    Eigen::Vector3d U;
    Eigen::Vector3d V;
    // alpha: Angle in radians between (P2-CC) and (P0-CC)
    double alpha;
    // alpha1: Angle in radians between (P1-CC) and (P0-CC)
    double alpha1;

    Eigen::Vector3d circumcenter();
    double circumradius();
    // TODO: Divide slerp into 2 or 3 parts
    double alpha_slerp(Eigen::Quaterniond q1, Eigen::Quaterniond q2, bool out_rads);
    //path functions
    //Eigen::Vector3d circumcenter2(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    //Eigen::Vector3d circumcenter3(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2);
    ARCCode_t circum_alpha(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, double *alpha);    
    Eigen::Vector3d circular_path_lambda(Eigen::Vector3d CC, double CR, Eigen::Vector3d U, Eigen::Vector3d V, double theta);

public:
    // P2
    Eigen::Vector3d get_P2(bool out_mm){ if(out_mm) return this->P2; else return this->P2/1000.0; }
    void set_P2(Eigen::Vector3d P2, bool in_mm){ if(in_mm) this->P2 = P2; else this->P2 = P2*1000.0; }

    // euler2
    Eigen::Vector3d get_euler2(bool out_rads){ if(out_rads) return this->euler2*DEG_TO_RAD; else return this->euler2; }
    void set_euler2(Eigen::Vector3d euler2, bool in_rads){ if(in_rads) this->euler2 = euler2*RAD_TO_DEG; else this->euler2 = euler2; }

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
    Path_circular(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector3d euler0, Eigen::Vector3d euler1, Eigen::Vector3d euler2, double N, double dt, bool in_mm, bool in_rads);
    ARCCode_t path_lenght(double *arc_length, bool out_mm);
    Eigen::MatrixXd xyz_array();
    Eigen::MatrixXd slerp(bool in_rads, bool out_rads);
    ARCCode_t get_pose_at_s(double s, bool out_rads, Eigen::Vector<double, 6> *pose);
};

#endif // PATH_CIRCULAR_H
