#ifndef PATH_H
#define PATH_H

#include "global.h"

class Path{ 
protected:
    Eigen::Vector3d P0;
    Eigen::Vector3d P1;
    Eigen::Vector3d euler0;
    Eigen::Vector3d euler1;
    Eigen::VectorXd s;
    double N;
    double dt;

public:
    //Path();
    Path(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d euler0, Eigen::Vector3d euler1, double N, double dt){
        this->P0 = P0;
        this->P1 = P1;
        this->euler0 = euler0;
        this->euler1 = euler1;
        this->N = N;
        this->dt = dt;
        s.setLinSpaced(N,0,1);
    }
    //~Path();

    // P0
    Eigen::Vector3d get_P0(){ return this->P0; }
    void set_P0(Eigen::Vector3d P0){ this->P0 = P0; }

    // P1
    Eigen::Vector3d get_P1(){ return this->P1; }
    void set_P1(Eigen::Vector3d P1){ this->P1 = P1; }

    // euler0
    Eigen::Vector3d get_euler0(){ return this->euler0; }
    void set_euler0(Eigen::Vector3d euler0){ this->euler0 = euler0; }

    // euler1
    Eigen::Vector3d get_euler1(){ return this->euler1; }
    void set_euler1(Eigen::Vector3d euler1){ this->euler1 = euler1; }

    // N
    double get_N(){ return this->N; }
    void set_N(double N){ this->N = N; }

    // dt
    double get_dt(){ return this->dt; }
    void set_dt(double dt){ this->dt = dt; }

    Eigen::MatrixXd slerp(bool in_rads, bool out_rads);

    // virtual functions
    virtual ARCCode_t path_lenght(double *lenght) {*lenght = 0.0; return ARC_CODE_OK;};
    virtual Eigen::MatrixXd xyz_array() {return Eigen::MatrixXd(0,0);};
    //virtual Eigen::MatrixXd slerp(bool in_rads, bool out_rads);
};

#endif // PATH_H