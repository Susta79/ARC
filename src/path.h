#ifndef PATH_H
#define PATH_H

#include "global.h"

class Path{ 
protected:
    // start point coordinates in mm
    Eigen::Vector3d P0;
    // end point coordinates in mm
    Eigen::Vector3d P1;
    // start point orientation in euler angles in degrees
    Eigen::Vector3d euler0;
    // end point orientation in euler angles in degrees
    Eigen::Vector3d euler1;
    Eigen::VectorXd s;
    double N;
    double dt;

public:
    Path(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d euler0, Eigen::Vector3d euler1, double N, double dt, bool in_mm, bool in_rads);
    //Path(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d euler0, Eigen::Vector3d euler1, bool in_mm, bool in_rads);
    //~Path();

    // P0
    Eigen::Vector3d get_P0(bool out_mm){ if (out_mm) return this->P0; else return this->P0/1000.0; }
    void set_P0(Eigen::Vector3d P0, bool in_mm){ if (in_mm) this->P0 = P0; else this->P0 = P0*1000.0; }

    // P1
    Eigen::Vector3d get_P1(bool out_mm){ if (out_mm) return this->P1; else return this->P1/1000.0; }
    void set_P1(Eigen::Vector3d P1, bool in_mm){ if (in_mm) this->P1 = P1; else this->P1 = P1*1000.0; }

    // euler0
    Eigen::Vector3d get_euler0(bool out_rads){ if (out_rads) return this->euler0*DEG_TO_RAD; else return this->euler0; }
    void set_euler0(Eigen::Vector3d euler0, bool in_rads){ if (in_rads) this->euler0 = euler0*RAD_TO_DEG; else this->euler0 = euler0; }

    // euler1
    Eigen::Vector3d get_euler1(bool out_rads){ if (out_rads) return this->euler1*DEG_TO_RAD; else return this->euler1; }
    void set_euler1(Eigen::Vector3d euler1, bool in_rads){ if (in_rads) this->euler1 = euler1*RAD_TO_DEG; else this->euler1 = euler1; }

    // N
    double get_N(){ return this->N; }
    void set_N(double N){ this->N = N; }

    // dt
    double get_dt(){ return this->dt; }
    void set_dt(double dt){ this->dt = dt; }

    // virtual functions
    virtual ARCCode_t path_lenght(double *lenght, bool out_mm) {*lenght = 0.0; return ARC_CODE_OK;};
    virtual Eigen::MatrixXd xyz_array() {return Eigen::MatrixXd(0,0);};
    virtual Eigen::MatrixXd slerp(bool in_rads, bool out_rads) {return Eigen::MatrixXd(0,0);};
    virtual ARCCode_t get_pose_at_s(double s, bool out_mm, bool out_rads, Eigen::Vector<double, 6> *pose) {*pose << 0, 0, 0, 0, 0, 0; return ARC_CODE_OK;};
};

#endif // PATH_H