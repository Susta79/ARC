#ifndef TRAJECTORY_SCURVE_H
#define TRAJECTORY_SCURVE_H

#include "global.h"
#include "trajectory.h"
#include "path.h"
#include <QDebug>

class Trajectory_scurve : public Trajectory {
    // S-Curve divided in 7 segments
protected:
    // Max acceleration in m/s^2
    double A;
    // Max jerk in m/s^3
    double J;
    // Delta time
    double T1;
    double T2;
    double T3;
    double T4;
    double T5;
    double T6;
    double T7;
    // Distance
    double D1;
    double D2;
    double D3;
    double D4;
    double D5;
    double D6;
    double D7;
    // Speed at the end of each segment
    double V1;
    double V2;
    double V3;
    double V4;
    double V5;
    double V6;
    double V7;

public:
    Trajectory_scurve(double V, double A, double J, Path *path);
    //~Trajectory_scurve();
    ARCCode_t process();

    // A
    double get_A(){ return this->A; }
    void set_A(double A){ if (A > 0.000001) this->A = A; else this->A = 0.000001; }

    // J
    double get_J(){ return this->J; }
    void set_J(double J){ if (J > 0.000001) this->J = J; else this->J = 0.000001; }

    // virtual functions
    double get_dist_at_t(double t);
};

#endif // TRAJECTORY_SCURVE_H