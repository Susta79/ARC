#ifndef TRAJECTORY_SCURVE_H
#define TRAJECTORY_SCURVE_H

#include "global.h"
#include "trajectory.h"
#include "path.h"

class Trajectory_scurve : public Trajectory {
    // S-Curve divided in 7 segments
protected:
    // Max acceleration in m/s^2
    double Amax;
    // Max jerk in m/s^3
    double Jmax;
    // Delta time
    double dt1;
    double dt2;
    double dt3;
    double dt4;
    double dt5;
    double dt6;
    double dt7;
    // Distance
    double d1;
    double d2;
    double d3;
    double d4;
    double d5;
    double d6;
    double d7;
    // Speed at the end of each segment
    double v1;
    double v2;
    double v3;
    double v4;
    double v5;
    double v6;
    double v7;

public:
    Trajectory_scurve(double Vmax, double Amax, double Jmax, Path *path);
    //~Trajectory_scurve();
    ARCCode_t process();

    // Amax
    double get_Amax(){ return this->Amax; }
    void set_Amax(double Amax){ if (Amax > 0.1) this->Amax = Amax; else this->Amax = 0.1; }

    // Jmax
    double get_Jmax(){ return this->Jmax; }
    void set_Jmax(double Jmax){ if (Jmax > 0.1) this->Jmax = Jmax; else this->Jmax = 0.1; }

    // virtual functions
    double get_dist_at_t(double t);
};

#endif // TRAJECTORY_SCURVE_H