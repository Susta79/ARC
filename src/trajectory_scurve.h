#ifndef TRAJECTORY_SCURVE_H
#define TRAJECTORY_SCURVE_H

#include "global.h"
#include "trajectory.h"

class Trajectory_scurve : public Trajectory {
    // S-Curve divided in 7 segments
protected:
    // Max acceleration in m/s^2
    double Amax;
    // Max jerk in m/s^3
    double Jmax;

public:
    Trajectory_scurve(double Vmax, double Amax, double Jmax) : Trajectory(Vmax){ this->Amax = Amax; this->Jmax = Jmax; };
    //~Trajectory_scurve();

    // Amax
    double get_Amax(){ return this->Amax; }
    void set_Amax(double Amax){ this->Amax = Amax; }

    // Jmax
    double get_Jmax(){ return this->Jmax; }
    void set_Jmax(double Jmax){ this->Jmax = Jmax; }

    // virtual functions
    double get_dist_at_t(double t);
};

#endif // TRAJECTORY_SCURVE_H