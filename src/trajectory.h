#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "global.h"
#include "path.h"

class Trajectory {
protected:
    // Max speed in m/s
    double Vmax;
    // Calculated total traversing time in seconds 
    double T;
    // Path object. Can be any path type: lineat, circular, spline...
    Path *path;

public:
    Trajectory(double Vmax, Path *path){ this->Vmax = Vmax; this->path = path; };
    ~Trajectory();

    // Vmax
    double get_Vmax(){ return this->Vmax; }
    void set_Vmax(double Vmax){ this->Vmax = Vmax; }

    // T
    double get_T(){ return this->T; }
    void set_T(double T){ this->T = T; }

    // path
    Path *get_path(){ return this->path; }
    void set_path(Path *path){ this->path = path; }

    // virtual functions
    virtual double get_dist_at_t(double t){ return 0.0; };
};

#endif // TRAJECTORY_H