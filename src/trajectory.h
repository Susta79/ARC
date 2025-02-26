#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "global.h"
#include "path.h"

class Trajectory {
protected:
    // Max speed in m/s
    double V;
    // Calculated total traversing time in seconds 
    double T;
    // Path object. Can be any path type: lineat, circular, spline...
    Path *path;

public:
    Trajectory(double V, Path *path);
    ~Trajectory();

    // V
    double get_V(){ return this->V; }
    void set_V(double V){ if(V > 0.000001) this->V = V; else this->V = 0.000001; }

    // T
    double get_T(){ return this->T; }
    void set_T(double T){ if (T > 0) this->T = T; else this->T = 0.0; }

    // path
    Path *get_path(){ return this->path; }
    void set_path(Path *path){ this->path = path; }

    // virtual functions
    virtual double get_dist_at_t(double t){ return 0.0; };
};

#endif // TRAJECTORY_H