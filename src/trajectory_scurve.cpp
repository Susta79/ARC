#include "trajectory_scurve.h"

// TODO: Implement this function
double Trajectory_scurve::get_dist_at_t(double t){
    double dist = 0.0;
    double dt;
    if (t <= 0)
        return 0.0;
    if (t <= this->dt1) {
        dist = (1.0 / 6.0) * this->Jmax * pow(t, 3);
        return dist;
    }
    if (t <= this->dt2) {
        dt = t - this->dt1;
        dist = this->v1 * dt + (1.0 / 2.0) * this->Amax * pow(dt, 2);
        return dist;
    }
    if (t <= this->dt3) {
        dt = t - this->dt1 - this->dt2;
        dist = this->d1 + this->d2 + this->Vmax * this->dt1 - (1.0 / 6.0) * this->Jmax * pow(dt, 3);
        return dist;
    }
    if (t <= this->dt4) {
        dt = t - this->dt1 - this->dt2 - this->dt3;
        dist = this->d1 + this->d2 + this->d3 + this->Vmax * dt;
        return dist;
    }
    // TODO: Complete
    return dist;
}

void Trajectory_scurve::process(){
    ARCCode_t ret;
    double L;
    if (Jmax < 0.001){
        // TODO: return an error code
        return;
    }
    ret = this->path->path_lenght(&L);
    // Segment 1: Jerk positive constant
    this->dt1 = this->Amax / this->Jmax;
    this->d1 = (1.0 / 6.0) * this->Jmax * pow(dt1, 3);
    this->v1 = (1.0 / 2.0) * this->Jmax * pow(dt1, 2);
    // Segment 2:
    this->dt2 = (this->Vmax / this->Amax) - (this->Amax / this->Jmax);
    this->d2 = this->v1 * this->dt2 + (1.0 / 2.0) * this->Amax * pow(this->dt2, 2);
    this->v2 = this->v1 + this->Amax * this->dt1;
    // Segment 3:
    this->dt3 = this->Amax / this->Jmax;
    this->d3 = this->Vmax * this->dt1 - (1.0 / 6.0) * this->Jmax * pow(dt1, 3);
    this->v3 = this->Vmax;
    // Segment 5:
    this->dt5 = this->dt3;
    this->d5 = this->d3;
    this->v5 = this->v2;
    // Segment 6:
    this->dt6 = this->dt2;
    this->d6 = this->d2;
    this->v5 = this->v1;
    // Segment 7:
    this->dt7 = this->dt1;
    this->d7 = this->d1;
    this->v7 = 0;
    // Segment 4:
    this->d4 = L - (this->dt1 + this->dt2 + this->dt3 + this->dt5 + this->dt6 + this->dt7);
    this->dt4 = this->Vmax * this->dt4;
    this->v4 = this->Vmax;
}