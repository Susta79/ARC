#include "trajectory_scurve.h"

Trajectory_scurve::Trajectory_scurve(double Vmax, double Amax, double Jmax, Path *path) : Trajectory(Vmax, path){
    ARCCode_t ret;
    this->set_Amax(Amax);
    this->set_Jmax(Jmax);
    // Delta time for each segment
    this->dt1 = 0.0;
    this->dt2 = 0.0;
    this->dt3 = 0.0;
    this->dt4 = 0.0;
    this->dt5 = 0.0;
    this->dt6 = 0.0;
    this->dt7 = 0.0;
    // Distance done in each segment
    this->d1 = 0.0;
    this->d2 = 0.0;
    this->d3 = 0.0;
    this->d4 = 0.0;
    this->d5 = 0.0;
    this->d6 = 0.0;
    this->d7 = 0.0;
    // Speed at the end of each segment
    this->v1 = 0.0;
    this->v2 = 0.0;
    this->v3 = 0.0;
    this->v4 = 0.0;
    this->v5 = 0.0;
    this->v6 = 0.0;
    this->v7 = 0.0;
    ret = this->process();
}

// TODO: Implement this function
double Trajectory_scurve::get_dist_at_t(double t){
    double dist, dt;
    if (t <= 0) {
        dist = 0.0;
    } else if (t <= this->dt1) {
        dist = (1.0 / 6.0) * this->Jmax * pow(t, 3);
    } else if (t <= this->dt2) {
        dt = t - this->dt1;
        dist = this->v1 * dt + (1.0 / 2.0) * this->Amax * pow(dt, 2);
        dist += this->d1;
    } else if (t <= this->dt3) {
        dt = t - this->dt1 - this->dt2;
        dist = (this->Vmax  - (1.0 / 2.0) * (pow(this->Amax, 2) / this->Jmax)) * dt - (1.0 / 6.0) * this->Jmax * pow(dt, 3) + (1.0 / 2.0) * this->Amax * pow(t, 2);
        dist += this->d1 + this->d2;
    } else if (t <= this->dt4) {
        dt = t - this->dt1 - this->dt2 - this->dt3;
        dist = this->Vmax * dt;
        dist += this->d1 + this->d2 + this->d3;
    } else if (t <= this->dt5) {
        dt = t - this->dt1 - this->dt2 - this->dt3 - this->dt4;
        dist = this->Vmax * dt - (1.0 / 6.0) * this->Jmax * pow(dt, 3);
        dist += this->d1 + this->d2 + this->d3 + this->d4;
    } else if (t <= this->dt6) {
        dt = t - this->dt1 - this->dt2 - this->dt3 - this->dt4 - this->dt5;
        dist = (this->Vmax - this->v1) * dt - (1.0 / 2.0) * this->Amax * pow(dt, 2);
        dist += this->d1 + this->d2 + this->d3 + this->d4 + this->d5;
    } else if (t <= this->dt7) {
        dt = t - this->dt1 - this->dt2 - this->dt3 - this->dt4 - this->dt5 - this->dt6;
        dist = (this->Vmax - this->v1) * dt - (1.0 / 2.0) * this->Amax * pow(dt, 2);
        dist += this->d1 + this->d2 + this->d3 + this->d4 + this->d5 + this->d6;
    } else if (t > this->dt7) {
        dist = this->d1 + this->d2 + this->d3 + this->d4 + this->d5 + this->d6 + this->d7;
    }
    return dist;
}

ARCCode_t Trajectory_scurve::process(){
    ARCCode_t ret;
    double L;
    ret = this->path->path_lenght(&L);
    if (ret != ARC_CODE_OK)
        return ret;
    // Segment 1: Jerk positive constant
    this->dt1 = this->Amax / this->Jmax;
    this->v1 = (1.0 / 2.0) * this->Jmax * pow(dt1, 2);
    this->d1 = (1.0 / 6.0) * this->Jmax * pow(dt1, 3);
    // Segment 2:
    this->dt2 = (this->Vmax / this->Amax) - (this->Amax / this->Jmax);
    this->v2 = this->v1 + this->Amax * this->dt2;
    this->d2 = this->v1 * this->dt2 + (1.0 / 2.0) * this->Amax * pow(this->dt2, 2);
    // Segment 3:
    this->dt3 = this->Amax / this->Jmax;
    this->v3 = this->Vmax;
    this->d3 = this->Vmax * this->dt1 - (1.0 / 6.0) * this->Jmax * pow(dt1, 3);
    // Segment 5:
    this->dt5 = this->dt3;
    this->v5 = this->v2;
    this->d5 = this->d3;
    // Segment 6:
    this->dt6 = this->dt2;
    this->v6 = this->v1;
    this->d6 = this->d2;
    // Segment 7:
    this->dt7 = this->dt1;
    this->v7 = 0;
    this->d7 = this->d1;
    // Segment 4:
    this->d4 = (L/1000.0) - (this->dt1 + this->dt2 + this->dt3 + this->dt5 + this->dt6 + this->dt7);
    this->dt4 = this->Vmax * this->d4;
    this->v4 = this->Vmax;
    // Total time
    this->set_T(this->dt1 + this->dt2 + this->dt3 + this->dt4 + this->dt5 + this->dt6 + this->dt7);
    //
    std::cout << "dt1: " << this->dt1 << std::endl;
    std::cout << "dt2: " << this->dt2 << std::endl;
    std::cout << "dt3: " << this->dt3 << std::endl;
    std::cout << "dt4: " << this->dt4 << std::endl;
    std::cout << "dt5: " << this->dt5 << std::endl;
    std::cout << "dt6: " << this->dt6 << std::endl;
    std::cout << "dt7: " << this->dt7 << std::endl;
    std::cout << "d1: " << this->d1 << std::endl;
    std::cout << "d2: " << this->d2 << std::endl;
    std::cout << "d3: " << this->d3 << std::endl;
    std::cout << "d4: " << this->d4 << std::endl;
    std::cout << "d5: " << this->d5 << std::endl;
    std::cout << "d6: " << this->d6 << std::endl;
    std::cout << "d7: " << this->d7 << std::endl;
    std::cout << "v1: " << this->v1 << std::endl;
    std::cout << "v2: " << this->v2 << std::endl;
    std::cout << "v3: " << this->v3 << std::endl;
    std::cout << "v4: " << this->v4 << std::endl;
    std::cout << "v5: " << this->v5 << std::endl;
    std::cout << "v6: " << this->v6 << std::endl;
    std::cout << "v7: " << this->v7 << std::endl;
    return ARC_CODE_OK;
}